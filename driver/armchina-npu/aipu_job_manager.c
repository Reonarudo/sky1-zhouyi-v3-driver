// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/file.h>
#include <linux/fs.h>
#include "aipu_job_manager.h"
#include "aipu_priv.h"
#include "aipu_common.h"
#include "aipu_partition.h"
#include "zhouyi.h"
#include "v3.h"

static struct aipu_thread_wait_queue *do_create_thread_wait_queue(int uthread_id, struct file *filp)
{
	struct aipu_thread_wait_queue *new_wait_queue =
		kzalloc(sizeof(*new_wait_queue), GFP_KERNEL);

	if (unlikely(!new_wait_queue))
		return ERR_PTR(-ENOMEM);
	new_wait_queue->ref_cnt = 0;
	new_wait_queue->uthread_id = uthread_id;
	new_wait_queue->filp = filp;
	init_waitqueue_head(&new_wait_queue->p_wait);
	INIT_LIST_HEAD(&new_wait_queue->node);
	return new_wait_queue;
}

static struct aipu_thread_wait_queue *get_thread_wait_queue(struct aipu_thread_wait_queue *head,
							    int uthread_id, struct file *filp)
{
	struct aipu_thread_wait_queue *curr = NULL;

	if (unlikely(!head))
		return ERR_PTR(-EINVAL);

	list_for_each_entry(curr, &head->node, node) {
		if ((curr->uthread_id == uthread_id && uthread_id) ||
		    (curr->filp == filp && filp))
			return curr;
		}
	return ERR_PTR(-EINVAL);
}

static struct aipu_thread_wait_queue *create_thread_wait_queue(struct aipu_thread_wait_queue *head,
							       int uthread_id, struct file *filp)
{
	struct aipu_thread_wait_queue *queue = get_thread_wait_queue(head, uthread_id, filp);

	if (IS_ERR(queue)) {
		queue = do_create_thread_wait_queue(uthread_id, filp);
		if (!IS_ERR(queue) && head)
			list_add_tail(&queue->node, &head->node);
		else
			return queue;
	}
	return queue;
}

static void delete_wait_queue(struct aipu_thread_wait_queue **wait_queue_head)
{
	struct aipu_thread_wait_queue *curr = NULL;
	struct aipu_thread_wait_queue *next = NULL;

	if (wait_queue_head && *wait_queue_head) {
		list_for_each_entry_safe(curr, next, &(*wait_queue_head)->node, node) {
			list_del(&curr->node);
			kfree(curr);
		}
		kfree(*wait_queue_head);
		*wait_queue_head = NULL;
	}
}

/*
 * a queue registered in a sleeping poller's poll table outlives
 * the poll_wait() call: &wq->p_wait stays referenced until the poller leaves
 * poll(). Freeing such a queue is a use-after-free on the poller's wakeup or
 * poll-table teardown, so a queue is only freeable when its p_wait
 * has no waiters — checked under p_wait's own lock so a mid-removal
 * remove_wait_queue() cannot be caught halfway.
 */
static bool wait_queue_in_use(struct aipu_thread_wait_queue *queue)
{
	unsigned long flags;
	bool in_use = false;

	spin_lock_irqsave(&queue->p_wait.lock, flags);
	in_use = !list_empty(&queue->p_wait.head);
	spin_unlock_irqrestore(&queue->p_wait.lock, flags);

	return in_use;
}

static void delete_thread_wait_queue(struct aipu_thread_wait_queue *head,
				     int uthread_id, struct file *filp)
{
	struct aipu_thread_wait_queue *curr = NULL;
	struct aipu_thread_wait_queue *next = NULL;

	if (unlikely(!head))
		return;
	list_for_each_entry_safe(curr, next, &head->node, node) {
		/*
		 * this swept away every zero-ref queue and
		 * ignored both scoping arguments, so any fd's release freed
		 * other processes' queues. Honour them now, keyed
		 * the same way the queues are created and looked up: by fd
		 * for enable_poll_opt queues (uthread_id 0, filp set), by
		 * thread otherwise (filp NULL, uthread_id set). A queue
		 * another fd's poll table still references is never freed
		 * here; it falls to a later close or to delete_wait_queue()
		 * at teardown.
		 */
		if (!curr->ref_cnt &&
		    ((curr->uthread_id == uthread_id && uthread_id) ||
		     (curr->filp == filp && filp)) &&
		    !wait_queue_in_use(curr)) {
			list_del(&curr->node);
			kfree(curr);
		}
	}
}

static void delete_wait_node(struct aipu_thread_wait_queue **wait_queue_head,
			     wait_queue_head_t *wait_node)
{
	struct aipu_thread_wait_queue *job_queue = NULL;

	job_queue =
		container_of(wait_node, struct aipu_thread_wait_queue, p_wait);

	job_queue->ref_cnt--;
}

static int init_aipu_job(struct aipu_job_manager *manager, struct aipu_job *job,
			 struct aipu_job_desc *desc, struct aipu_thread_wait_queue *queue,
			 struct file *filp)
{
	if (unlikely(!job))
		return -EINVAL;

	if (likely(desc))
		job->desc = *desc;
	else
		memset(&job->desc, 0, sizeof(job->desc));

	if (queue) {
		job->thread_queue = &queue->p_wait;
		queue->ref_cnt++;
	} else {
		job->thread_queue = NULL;
	}

	job->filp = filp;
	job->uthread_id = task_pid_nr(current);
	job->core_id = -1;
	job->state = AIPU_JOB_STATE_IDLE;
	INIT_LIST_HEAD(&job->node);
	job->sched_time = ns_to_ktime(0);
	job->done_time = ns_to_ktime(0);
	job->wake_up = 0;
	job->curr_hold_tcb = 0;
	job->prof_filp = NULL;
	job->prof_head = NULL;

	return 0;
}

static void destroy_aipu_job(struct aipu_job_manager *manager, struct aipu_job *job)
{
	WARN_ON(!job);

	kmem_cache_free(manager->job_cache, job);
}

static struct aipu_job *create_aipu_job(struct aipu_job_manager *manager,
					struct aipu_job_desc *desc,
					struct aipu_thread_wait_queue *queue, struct file *filp)
{
	int ret = 0;
	struct aipu_job *new_aipu_job = NULL;

	new_aipu_job = kmem_cache_zalloc(manager->job_cache, GFP_KERNEL);
	if (unlikely(!new_aipu_job))
		return ERR_PTR(-ENOMEM);

	ret = init_aipu_job(manager, new_aipu_job, desc, queue, filp);
	if (unlikely(ret)) {
		destroy_aipu_job(manager, new_aipu_job);
		new_aipu_job = NULL;
		return ERR_PTR(ret);
	}

	return new_aipu_job;
}

static void remove_aipu_job(struct aipu_job_manager *manager, struct aipu_job *job)
{
	WARN_ON(!job);
	list_del(&job->node);
	destroy_aipu_job(manager, job);
}

static void delete_job_queue(struct aipu_job_manager *manager, struct aipu_job **head)
{
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;

	if (head && *head) {
		list_for_each_entry_safe(curr, next, &(*head)->node, node) {
			remove_aipu_job(manager, curr);
		}
		kmem_cache_free(manager->job_cache, *head);
		*head = NULL;
	}
}

inline bool is_job_version_match(struct aipu_partition *core, struct aipu_job_desc *user_job)
{
	if (core->arch == user_job->aipu_arch && user_job->version_compatible)
		return true;

	return (core->arch == user_job->aipu_arch) &&
		(core->version == user_job->aipu_version) &&
		(core->config == user_job->aipu_config);
}

inline bool is_job_ok_for_core(struct aipu_partition *core, struct aipu_job_desc *user_job)
{
	if (!is_job_version_match(core, user_job)) {
		dev_err(core->dev, "invalid specified arch %d or version %d or configuration %d\n",
			core->arch, core->version, core->config);
		return false;
	}

	if (user_job->dtcm_size_kb > (core->dtcm_size >> 10)) {
		dev_err(core->dev, "the requested dtcm size (%dKB) > total dtcm size (%dKB)\n",
			user_job->dtcm_size_kb, core->dtcm_size >> 10);
		return false;
	}

	return true;
}

static bool is_user_job_valid(struct aipu_job_manager *manager, struct aipu_job_desc *user_job)
{
	int partition_id = user_job->partition_id;
	int core_id = user_job->core_id;
	int core_cnt = 0;
	struct aipu_partition *partition = NULL;
	struct aipu_partition *core = NULL;

	if (user_job->aipu_version >= AIPU_ISA_VERSION_ZHOUYI_V3_2) {
		if (manager->version != user_job->aipu_version ||
		    partition_id > 1) {
			dev_err(manager->dev, "invalid version number (%d) or partition ID (%d)",
				user_job->aipu_version, partition_id);
			return false;
		}

		return true;
	}

	/* for v3: partition->clusters->cores arch */
	if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (manager->version != AIPU_ISA_VERSION_ZHOUYI_V3 ||
		    partition_id >= manager->partition_cnt) {
			dev_err(manager->dev, "invalid version number (%d) or partition ID (%d)",
				user_job->aipu_version, partition_id);
			return false;
		}

		/*
		 * enable_prof is documented v1/v2-only, and the
		 * V3 completion path would call sky1's all-NULL soc_ops
		 * vtable through it, in hard-IRQ context. Refuse the
		 * input the hardware cannot service, like the version and
		 * partition checks above.
		 */
		if (user_job->enable_prof) {
			dev_err(manager->dev,
				"invalid job: enable_prof is not supported on v3");
			return false;
		}

		if (user_job->exec_flag & AIPU_JOB_EXEC_FLAG_DBG_DISPATCH) {
			partition = &manager->partitions[partition_id];
			core_cnt = atomic_read(&partition->clusters[0].en_core_cnt);
			if (core_id >= core_cnt) {
				dev_err(partition->dev, "invalid core ID (%d) >= enabled core count (%d)",
					core_id, core_cnt);
				return false;
			}
		}

		return true;
	}

	/* for non-v3 archs, a partition just means an AIPU core */
	core_cnt = manager->partition_cnt;

	/* debugger deferred execution: reserve the specified core ID */
	if (user_job->is_defer_run) {
		if (core_id >= core_cnt) {
			dev_err(manager->dev, "invalid core ID (%d) >= core count (%d)",
				core_id, core_cnt);
			return false;
		}

		core = &manager->partitions[core_id];
		return is_job_ok_for_core(core, user_job);
	}

	/* for non-v3 normal execution: find an available core ID */
	for (core_id = 0; core_id < core_cnt; core_id++) {
		core = &manager->partitions[core_id];
		if (is_job_ok_for_core(core, user_job))
			return true;
	}

	return false;
}

static int get_available_core_no_lock(struct aipu_job_manager *manager, struct aipu_job *job)
{
	int id = 0;
	struct aipu_partition *core = NULL;

	if (unlikely(!manager))
		return -1;

	for (id = 0; id < manager->partition_cnt; id++) {
		core = &manager->partitions[id];
		if (!atomic_read(&core->disable) && manager->idle_bmap[id] &&
		    is_job_ok_for_core(core, &job->desc))
			return id;
	}

	return -1;
}

static void reserve_core_for_job_no_lock(struct aipu_job_manager *manager, struct aipu_job *job,
					 int do_trigger)
{
	struct aipu_partition *sched_core = NULL;
	int ret = 0;

	WARN_ON(job->core_id < 0);
	WARN_ON(job->core_id >= manager->partition_cnt);

	sched_core = &manager->partitions[job->core_id];
	manager->idle_bmap[job->core_id] = 0;
	/*
	 * stamped unconditionally: the Job Deadline needs a
	 * dispatch time on every job, not only profiled ones.
	 */
	job->sched_time = ktime_get();

	ret = sched_core->ops->reserve(sched_core, &job->desc, do_trigger, 0);
	if (do_trigger && !ret)
		job->state = AIPU_JOB_STATE_RUNNING;

	if (do_trigger)
		dev_dbg(sched_core->dev, "[Job %lld of Thread %d] trigger job running done\n",
			job->desc.job_id, job->uthread_id);
	else
		dev_dbg(sched_core->dev, "[Job %lld of Thread %d] reserve for deferred job done\n",
			job->desc.job_id, job->uthread_id);
}

static void check_enable_tec_interrupts(struct aipu_job_manager *manager, struct aipu_job *job)
{
	struct aipu_tcb *tcb = NULL;
	struct aipu_hold_tcb_buf *htbuf = NULL;

	if (!manager || !job || manager->tec_intr_en)
		return;

	htbuf = aipu_mm_get_hold_htbuf(manager->mm, job->curr_hold_tcb);
	if (!htbuf) {
		dev_err(manager->dev, "get hold tbuf failed: 0x%llx\n", job->curr_hold_tcb);
		return;
	}

	/*
	 * A hold buffer that never completed its alloc (or is racing a
	 * teardown) carries NULL prev_tbuf / tail_tcb: the same fields
	 * aipu_mm_unlink_tcb() was hardened against after a shipped Oops.
	 */
	if (!htbuf->prev_tbuf || !htbuf->prev_tbuf->tail_tcb) {
		dev_err(manager->dev, "hold tbuf 0x%llx has no prev tail tcb\n",
			job->curr_hold_tcb);
		return;
	}

	tcb = htbuf->prev_tbuf->tail_tcb;
	if (!tcb->task.pprint)
		return;

	manager->partitions[0].ops->enable_interrupt(&manager->partitions[0], true);
	manager->tec_intr_en = true;
}

static int schedule_v3_job_no_lock(struct aipu_job_manager *manager, struct aipu_job *job)
{
	int ret = 0;
	int trigger_type = 0;
	int partition_id = job->desc.partition_id;
	struct aipu_partition *partition = &manager->partitions[partition_id];
	struct command_pool *pool = &manager->pools[partition_id];
	struct qos *qlist = NULL;
	struct aipu_hold_tcb_buf *htbuf = NULL;

	if (!pool)
		return -EINVAL;

	htbuf = aipu_mm_get_hold_htbuf(manager->mm, job->curr_hold_tcb);
	if (!htbuf) {
		dev_err(partition->dev, "get hold tcb buffer failed.");
		return -EINVAL;
	}

	if (job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_QOS_SLOW)
		qlist = &pool->qlist[AIPU_JOB_QOS_SLOW];
	else
		qlist = &pool->qlist[AIPU_JOB_QOS_FAST];

	if (!pool->created)
		trigger_type = ZHOUYI_TRIGGER_TYPE_CREATE;
	else if (pool->aborted || !qlist->pool_head)
		trigger_type = ZHOUYI_TRIGGER_TYPE_UPDATE_DISPATCH;
	else
		trigger_type = ZHOUYI_TRIGGER_TYPE_DISPATCH;

	check_enable_tec_interrupts(manager, job);

	/**
	 * 1. debug-dispatch tasks cannot be linked to any existing command pool.
	 * 2. command pool containing debug-dispatch tasks cannot be linked by any task TCB;
	 * 3. we must create a new command pool to enable debug-dispatch.
	 */
	if (pool->debug) {
		dev_err(partition->dev, "debug-dispatch command pool cannot be linked");
		return -EINVAL;
	}

	if (job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_DBG_DISPATCH)
		trigger_type = ZHOUYI_TRIGGER_TYPE_DEBUG_DISPATCH;

	/* remove init tcb of single group jobs to make them run in parallel on multi-core */
	if (atomic_read(&partition->clusters[0].en_core_cnt) != 1 &&
	    pool->last_exec_flag & AIPU_JOB_EXEC_FLAG_SINGLE_GROUP &&
	    job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SINGLE_GROUP  &&
	    !(pool->last_exec_flag & AIPU_JOB_EXEC_FLAG_SEG_MMU)   &&
	    !(job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SEG_MMU))
		job->desc.head_tcb_pa = job->desc.first_task_tcb_pa;

	if (!qlist->pool_head) {
		qlist->pool_head = job->desc.head_tcb_pa;
		htbuf->prev_hold_tcb = 0;
	} else {
		ret = aipu_mm_link_tcb(manager->mm, qlist->curr_tail,
				       job->desc.head_tcb_pa, job->desc.job_id);
		if (ret) {
			dev_err(partition->dev, "link TCB 0x%llx to 0x%llx failed (step 2)",
				job->desc.head_tcb_pa, qlist->curr_tail);
			return ret;
		}
		htbuf->prev_hold_tcb = qlist->curr_tail;
	}

	qlist->curr_head = job->desc.head_tcb_pa;
	qlist->curr_tail = htbuf->head;
	qlist->tail_tcb  = htbuf->hold_tcb;
	aipu_mm_set_final_htbuf_index(manager->mm, htbuf->index);
	pool->last_exec_flag = job->desc.exec_flag;
	dev_dbg(partition->dev, "final htcb index %d status %d.\n", htbuf->index, htbuf->status);

	ret = partition->ops->reserve(partition, &job->desc, trigger_type, 0);
	if (!ret) {
		if (trigger_type == ZHOUYI_TRIGGER_TYPE_CREATE ||
		    trigger_type == ZHOUYI_TRIGGER_TYPE_DEBUG_DISPATCH)
			pool->created = true;

		if (trigger_type == ZHOUYI_TRIGGER_TYPE_DEBUG_DISPATCH)
			pool->debug = true;

		if (pool->aborted)
			pool->aborted = false;
	}

	return ret;
}

/*
 * the Job Deadline: the driver-side give-up threshold
 * for a submitted job. A job that has neither completed nor faulted within it
 * is expired -- reported to userspace as an exception and retired exactly as
 * the completion path retires a finished one -- never recovered (no driver action
 * was found that clears a true wedge, so an expiry
 * action that "recovered" would act on an assumed mechanism). 30 s is 20x the
 * longest measured inference and 3x the stock UMD's own ~10 s give-up, so a
 * healthy system never sees it; 0 disables and restores stock semantics
 * exactly (no sweep is ever armed).
 */
static uint job_deadline_s = 30;
module_param(job_deadline_s, uint, 0644);
MODULE_PARM_DESC(job_deadline_s,
		 "seconds a job may stay running/deferred before it is expired and reported as an exception (default 30, 0 disables)");

#define AIPU_JOB_DEADLINE_SWEEP_MS 5000

/*
 * Periodic self-rescheduling sweep, armed at dispatch, lapsing when no job is
 * left running/deferred. No per-job timer: coarse expiry (deadline + period)
 * is meaningless slack against a give-up threshold, and the measured hot
 * paths gain nothing but the dispatch stamp.
 *
 * DEFERRED is expired too: a job parked behind a wedged pool is equally a
 * zombie, and the suspend guard counts both.
 */
static void aipu_job_manager_deadline_sweep(struct work_struct *work)
{
	struct aipu_priv *aipu = container_of(work, struct aipu_priv, job_deadline_work.work);
	struct aipu_job_manager *manager = &aipu->job_manager;
	struct aipu_job *curr = NULL;
	u32 deadline_s = READ_ONCE(job_deadline_s);
	ktime_t now = ktime_get();
	bool outstanding = false;
	bool expired = false;
	unsigned long flags;
	int id = 0;

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->state != AIPU_JOB_STATE_RUNNING &&
		    curr->state != AIPU_JOB_STATE_DEFERRED)
			continue;

		if (deadline_s && ktime_to_ns(curr->sched_time) &&
		    ktime_ms_delta(now, curr->sched_time) >= (s64)deadline_s * MSEC_PER_SEC) {
			/*
			 * This warning is the driver's entire wedge-detection
			 * contribution: work was accepted and no
			 * completion interrupt retired it within the deadline.
			 */
			dev_warn(manager->dev,
				 "job 0x%llx (thread %d, %s) got no completion interrupt within the %u s job deadline; reporting it as an exception (possible NPU wedge)\n",
				 curr->desc.job_id, curr->uthread_id,
				 curr->state == AIPU_JOB_STATE_RUNNING ? "running" : "deferred",
				 deadline_s);
			curr->state = AIPU_JOB_STATE_EXCEP;
			expired = true;
			continue;
		}

		outstanding = true;
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	/* retire like the completion path: EXCEP -> bottom half -> wake-up */
	if (expired && manager->partitions) {
		for (id = 0; id < manager->partition_cnt; id++)
			aipu_job_manager_irq_bottom_half(&manager->partitions[id]);
	}

	if (outstanding && deadline_s)
		schedule_delayed_work(&aipu->job_deadline_work,
				      msecs_to_jiffies(AIPU_JOB_DEADLINE_SWEEP_MS));
}

/* a no-op while the sweep is already pending (schedule_delayed_work) */
static void aipu_job_manager_arm_deadline(struct aipu_job_manager *manager)
{
	struct aipu_priv *aipu = (struct aipu_priv *)manager->priv;

	if (READ_ONCE(job_deadline_s))
		schedule_delayed_work(&aipu->job_deadline_work,
				      msecs_to_jiffies(AIPU_JOB_DEADLINE_SWEEP_MS));
}

static int schedule_new_job(struct aipu_job_manager *manager, struct aipu_job_desc *user_job,
			    struct file *filp, int do_trigger)
{
	int ret = 0;
	struct aipu_job *kern_job = NULL;
	struct aipu_thread_wait_queue *queue = NULL;
	unsigned long flags;

	mutex_lock(&manager->wq_lock);
	if (user_job->enable_poll_opt)
		queue = create_thread_wait_queue(manager->wait_queue_head, 0, filp);
	else
		queue = create_thread_wait_queue(manager->wait_queue_head,
						 task_pid_nr(current), NULL);

	/*
	 * an ERR_PTR queue passes init_aipu_job's `if (queue)`
	 * and gets written through (queue->ref_cnt++) — a wild write the old
	 * WARN_ON here only narrated.
	 */
	if (IS_ERR(queue)) {
		mutex_unlock(&manager->wq_lock);
		return PTR_ERR(queue);
	}

	kern_job = create_aipu_job(manager, user_job, queue, filp);
	if (IS_ERR(kern_job)) {
		mutex_unlock(&manager->wq_lock);
		return PTR_ERR(kern_job);
	}
	mutex_unlock(&manager->wq_lock);

	if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		ret = aipu_mm_hold_tcb_buf_alloc(manager->mm, kern_job);
		if (ret != 0) {
			dev_err(manager->dev, "malloc placeholder tcb failed.\n");
			/*
			 * The job is not yet on scheduled_head: nothing else
			 * can find it, so drop its wait-queue reference and
			 * free it here or it leaks (and pins the queue) forever.
			 */
			mutex_lock(&manager->wq_lock);
			if (kern_job->thread_queue)
				delete_wait_node(&manager->wait_queue_head,
						 kern_job->thread_queue);
			destroy_aipu_job(manager, kern_job);
			mutex_unlock(&manager->wq_lock);
			return -ENOMEM;
		}
	}

	spin_lock_irqsave(&manager->lock, flags);
	/*
	 * every branch below leaves the job RUNNING, DEFERRED or
	 * PENDING, so it needs the dispatch stamp the Job Deadline measures
	 * from; the pre-v3 reserve path re-stamps at its own trigger.
	 */
	kern_job->sched_time = ktime_get();
	if (do_trigger) {
		kern_job->state = AIPU_JOB_STATE_PENDING;

		if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
			list_add_tail(&kern_job->node, &manager->scheduled_head->node);
			ret = schedule_v3_job_no_lock(manager, kern_job);
			if (!ret)
				kern_job->state = AIPU_JOB_STATE_RUNNING;
			else
				kern_job->state = AIPU_JOB_STATE_DEFERRED;
		} else {
			/*
			 * For a job using SRAM managed by AIPU Gbuilder, it should be
			 * executed exclusively in serial with other Gbuilder-managed-SRAM ones,
			 * and parallel scheduling is not allowed.
			 *
			 * Pending it if there has been a Gbuilder-managed-SRAM job running;
			 * otherwise mark the flag and reserve a core for running.
			 */
			list_add_tail(&kern_job->node, &manager->scheduled_head->node);
			if (kern_job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX) {
				if (manager->exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX)
					goto unlock;
				else
					manager->exec_flag |= AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;
			}
			kern_job->core_id = get_available_core_no_lock(manager, kern_job);
			if (kern_job->core_id >= 0)
				reserve_core_for_job_no_lock(manager, kern_job, do_trigger);
		}
	} else {
		if (user_job->aipu_version < AIPU_ISA_VERSION_ZHOUYI_V3 &&
		    (user_job->core_id >= manager->partition_cnt ||
		     !manager->idle_bmap[user_job->core_id])) {
			dev_err(manager->dev, "schedule new job (0x%llx) failed: invalid core ID %u",
				kern_job->desc.job_id, user_job->core_id);
			ret = -EINVAL;
			goto unlock;
		}

		kern_job->state = AIPU_JOB_STATE_DEFERRED;
		kern_job->core_id = user_job->core_id;
		list_add_tail(&kern_job->node, &manager->scheduled_head->node);

		if (user_job->aipu_version < AIPU_ISA_VERSION_ZHOUYI_V3)
			reserve_core_for_job_no_lock(manager, kern_job, do_trigger);
	}
unlock:
	spin_unlock_irqrestore(&manager->lock, flags);

	/* a job just went outstanding: keep watch */
	aipu_job_manager_arm_deadline(manager);

	return ret;
}

static int trigger_deferred_job_run(struct aipu_job_manager *manager,
				    struct aipu_job_desc *user_job)
{
	unsigned long flags;
	struct aipu_job *curr = NULL;
	struct aipu_partition *sched_core = NULL;
	int triggered = 0;

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->uthread_id == task_pid_nr(current) &&
		    curr->desc.job_id == user_job->job_id &&
		    curr->state == AIPU_JOB_STATE_DEFERRED) {
			curr->state = AIPU_JOB_STATE_RUNNING;
			/* the deadline runs from the real trigger */
			curr->sched_time = ktime_get();
			if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
				/**
				 * for debugger: it should ensure that the NPUs are free to accept
				 * new jobs before the deferred job is scheduled.
				 */
				schedule_v3_job_no_lock(manager, curr);
			} else {
				sched_core = &manager->partitions[curr->core_id];
				sched_core->ops->trigger(sched_core);
			}
			triggered = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	if (!triggered) {
		dev_err(manager->dev, "trigger deferred job (0x%llx) failed",
			user_job->job_id);
		return -EINVAL;
	}

	/*
	 * normally armed since the deferred schedule; re-arm here
	 * so a deadline enabled between defer and trigger still takes.
	 */
	aipu_job_manager_arm_deadline(manager);

	return 0;
}

/**
 * @init_aipu_job_manager() - initialize an existing job manager struct during driver probe phase
 * @manager: pointer to the struct job_manager struct to be initialized
 * @mm: pointer to the struct aipu_memory_manager
 * @priv: pointer to the struct aipu_priv
 *
 * Return: 0 on success and error code otherwise.
 */
int init_aipu_job_manager(struct aipu_job_manager *manager, struct aipu_memory_manager *mm,
			  void *priv)
{
	int ret = 0;
	struct aipu_cap cap;

	if (!manager || !mm || !priv)
		return -EINVAL;

	manager->is_init = 0;
	manager->version = ((struct aipu_priv *)priv)->version;
	manager->dev = ((struct aipu_priv *)priv)->dev;
	manager->partition_cnt = 0;
	manager->partitions = NULL;
	manager->pools = NULL;
	manager->idle_bmap = NULL;
	manager->job_cache =
		kmem_cache_create("aipu_job_cache", sizeof(struct aipu_job), 0, SLAB_PANIC, NULL);
	manager->prof_cache = NULL;
	mutex_init(&manager->wq_lock);
	/*
	 * both list heads used to be created unchecked — the
	 * INIT_LIST_HEAD below dereferenced a possible ERR_PTR before the
	 * old WARN_ONs even ran, and the function returned 0 regardless, so
	 * probe marked the driver is_init around error-pointer list heads.
	 */
	manager->scheduled_head = create_aipu_job(manager, NULL, NULL, NULL);
	if (IS_ERR(manager->scheduled_head)) {
		ret = PTR_ERR(manager->scheduled_head);
		manager->scheduled_head = NULL;
		goto err_destroy_cache;
	}
	INIT_LIST_HEAD(&manager->scheduled_head->node);
	spin_lock_init(&manager->lock);
	manager->wait_queue_head = create_thread_wait_queue(NULL, 0, NULL);
	if (IS_ERR(manager->wait_queue_head)) {
		ret = PTR_ERR(manager->wait_queue_head);
		manager->wait_queue_head = NULL;
		goto err_destroy_scheduled_head;
	}
	mutex_init(&manager->id_lock);
	manager->exec_flag = 0;
	manager->mm = mm;
	manager->priv = priv;
	/* the deferred printf ring lives in aipu_priv */
	spin_lock_init(&((struct aipu_priv *)priv)->printf_lock);
	((struct aipu_priv *)priv)->printf_head = 0;
	((struct aipu_priv *)priv)->printf_cnt = 0;
	((struct aipu_priv *)priv)->printf_dropped = 0;
	/* the Job Deadline sweep also lives in aipu_priv */
	INIT_DELAYED_WORK(&((struct aipu_priv *)priv)->job_deadline_work,
			  aipu_job_manager_deadline_sweep);
	aipu_mm_get_asid(mm, &cap);
	manager->asid0_base = aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0);
	atomic_set(&manager->tick_counter, 0);
	atomic_set(&manager->is_suspend, 0);

	memset(&manager->exit_tcb_desc, 0, sizeof(manager->exit_tcb_desc));

	manager->exec_id = AIPU_EXEC_ID;

	manager->is_init = 1;
	return ret;

err_destroy_scheduled_head:
	destroy_aipu_job(manager, manager->scheduled_head);
	manager->scheduled_head = NULL;
err_destroy_cache:
	kmem_cache_destroy(manager->job_cache);
	manager->job_cache = NULL;
	mutex_destroy(&manager->wq_lock);
	return ret;
}

/**
 * @deinit_aipu_job_manager() - de-init the job manager
 * @manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 */
void deinit_aipu_job_manager(struct aipu_job_manager *manager)
{
	if (!manager || !manager->is_init)
		return;

	/*
	 * the sweep walks scheduled_head and partitions: stop it
	 * (and any self-reschedule) before they are torn down below.
	 */
	cancel_delayed_work_sync(&((struct aipu_priv *)manager->priv)->job_deadline_work);
	kfree(manager->idle_bmap);
	manager->idle_bmap = NULL;
	delete_job_queue(manager, &manager->scheduled_head);
	delete_wait_queue(&manager->wait_queue_head);
	mutex_destroy(&manager->wq_lock);
	kmem_cache_destroy(manager->job_cache);
	manager->job_cache = NULL;
	/*
	 * both are devm memory released with the unbind; a stale
	 * manager must fault loudly on NULL rather than silently walk them.
	 */
	manager->partitions = NULL;
	manager->pools = NULL;
	manager->is_init = 0;
}

/**
 * @aipu_job_manager_set_partitions_info() - set multicore info. while probing
 * @manager:  pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @partition_cnt: AIPU core count
 * @partitions:    pointer to AIPU core struct array
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_set_partitions_info(struct aipu_job_manager *manager, int partition_cnt,
					 struct aipu_partition *partitions)
{
	if (!manager || !partition_cnt || !partitions)
		return -EINVAL;

	manager->partition_cnt = partition_cnt;
	manager->partitions = partitions;
	if (manager->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		manager->pools = devm_kzalloc(partitions[0].dev,
					      partition_cnt * sizeof(*manager->pools),
					      GFP_KERNEL);
		if (!manager->pools)
			goto err;
	}
	manager->idle_bmap = kmalloc_array(partition_cnt, sizeof(bool), GFP_KERNEL);
	if (!manager->idle_bmap)
		goto err;
	memset(manager->idle_bmap, 1, partition_cnt);
	return 0;

err:
	/*
	 * leave no half-set state behind: the caller unwinds the
	 * partitions array on failure, so a stale pointer here would be
	 * walked again by the next re-bind. pools is devm memory; the
	 * unbind frees it.
	 */
	manager->partition_cnt = 0;
	manager->partitions = NULL;
	manager->pools = NULL;
	return -ENOMEM;
}

/**
 * @aipu_job_manager_scheduler() - schedule a job flushed from userland
 * @manager:  pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @user_job: pointer to the userspace job descriptor
 * @filp:     pointer to the device char file
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_scheduler(struct aipu_job_manager *manager, struct aipu_job_desc *user_job,
			       struct file *filp)
{
	int ret = 0;

	if (unlikely(!manager || !user_job || !filp))
		return -EINVAL;

	if (unlikely(!is_user_job_valid(manager, user_job))) {
		dev_err(manager->dev, "[scheduler] invalid user job (0x%llx)", user_job->job_id);
		return -EINVAL;
	}

	if (atomic_read(&manager->is_suspend)) {
		dev_err(manager->dev, "[scheduler] the NPU hw is not available now");
		return -ENODEV;
	}

	/*
	 * asid0_base is a [kmd] field: the completion path keys job matching on
	 * (last_task_tcb_pa - asid0_base), so a caller-supplied asid0_base lets a
	 * job match another process's completion IRQ and mark its running jobs
	 * SUCCESS. Overwrite it with the manager's trusted base before the desc
	 * is copied into the kernel job -- is_v3x_job_done_or_excep then matches
	 * on a value the caller cannot forge.
	 */
	user_job->asid0_base = manager->asid0_base;

	/*
	 * The TCB addresses below are written straight into the TSM dispatch
	 * registers (head/first_task) and drive completion matching
	 * (last_task). Reject any non-zero address that does not resolve to a
	 * buffer this fd owns, before it reaches the hardware -- this runs in
	 * process context so the owning-buffer lookup (which takes buffer_mutex)
	 * is safe here, unlike at the dispatch site under manager->lock.
	 */
	if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		u64 tcb_pa[] = {
			user_job->head_tcb_pa,
			user_job->first_task_tcb_pa,
			user_job->last_task_tcb_pa,
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(tcb_pa); i++) {
			if (tcb_pa[i] &&
			    !aipu_mm_v3_pa_owned_by(manager->mm, tcb_pa[i], filp)) {
				dev_err(manager->dev,
					"[scheduler] job 0x%llx TCB address 0x%llx not owned by caller",
					user_job->job_id, tcb_pa[i]);
				return -EINVAL;
			}
		}
	}

	if (!user_job->is_defer_run)
		ret = schedule_new_job(manager, user_job, filp, 1);
	else if (!user_job->do_trigger)
		ret = schedule_new_job(manager, user_job, filp, 0);
	else
		ret = trigger_deferred_job_run(manager, user_job);

	if (ret)
		dev_err(manager->dev,
			"[scheduler] schedule job (0x%llx) failed: is_defer_run %d, do_trigger %d",
			user_job->job_id, user_job->is_defer_run, user_job->do_trigger);
	return ret;
}

/*
 * runs in the bottom half only: the IOVA -> VA walks below take
 * mm's buffer_mutex/phy_mutex, which used to sleep (and could deadlock)
 * inside the hard-IRQ handler.
 */
static void aipu_job_manager_real_time_printk(struct aipu_job_manager *manager,
					      struct aipu_partition *partition,
					      struct aipu_printf_sig *sig)
{
	struct aipu_tcb *tcb = NULL;
	char *buf = NULL;

	if (GET_PRINF_SIZE(sig->sig_flag)) {
		/* here: tail TCBP is the exact TCB sending a printf signal */
		tcb = aipu_mm_get_tcb(manager->mm, manager->asid0_base + sig->tail_tcbp);
		if (!tcb) {
			dev_err(partition->dev, "real time printk: no TCB found (0x%x)\n",
				sig->tail_tcbp);
			return;
		}

		buf = aipu_mm_get_va(manager->mm, manager->asid0_base + tcb->task.pprint);
		if (buf)
			dev_info(partition->dev, "[real-time printk 0x%x] %s",
				 tcb->task.pprint, buf);
		else
			dev_err(partition->dev, "real time printk: no pbuf found (0x%x)\n",
				tcb->task.pprint);
	}
}

/* hard-IRQ side: capture the two registers the bottom half needs, no locks
 * beyond the ring's own spinlock, no allocation, no mm walk
 */
static void aipu_job_manager_defer_real_time_printk(struct aipu_job_manager *manager,
						    struct job_irq_info *info)
{
	struct aipu_priv *aipu = (struct aipu_priv *)manager->priv;
	u32 tail = 0;

	spin_lock(&aipu->printf_lock);
	if (aipu->printf_cnt < AIPU_PRINTF_SIG_MAX) {
		tail = (aipu->printf_head + aipu->printf_cnt) % AIPU_PRINTF_SIG_MAX;
		aipu->printf_sigs[tail].tail_tcbp = info->tail_tcbp;
		aipu->printf_sigs[tail].sig_flag = info->sig_flag;
		aipu->printf_cnt++;
	} else {
		aipu->printf_dropped++;
	}
	spin_unlock(&aipu->printf_lock);
}

static void aipu_job_manager_flush_printf_sigs(struct aipu_job_manager *manager,
					       struct aipu_partition *partition)
{
	struct aipu_priv *aipu = (struct aipu_priv *)manager->priv;
	struct aipu_printf_sig sig;
	unsigned long flags;
	u32 dropped = 0;

	while (true) {
		spin_lock_irqsave(&aipu->printf_lock, flags);
		if (!aipu->printf_cnt) {
			dropped = aipu->printf_dropped;
			aipu->printf_dropped = 0;
			spin_unlock_irqrestore(&aipu->printf_lock, flags);
			break;
		}
		sig = aipu->printf_sigs[aipu->printf_head];
		aipu->printf_head = (aipu->printf_head + 1) % AIPU_PRINTF_SIG_MAX;
		aipu->printf_cnt--;
		spin_unlock_irqrestore(&aipu->printf_lock, flags);

		/* outside the ring lock: this path takes mm's mutexes */
		aipu_job_manager_real_time_printk(manager, partition, &sig);
	}

	if (dropped)
		dev_warn(partition->dev, "real time printk: %u signal(s) dropped\n", dropped);
}

static bool is_curr_irq_job(struct aipu_job *job, struct job_irq_info *info, u64 asid_base)
{
	return info->tail_tcbp >= (u32)((job->desc.first_task_tcb_pa - asid_base)) &&
		   info->tail_tcbp <= (u32)(job->desc.last_task_tcb_pa - asid_base);
}

static bool is_v3x_job_done_or_excep(struct aipu_job *job, struct job_irq_info *info,
				     int flag)
{
	return info->tail_tcbp == (u32)(job->desc.last_task_tcb_pa - job->desc.asid0_base) &&
	       (IS_DONE_IRQ(flag) || IS_EXCEPTION_IRQ(flag));
}

static bool do_abortion(int flag, struct job_irq_info *info)
{
	if (!info)
		return false;

	return IS_SERIOUS_ERR(flag) || IS_EXCEPTION_SIGNAL(info->sig_flag);
}

static bool is_job_end(struct aipu_job *job, struct aipu_partition *partition,
		       struct job_irq_info *info, int flag)
{
	if (job->state != AIPU_JOB_STATE_RUNNING)
		return false;

	if (job->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		return is_v3x_job_done_or_excep(job, info, flag) ||
			do_abortion(flag, info);
	}

	return job->core_id == partition->id;
}

static bool is_job_abnormal(struct aipu_job *job, int flag,
			    struct job_irq_info *info)
{
	if (job->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3)
		return IS_ABNORMAL(flag) || IS_EXCEPTION_SIGNAL(info->sig_flag);

	return flag != 0;
}

/*
 * retire completed jobs -- profiling stamp, hold-TCB unlink,
 * poller wake -- in the upper half, exactly as the bottom half would (its
 * two loops skip anything retired here via !wake_up). The wake used to
 * happen only in the bottom half, so every completion paid a hardirq ->
 * normal-priority single-thread workqueue hand-off before poll() could
 * return: 128-256 us on a 0.3 ms inference. The waiter-visible
 * state (state, wake_up, pdata) is final before the wake, and the unlink
 * must ride along with it: once woken, GET_JOB_STATUS can harvest and free
 * the job record, and nothing else unlinks curr_hold_tcb -- the hold pool
 * would shrink by one buffer every time the harvest beat the workqueue.
 * Runs under manager->lock and nests mm->shlock inside it, exactly as the
 * bottom half already does. The bottom half still owns every path that
 * marks jobs outside this handler (pool abort, coredump signal, deadline
 * expiry) and everything process-context (printf flush, pool destroy).
 */
static void retire_done_jobs_no_lock(struct aipu_job_manager *manager,
				     struct aipu_partition *partition)
{
	struct aipu_job *curr = NULL;

	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->state >= AIPU_JOB_STATE_EXCEP && !curr->wake_up &&
		    (curr->desc.aipu_version >= AIPU_ISA_VERSION_ZHOUYI_V3 ||
		     curr->core_id == partition->id)) {
			if (curr->desc.enable_prof)
				curr->pdata.execution_time_ns =
				(long)ktime_to_ns(ktime_sub(curr->done_time, curr->sched_time));

			if (curr->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
			    manager->mm && curr->curr_hold_tcb) {
				if (aipu_mm_unlink_tcb(manager->mm, curr->curr_hold_tcb))
					dev_warn_ratelimited(partition->dev,
						"unlink_tcb failed (hold 0x%llx); MM may be torn down mid-job\n",
						curr->curr_hold_tcb);
			}

			wake_up_interruptible(curr->thread_queue);
			curr->wake_up = 1;
		}
	}
}

/**
 * @aipu_job_manager_irq_upper_half() - aipu interrupt upper half handler
 * @core:           pointer to the aipu core struct
 * @exception_flag: exception flag
 * @info:           pointer to struct job interrupt info. (for v3 only)
 */
void aipu_job_manager_irq_upper_half(struct aipu_partition *partition, int flag,
				     struct job_irq_info *info)
{
	struct aipu_job *curr = NULL;
	struct aipu_job_manager *manager = NULL;
	int triggered = 0;
	bool abort_cmdpool = false;
	int ret= 0;

	if (unlikely(!partition))
		return;

	manager = get_job_manager(partition);

	if (manager->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (IS_SIGNAL_IRQ(flag) && !IS_EXCEPTION_SIGNAL(info->sig_flag)) {
			if (IS_PRINTF_SIGNAL(info->sig_flag))
				aipu_job_manager_defer_real_time_printk(manager, info);
			if (IS_COREDUMP_SIGNAL(info->sig_flag)) {
				spin_lock(&manager->lock);
				list_for_each_entry(curr, &manager->scheduled_head->node, node) {
					if (is_curr_irq_job(curr, info, manager->asid0_base)) {
						curr->state = AIPU_JOB_STATE_CORED;
						break;
					}
				}
				spin_unlock(&manager->lock);
			}
			return;
		}
	}

	spin_lock(&manager->lock);

	/* soft reset association irq not in coredump scope */
	if (manager->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (do_abortion(flag, info))
			abort_cmdpool = true;
	}

	if (abort_cmdpool) {
		list_for_each_entry(curr, &manager->scheduled_head->node, node) {
			/* coredump irq follows fault irq */
			if (curr->desc.is_coredump_en) {
				spin_unlock(&manager->lock);
				return;
			}
		}
		if (manager->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
			ret = partition->ops->abort_command_pool(partition, 0);
			if (!ret && manager->pools) {
				manager->pools[partition->id].aborted = true;
				list_for_each_entry(curr, &manager->scheduled_head->node, node)
					curr->state = AIPU_JOB_STATE_EXCEP;
				spin_unlock(&manager->lock);
				return;
			}
		}
	}

	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (is_job_end(curr, partition, info, flag)) {
			if (unlikely(is_job_abnormal(curr, flag, info)))
				curr->state = AIPU_JOB_STATE_EXCEP;
			else
				curr->state = AIPU_JOB_STATE_SUCCESS;

			if (curr->desc.enable_prof)
				curr->done_time = ktime_get();

			if (atomic_read(&manager->tick_counter) && info)
				curr->pdata.tick_counter = info->tick_counter;
			else
				curr->pdata.tick_counter = 0;

			if (curr->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX)
				manager->exec_flag &= ~AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;

			if (manager->pools && manager->pools[partition->id].debug)
				manager->dbg_do_destroy = true;

				/*
				 * For v3/v3_2: the hardware fires one done interrupt
				 * per command pool execution, reporting only the last
				 * completed TCB. When multiple jobs from different
				 * processes are chained in the same command pool,
				 * preceding RUNNING jobs whose TCBs finished before
				 * the matched job also need to be marked SUCCESS.
				 *
				 * Assumptions:
				 * - scheduled_head list order matches the hardware
				 *   hold-TCB chain order (both follow schedule order).
				 * - partition_id plus QoS class scope the chain: V3
				 *   keeps two independent hardware chains per pool
				 *   (qlist[SLOW]/qlist[FAST]) that progress and
				 *   complete separately, so a completion on one chain
				 *   must not retire preceding jobs on the other -- they
				 *   may still be executing.
				 * - Profiling data (done_time, tick_counter) is not
				 *   collected for these preceding jobs. Jobs that
				 *   require enable_prof should run without competing
				 *   jobs on the same command pool.
				 */
			if (curr->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3 ||
			    curr->desc.aipu_version >= AIPU_ISA_VERSION_ZHOUYI_V3_2) {
				struct aipu_job *prev = NULL;
				list_for_each_entry(prev, &manager->scheduled_head->node, node) {
					if (prev == curr)
						break;
					if (prev->state == AIPU_JOB_STATE_RUNNING &&
					    prev->desc.partition_id == curr->desc.partition_id &&
					    (prev->desc.exec_flag & AIPU_JOB_EXEC_FLAG_QOS_SLOW) ==
					    (curr->desc.exec_flag & AIPU_JOB_EXEC_FLAG_QOS_SLOW)) {
						prev->state = AIPU_JOB_STATE_SUCCESS;
						if (prev->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX)
							manager->exec_flag &= ~AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;
					}
				}
			}

			break;
		}
	}

	if (!atomic_read(&partition->disable)) {
		list_for_each_entry(curr, &manager->scheduled_head->node, node) {
			if (curr->state == AIPU_JOB_STATE_PENDING &&
			    is_job_ok_for_core(partition, &curr->desc)) {
				if (curr->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX) {
					if (manager->exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX)
						continue;
					else
						manager->exec_flag |= AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;
				}
				curr->core_id = partition->id;
				reserve_core_for_job_no_lock(manager, curr, 1);
				triggered = 1;
				break;
			}
		}
	}

	if (!triggered)
		manager->idle_bmap[partition->id] = 1;

	retire_done_jobs_no_lock(manager, partition);

	spin_unlock(&manager->lock);
}

static void aipu_job_manager_destroy_command_pool_no_lock(struct aipu_job_manager *manager,
							  struct aipu_partition *partition)
{
	struct command_pool *pool;
	int ret = 0;

	if (!manager || !partition || !manager->pools)
		return;

	/*
	 * index the pool by the partition this was handed, like
	 * every other pools[] site -- pools[0] was correct only by accident
	 * on a single-partition part (the dead unlink parameter goes
	 * with it).
	 */
	pool = &manager->pools[partition->id];
	if (pool->created) {
		if (manager->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
			ret = partition->ops->destroy_command_pool(partition, 0);
			if (ret == 0) {
				memset(pool->qlist, 0, sizeof(*pool->qlist) * AIPU_JOB_QOS_MAX);
				pool->created = false;
				pool->debug = false;
				manager->tec_intr_en = false;
				aipu_mm_set_final_htbuf_index(manager->mm, -1);
			}
		}
	}
}

/**
 * @aipu_job_manager_irq_bottom_half() - aipu interrupt bottom half handler
 * @core: pointer to the aipu core struct
 */
void aipu_job_manager_irq_bottom_half(struct aipu_partition *core)
{
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;
	struct aipu_job_manager *manager = NULL;
	unsigned long flags;
	bool do_destroy = core->version >= AIPU_ISA_VERSION_ZHOUYI_V3;

	if (unlikely(!core))
		return;

	manager = get_job_manager(core);

	/* process context now: resolve and print the deferred printf signals */
	aipu_job_manager_flush_printf_sigs(manager, core);

	spin_lock_irqsave(&manager->lock, flags);

	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->state >= AIPU_JOB_STATE_EXCEP && !curr->wake_up &&
		    (curr->desc.aipu_version >= AIPU_ISA_VERSION_ZHOUYI_V3 ||
		     curr->core_id == core->id)) {
			if (curr->desc.enable_prof)
				curr->pdata.execution_time_ns =
				(long)ktime_to_ns(ktime_sub(curr->done_time, curr->sched_time));

			if (curr->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
			    manager->mm && curr->curr_hold_tcb) {
				if (aipu_mm_unlink_tcb(manager->mm, curr->curr_hold_tcb))
					dev_warn_ratelimited(core->dev,
						"unlink_tcb failed (hold 0x%llx); MM may be torn down mid-job\n",
						curr->curr_hold_tcb);
			}
		}

		/* destroy the v3 command pool if all jobs are done */
		if (curr->state < AIPU_JOB_STATE_EXCEP || curr->desc.is_coredump_en)
			do_destroy = false;
	}

	if (manager->pools &&
	    (!is_grid_end(manager->pools[core->id].qlist[AIPU_JOB_QOS_SLOW].tail_tcb) ||
	     !is_grid_end(manager->pools[core->id].qlist[AIPU_JOB_QOS_FAST].tail_tcb)))
		do_destroy = false;

	if (manager->dbg_do_destroy) {
		do_destroy = true;
		manager->dbg_do_destroy = false;
	}

	if (do_destroy)
		aipu_job_manager_destroy_command_pool_no_lock(manager, core);

	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->state >= AIPU_JOB_STATE_EXCEP && !curr->wake_up &&
		    (curr->desc.aipu_version >= AIPU_ISA_VERSION_ZHOUYI_V3 ||
		     curr->core_id == core->id)) {
			wake_up_interruptible(curr->thread_queue);
			curr->wake_up = 1;
#ifdef DEBUG
			/* debug */
			print_core_id(manager->mm, curr->desc.head_tcb_pa, curr->desc.tail_tcb_pa);
#endif
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);
}

int aipu_job_manager_abort_cmd_pool(struct aipu_job_manager *manager)
{
	int ret = 0;
	unsigned long flags;
	struct aipu_partition *partition = NULL;
	struct aipu_job *curr = NULL;

	if (!manager)
		return -EINVAL;

	/*
	 * pools is V3-only and was dereferenced here before the
	 * NULL check four lines below it ever ran. (pools != NULL also
	 * implies partitions was set up, so forming the pointer is safe.)
	 */
	if (!manager->pools)
		return -ENODEV;

	partition = &manager->partitions[0];
	if (!manager->pools[partition->id].created)
		return -ENODEV;

	spin_lock_irqsave(&manager->lock, flags);
	ret = partition->ops->abort_command_pool(partition, 0);
	if (!ret)
		manager->pools[partition->id].aborted = true;

	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->state == AIPU_JOB_STATE_SUCCESS)
			continue;
		curr->state = AIPU_JOB_STATE_EXCEP;
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	aipu_job_manager_irq_bottom_half(partition);

	return ret;
}

/**
 * @aipu_job_manager_cancel_jobs() - cancel all jobs flushed for certain user closing fd
 * @param manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @param filp: file struct pointer
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_cancel_jobs(struct aipu_job_manager *manager, struct file *filp)
{
	unsigned long flags;
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;
	struct aipu_partition *par = NULL;
	bool multi_process = false;
	bool abort_cmd_pool = false;
	int job_total = 0;
	int job_index = 0;
	struct aipu_job **delete_jobs = NULL;

	if (!manager || !filp)
		return -EINVAL;
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->filp == filp)
			job_total++;
	}
	spin_unlock_irqrestore(&manager->lock, flags);
	delete_jobs = kcalloc(job_total, sizeof(*delete_jobs), GFP_KERNEL);
	if (!delete_jobs)
		return -ENOMEM;

	/* jobs should be cleaned first */
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->filp == filp) {
			/*
			 * core_id stays -1 on every V3 job (only
			 * the pre-V3 branch assigns it), so form the
			 * partition pointer only where it is used.
			 */
			if (curr->desc.aipu_version < AIPU_ISA_VERSION_ZHOUYI_V3) {
				par = &manager->partitions[curr->core_id];
				if (curr->state == AIPU_JOB_STATE_DEFERRED ||
				    (curr->state == AIPU_JOB_STATE_RUNNING &&
				     par->ops->is_idle(par)))
					manager->idle_bmap[curr->core_id] = 1;
			}

			if ((manager->version >= AIPU_ISA_VERSION_ZHOUYI_V3) &&
				curr->state == AIPU_JOB_STATE_RUNNING)
				abort_cmd_pool = true;
			delete_jobs[job_index] = curr;
			job_index++;
			list_del(&curr->node);
		} else {
			multi_process = true;
		}
	}

	if (!multi_process &&
	    (manager->version >= AIPU_ISA_VERSION_ZHOUYI_V3)) {
		aipu_job_manager_destroy_command_pool_no_lock(manager,
							      &manager->partitions[0]);
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	//For ctlr+c app exit
	if (!multi_process && abort_cmd_pool)
		aipu_job_manager_abort_cmd_pool(manager);

	/*
	 * the multi_process guard: with foreign jobs scheduled
	 * the pool is neither destroyed nor aborted on this close. Correct --
	 * but it was silent, and this skip is the first breadcrumb of a
	 * cross-process wedge investigation, so say it happened.
	 */
	if (multi_process && abort_cmd_pool)
		dev_info(manager->dev,
			 "fd closed with jobs still running; command pool left intact (other processes have jobs scheduled)\n");

	mutex_lock(&manager->wq_lock);
	for (job_index = 0; job_index < job_total; job_index++) {
		delete_wait_node(&manager->wait_queue_head, delete_jobs[job_index]->thread_queue);
		kmem_cache_free(manager->job_cache, delete_jobs[job_index]);
	}
	delete_thread_wait_queue(manager->wait_queue_head, task_pid_nr(current), filp);
	mutex_unlock(&manager->wq_lock);

	kfree(delete_jobs);
	return 0;
}

/**
 * @aipu_job_manager_get_job_status() - get AIPU jobs' statuses after a polling event returns
 * @manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @job_status: job status array stores statuys info filled by this API
 * @filp: file struct pointer
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_get_job_status(struct aipu_job_manager *manager,
				    struct aipu_job_status_query *job_status, struct file *filp)
{
	int ret = 0;
	struct aipu_job_status_desc *status = NULL;
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;
	struct aipu_job **done_jobs = NULL;
	int poll_iter = 0;
	unsigned long flags;

	if (unlikely(!manager || !job_status || job_status->max_cnt < 1))
		return -EINVAL;

	status = kcalloc(job_status->max_cnt, sizeof(*status), GFP_KERNEL);
	if (!status)
		return -ENOMEM;

	done_jobs = kcalloc(job_status->max_cnt, sizeof(*done_jobs), GFP_KERNEL);
	if (!done_jobs) {
		kfree(status);
		return -ENOMEM;
	}

	job_status->poll_cnt = 0;
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (job_status->poll_cnt == job_status->max_cnt)
			break;

		if (curr->state < AIPU_JOB_STATE_EXCEP)
			continue;

		if (curr->filp != filp)
			continue;

		if ((job_status->of_this_thread && curr->uthread_id == task_pid_nr(current)) ||
		    !job_status->of_this_thread) {
			status[poll_iter].job_id = curr->desc.job_id;
			status[poll_iter].thread_id = curr->uthread_id;
			if (curr->state == AIPU_JOB_STATE_SUCCESS)
				status[poll_iter].state = AIPU_JOB_STATE_DONE;
			else if (curr->state == AIPU_JOB_STATE_CORED)
				status[poll_iter].state = AIPU_JOB_STATE_COREDUMP;
			else
				status[poll_iter].state = AIPU_JOB_STATE_EXCEPTION;

			if (curr->desc.enable_prof || curr->pdata.tick_counter)
				status[poll_iter].pdata = curr->pdata;

			done_jobs[poll_iter] = curr;
			list_del(&curr->node);
			job_status->poll_cnt++;
			poll_iter++;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	/*
	 * No cache maintenance on harvest. Every alias of
	 * NPU-written memory is uncached: user mappings are Normal-NC
	 * (aipu_mm_mmap_buf), the kernel vmap alias is pgprot_noncached,
	 * and the linear map is only ever CPU-written at allocation time,
	 * where alloc_phy_and_map clean+invalidates each block. Post-alloc
	 * the linear map can hold only clean speculative lines, whose
	 * eviction is architecturally silent -- so the whole-fd
	 * clean+invalidate walk that lived here (O(live REUSE+STACK bytes)
	 * under buffer_mutex, once per completion: 12% of a small-graph
	 * inference, 8.5 ms of a VAE decode) maintained lines no reader
	 * ever consumed.
	 */

	/*
	 * deliver the statuses before destroying the jobs: they
	 * were unlinked under the lock above, so a failed copy can relink
	 * them and report -EFAULT (not the raw residual, a positive
	 * "success") instead of freeing completion records the caller never
	 * received and can never re-poll.
	 */
	if (copy_to_user((struct job_status_desc __user *)job_status->status, status,
			 job_status->poll_cnt * sizeof(*status))) {
		spin_lock_irqsave(&manager->lock, flags);
		for (poll_iter = 0; poll_iter < job_status->poll_cnt; poll_iter++)
			list_add_tail(&done_jobs[poll_iter]->node,
				      &manager->scheduled_head->node);
		spin_unlock_irqrestore(&manager->lock, flags);
		ret = -EFAULT;
		goto out;
	}

	mutex_lock(&manager->wq_lock);
	for (poll_iter = 0; poll_iter < job_status->poll_cnt; poll_iter++) {
		delete_wait_node(&manager->wait_queue_head, done_jobs[poll_iter]->thread_queue);
		destroy_aipu_job(manager, done_jobs[poll_iter]);
	}
	mutex_unlock(&manager->wq_lock);

out:
	kfree(status);
	kfree(done_jobs);
	return ret;
}

/**
 * @aipu_job_manager_has_end_job() - check if a user thread has end job(s) to query
 * @manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @filp: file struct pointer
 * @wait: wait table struct from kernel
 * @uthread_id: thread ID
 *
 * Return: true if there is/are AIPU job(s) done and false otherwise.
 */
bool aipu_job_manager_has_end_job(struct aipu_job_manager *manager, struct file *filp,
				  struct poll_table_struct *wait, int uthread_id)
{
	bool ret = false;
	struct aipu_job *curr = NULL;
	struct aipu_thread_wait_queue *wq = NULL;
	unsigned long flags;

	if (unlikely(!manager || !filp))
		return -EINVAL;

	mutex_lock(&manager->wq_lock);
	list_for_each_entry(wq, &manager->wait_queue_head->node, node) {
		if (wq->ref_cnt &&
		    (wq->uthread_id == uthread_id || wq->filp == filp)) {
			poll_wait(filp, &wq->p_wait, wait);
			break;
		}
	}
	mutex_unlock(&manager->wq_lock);

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->filp == filp &&
		    curr->wake_up == 1 &&
		    (curr->desc.enable_poll_opt || curr->uthread_id == uthread_id)) {
			ret = true;
			break;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	return ret;
}

int aipu_job_manager_get_hw_status(struct aipu_job_manager *manager, struct aipu_hw_status *hw)
{
	unsigned long flags;
	struct aipu_job *curr = NULL;

	if (!manager || !hw)
		return -EINVAL;

	hw->status = AIPU_STATUS_IDLE;
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		/* exception jobs should be cleared and hw is reset */
		if (curr->state == AIPU_JOB_STATE_RUNNING ||
		    curr->state == AIPU_JOB_STATE_DEFERRED) {
			hw->status = AIPU_STATUS_BUSY;
			break;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	return 0;
}
int aipu_job_manager_get_running_job_thread(struct aipu_job_manager *manager,
						struct aipu_running_job_query *query)
{
	unsigned long flags;
	struct aipu_job *curr = NULL;
	unsigned int core_i = 0;
	unsigned int max_cnt = 0;

	if (!manager || !query)
		return -EINVAL;

	if (manager->version < AIPU_ISA_VERSION_ZHOUYI_V3)
		return 0;

	/*
	 * this read replaces an unsynchronised ~600-byte struct
	 * copy (two mutexes included, owner/waiter state and all); the loop
	 * bound is capped by the UAPI array so a core-count bump can never
	 * write past the caller's thread_id[] (the old bound also wrote
	 * core_cnt + 1 entries).
	 */
	max_cnt = min_t(unsigned int, manager->partitions[0].clusters[0].core_cnt,
			ARRAY_SIZE(query->thread_id));
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->state == AIPU_JOB_STATE_RUNNING) {
			if (core_i == max_cnt)
				break;
			query->thread_id[core_i] = curr->uthread_id;
			core_i++;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);
	return 0;
}

int aipu_job_manager_disable_tick_counter(struct aipu_job_manager *manager)
{
	if (!manager)
		return -EINVAL;

	if (manager->version < AIPU_ISA_VERSION_ZHOUYI_V3 || !atomic_read(&manager->tick_counter))
		return 0;

	if (atomic_dec_and_test(&manager->tick_counter))
		manager->partitions[0].ops->disable_tick_counter(&manager->partitions[0]);

	return 0;
}

int aipu_job_manager_enable_tick_counter(struct aipu_job_manager *manager)
{
	if (!manager)
		return -EINVAL;

	if (manager->version < AIPU_ISA_VERSION_ZHOUYI_V3)
		return 0;

	if (atomic_inc_return(&manager->tick_counter) == 1)
		manager->partitions[0].ops->enable_tick_counter(&manager->partitions[0]);

	return 0;
}

int aipu_job_manager_config_clusters(struct aipu_job_manager *manager,
				     struct aipu_config_clusters *cfg)
{
	int ret = 0;
	unsigned long flags;
	struct aipu_job *curr = NULL;
	struct aipu_partition *partition = NULL;
	int idx = 0;
	u32 en_count = 0;
	u32 core_cnt = 0;

	if (!manager || !cfg)
		return -EINVAL;

	if (manager->version < AIPU_ISA_VERSION_ZHOUYI_V3)
		return 0;

	partition = &manager->partitions[0];

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->state == AIPU_JOB_STATE_RUNNING ||
		    curr->state == AIPU_JOB_STATE_DEFERRED) {
			ret = -EBUSY;
			dev_err(manager->dev, "config clusters failed: aipu is busy now");
			goto unlock;
		}
	}

	for (idx = 0; idx < partition->cluster_cnt; idx++) {
		en_count = cfg->clusters[idx].en_core_cnt;
		core_cnt = partition->clusters[idx].core_cnt;

		if (en_count > core_cnt) {
			ret = -EINVAL;
			dev_err(manager->dev, "invalid enable core count: %u (> tot count %u)",
				en_count, core_cnt);
			goto unlock;
		}
		partition->ops->enable_core_cnt(partition, idx, en_count);
		aipu_job_manager_destroy_command_pool_no_lock(manager, partition);
	}

	atomic_set(&manager->is_suspend, !en_count);

unlock:
	spin_unlock_irqrestore(&manager->lock, flags);
	return ret;
}

int aipu_job_manager_suspend(struct aipu_job_manager *manager)
{
	struct aipu_config_clusters cfg;

	if (!manager)
		return -EINVAL;

	memset(&cfg, 0, sizeof(cfg));

	if (manager->version >= AIPU_ISA_VERSION_ZHOUYI_V3)
		return aipu_job_manager_config_clusters(manager, &cfg);

	atomic_set(&manager->is_suspend, 1);
	return 0;
}

int aipu_job_manager_resume(struct aipu_job_manager *manager)
{
	struct aipu_config_clusters cfg;

	if (!manager)
		return -EINVAL;

	/*
	 * config_clusters programs every cluster's en_core_cnt,
	 * so the ones after [0] must not come from stack garbage — suspend
	 * (above) already clears its cfg.
	 */
	memset(&cfg, 0, sizeof(cfg));

	cfg.clusters[0].en_core_cnt = manager->partitions[0].clusters[0].core_cnt;

	if (manager->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		int ret = aipu_job_manager_config_clusters(manager, &cfg);

		if (ret)
			return ret;

		/*
		 * CMD_POOL_INTR_CTRL does not survive the power
		 * domain dropping, and probe is the only unconditional
		 * write: without this restore, a client that never sets
		 * task.pprint dispatches with every interrupt source
		 * masked and its jobs complete unreported.
		 * Write the mask the software state says is armed.
		 */
		manager->partitions[0].ops->enable_interrupt(&manager->partitions[0],
							     manager->tec_intr_en);
		return 0;
	}

	atomic_set(&manager->is_suspend, 0);
	return 0;
}

int aipu_job_manager_alloc_grid_id(struct aipu_job_manager *manager)
{
	u16 id = 0;

	if (!manager)
		return -EINVAL;

	/*
	 * grid IDs have no free path by design (UAPI: "do not
	 * need to free"), so the u16 counter must saturate, not wrap into
	 * IDs that may still be live -- an unprivileged tight loop reaches
	 * the wrap in under a second. Widening the ID is UAPI-frozen.
	 */
	mutex_lock(&manager->id_lock);
	if (manager->grid_id == U16_MAX) {
		mutex_unlock(&manager->id_lock);
		dev_warn_once(manager->dev, "grid IDs exhausted (u16, no free path)\n");
		return -ENOSPC;
	}
	id = manager->grid_id++;
	mutex_unlock(&manager->id_lock);

	return id;
}

int aipu_job_manager_alloc_exec_id(struct aipu_job_manager *manager, u64 *exec_id)
{
	if (!manager || !exec_id)
		return -EINVAL;

	mutex_lock(&manager->id_lock);
	*exec_id = manager->exec_id++;
	mutex_unlock(&manager->id_lock);

	return 0;
}

int aipu_job_manager_alloc_group_id(struct aipu_job_manager *manager,
				    struct aipu_id_desc *desc)
{
	int ret = 0;
	unsigned long id = 0;

	if (!manager || !desc || !manager->group_id_bmap)
		return -EINVAL;

	if (!desc->size || desc->size >= manager->group_id_num)
		return -EINVAL;

	mutex_lock(&manager->id_lock);
	id = bitmap_find_next_zero_area(manager->group_id_bmap, manager->group_id_num,
					0, desc->size, 0);
	if ((id + desc->size) < manager->group_id_num) {
		bitmap_set(manager->group_id_bmap, id, desc->size);
		desc->first_id = id;
	} else {
		desc->first_id = 0;
		ret = -EINVAL;
	}
	mutex_unlock(&manager->id_lock);

	return ret;
}

int aipu_job_manager_free_group_id(struct aipu_job_manager *manager,
				   struct aipu_id_desc *desc)
{
	if (!manager || !desc || !manager->group_id_bmap)
		return -EINVAL;

	if (desc->first_id >= manager->group_id_num ||
	    (desc->first_id + desc->size) >= manager->group_id_num)
		return -EINVAL;

	mutex_lock(&manager->id_lock);
	bitmap_clear(manager->group_id_bmap, desc->first_id, desc->size);
	mutex_unlock(&manager->id_lock);

	return 0;
}

int aipu_job_manager_alloc_sflag_id(struct aipu_job_manager *manager,
				    struct aipu_id_desc *desc)
{
	int ret = 0;
	unsigned long id = 0;

	if (!manager || !desc || !manager->sflag_id_bmap)
		return -EINVAL;

	if (!desc->size || desc->size >= manager->sflag_id_num)
		return -EINVAL;

	mutex_lock(&manager->id_lock);
	id = bitmap_find_next_zero_area(manager->sflag_id_bmap, manager->sflag_id_num,
					0, desc->size, 0);

	if ((id + desc->size) < manager->sflag_id_num) {
		bitmap_set(manager->sflag_id_bmap, id, desc->size);
		desc->first_id = id;
	} else {
		desc->first_id = 0;
		ret = -EINVAL;
	}
	mutex_unlock(&manager->id_lock);

	return ret;
}

int aipu_job_manager_free_sflag_id(struct aipu_job_manager *manager,
				   struct aipu_id_desc *desc)
{
	if (!manager || !desc || !manager->sflag_id_bmap)
		return -EINVAL;

	if (desc->first_id >= manager->sflag_id_num ||
	    (desc->first_id + desc->size) >= manager->sflag_id_num)
		return -EINVAL;

	mutex_lock(&manager->id_lock);
	bitmap_clear(manager->sflag_id_bmap, desc->first_id, desc->size);
	mutex_unlock(&manager->id_lock);

	return 0;
}
