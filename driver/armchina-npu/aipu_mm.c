// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/types.h>
#include <linux/moduleparam.h>
#include <asm/cacheflush.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/iommu.h>
#include <linux/bitmap.h>
#include <linux/iova.h>
#include <linux/dma-mapping.h>
#include <linux/acpi.h>
#include "config.h"
#include "aipu_priv.h"
#include "aipu_mm.h"
#include "aipu_common.h"
#include "aipu_dma_buf.h"

/* Forward declarations for V3 custom IOVA support */
static int aipu_mm_v3_reserve_never_map_zones(struct aipu_memory_manager *mm);
static int aipu_mm_v3_block_invalidate(struct aipu_memory_manager *mm,
				       struct aipu_phy_block *block, u64 offset, u64 size);

/*
 * Invalidate a range WITHOUT cleaning and without the trailing
 * barrier -- flush_range_nb's contract, for the other half of the operation.
 *
 * A file-static and not a struct __cache_ops member on purpose: that struct is
 * embedded in struct aipu_memory_manager, so one more pointer in it moves every
 * field after it and displaces some forty functions that this change does not
 * touch. The ops table is identical for every mm anyway -- init_cache_ops fills
 * it from the arch, once -- so a static costs nothing but the symbol.
 */
static void (*aipu_invalidate_range_nb)(void *vaddr, size_t size);

#define EXIT_ENCODE_BUF_SIZE (1024)
struct exporter_dma_buf {
	void *vaddr;
	dma_addr_t paddr;
	size_t size;
	struct dma_buf *dmabuf;
	struct list_head list;
	struct page **pages;
	int fd;
};
static struct aipu_mem_region *aipu_mm_find_region_no_lock(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str);
static struct aipu_tcb_buf *aipu_mm_find_tbuf_in_region_no_lock(struct aipu_memory_manager *mm,
								struct aipu_mem_region *reg,
								u64 iova);
static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str);
static struct aipu_iova_buffer *aipu_mm_v3_find_buffer_by_iova(struct aipu_memory_manager *mm,
							      u64 iova);
static struct aipu_iova_buffer *aipu_mm_v3_find_owned_buffer_by_iova(struct aipu_memory_manager *mm,
								     u64 iova, struct file *filp);
static struct aipu_tcb_buf *aipu_mm_v3_find_tcb_buf_no_lock(struct aipu_memory_manager *mm,
							    u64 pa);
static int init_cache_ops(struct aipu_memory_manager *mm);
static void aipu_debug_print_buffers(struct aipu_memory_manager *mm);
static struct aipu_iova_buffer *aipu_get_iova_buffer(struct aipu_memory_manager *mm, u64 iova, char* str);
static char *aipu_mm_v3_kva_by_pa(struct aipu_memory_manager *mm, u64 dev_pa);

/*
 * The address indexes (kmd-walk). Every job submit resolves a handful of
 * caller-supplied addresses to their buffers: three ownership checks
 * (aipu_mm_v3_pa_owned_by), two kernel aliases of TCBs (aipu_mm_get_tcb) and
 * two TCB descriptors (aipu_mm_v3_find_tcb_buf_no_lock). Each used to walk a
 * list of every buffer on the device, ~30 ns a buffer under buffer_mutex: a
 * submit cost 22 us with a few buffers and 608 us with 20 000. The lists stay
 * (fd-close sweeps, exec_id lookups and the debug dump walk them, none per
 * submit); these trees answer the address lookups.
 *
 * mm->buffer_tree holds every buffer on buffer_list, keyed by iova_start,
 * under buffer_mutex. Buffer IOVA ranges come from the one iova_domain and are
 * removed from the tree before their IOVA is returned to it, so the ranges in
 * the tree are disjoint and the floor lookup below returns the one buffer the
 * list walk would have returned, or none.
 */
static void aipu_buffer_tree_insert(struct aipu_memory_manager *mm,
				    struct aipu_iova_buffer *buffer)
{
	struct rb_node **link = &mm->buffer_tree.rb_node, *parent = NULL;
	struct rb_node *nb;
	struct aipu_iova_buffer *b;

	while (*link) {
		parent = *link;
		b = rb_entry(parent, struct aipu_iova_buffer, rb);
		if (buffer->iova_start < b->iova_start)
			link = &parent->rb_left;
		else
			link = &parent->rb_right;
	}
	rb_link_node(&buffer->rb, parent, link);
	rb_insert_color(&buffer->rb, &mm->buffer_tree);

	/* the disjointness the lookups rely on; iova_domain guarantees it */
	nb = rb_prev(&buffer->rb);
	if (nb) {
		b = rb_entry(nb, struct aipu_iova_buffer, rb);
		WARN_ONCE(b->iova_start + b->iova_size > buffer->iova_start,
			  "aipu_mm: buffer IOVA 0x%llx+0x%llx overlaps 0x%llx+0x%llx\n",
			  buffer->iova_start, buffer->iova_size, b->iova_start, b->iova_size);
	}
	nb = rb_next(&buffer->rb);
	if (nb) {
		b = rb_entry(nb, struct aipu_iova_buffer, rb);
		WARN_ONCE(buffer->iova_start + buffer->iova_size > b->iova_start,
			  "aipu_mm: buffer IOVA 0x%llx+0x%llx overlaps 0x%llx+0x%llx\n",
			  buffer->iova_start, buffer->iova_size, b->iova_start, b->iova_size);
	}
}

static void aipu_buffer_tree_erase(struct aipu_memory_manager *mm,
				   struct aipu_iova_buffer *buffer)
{
	if (RB_EMPTY_NODE(&buffer->rb))
		return;
	rb_erase(&buffer->rb, &mm->buffer_tree);
	RB_CLEAR_NODE(&buffer->rb);
}

/* the buffer whose IOVA range contains @iova, or NULL; under buffer_mutex */
static struct aipu_iova_buffer *aipu_buffer_tree_find(struct aipu_memory_manager *mm, u64 iova)
{
	struct rb_node *n = mm->buffer_tree.rb_node;
	struct aipu_iova_buffer *b, *floor = NULL;

	while (n) {
		b = rb_entry(n, struct aipu_iova_buffer, rb);
		if (iova < b->iova_start) {
			n = n->rb_left;
		} else {
			floor = b;
			n = n->rb_right;
		}
	}
	if (floor && iova - floor->iova_start < floor->iova_size)
		return floor;
	return NULL;
}

/*
 * mm->v3_tcb_tree: the V3 TCB descriptors on v3_tcb_buf_head, keyed by head,
 * under mm->slock. A descriptor leaves the tree (and the list) when the
 * buffer holding it is destroyed -- by FREE_BUF as before, and now also at
 * fd close, which used to leave it on the list for the module's lifetime.
 */
static void aipu_tcb_tree_insert(struct aipu_memory_manager *mm, struct aipu_tcb_buf *tbuf)
{
	struct rb_node **link = &mm->v3_tcb_tree.rb_node, *parent = NULL;
	struct aipu_tcb_buf *t;

	while (*link) {
		parent = *link;
		t = rb_entry(parent, struct aipu_tcb_buf, rb);
		if (tbuf->head == t->head) {
			/* cannot happen while buffers are disjoint; keep the new one findable */
			WARN_ONCE(1, "aipu_mm: two TCB descriptors at 0x%llx\n", tbuf->head);
			rb_replace_node(parent, &tbuf->rb, &mm->v3_tcb_tree);
			RB_CLEAR_NODE(&t->rb);
			return;
		}
		if (tbuf->head < t->head)
			link = &parent->rb_left;
		else
			link = &parent->rb_right;
	}
	rb_link_node(&tbuf->rb, parent, link);
	rb_insert_color(&tbuf->rb, &mm->v3_tcb_tree);
}

/* unlink a V3 TCB descriptor from the tree and the list and free it; under slock */
static void aipu_tcb_buf_remove_no_lock(struct aipu_memory_manager *mm, struct aipu_tcb_buf *tbuf)
{
	if (!RB_EMPTY_NODE(&tbuf->rb)) {
		rb_erase(&tbuf->rb, &mm->v3_tcb_tree);
		RB_CLEAR_NODE(&tbuf->rb);
	}
	list_del(&tbuf->node);
	kmem_cache_free(mm->tbuf_cache, tbuf);
}

/* drop every V3 TCB descriptor whose head lies in [lo, hi) */
static void aipu_tcb_bufs_drop_range(struct aipu_memory_manager *mm, u64 lo, u64 hi)
{
	struct rb_node *n, *next;
	struct aipu_tcb_buf *t, *first = NULL;
	unsigned long flags;

	if (!mm->v3_tcb_buf_head || lo >= hi)
		return;

	spin_lock_irqsave(&mm->slock, flags);
	n = mm->v3_tcb_tree.rb_node;
	while (n) {
		t = rb_entry(n, struct aipu_tcb_buf, rb);
		if (t->head >= lo) {
			first = t;
			n = n->rb_left;
		} else {
			n = n->rb_right;
		}
	}
	for (n = first ? &first->rb : NULL; n; n = next) {
		t = rb_entry(n, struct aipu_tcb_buf, rb);
		if (t->head >= hi)
			break;
		next = rb_next(n);
		aipu_tcb_buf_remove_no_lock(mm, t);
	}
	spin_unlock_irqrestore(&mm->slock, flags);
}

int aipu_mm_hold_tcb_buf_alloc(struct aipu_memory_manager *mm, struct aipu_job *kern_job)
{
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	struct aipu_tcb_buf *prev_tbuf = NULL;
	struct aipu_tcb_buf *curr_tbuf = NULL;
	struct aipu_mem_region *reg = NULL;
	struct aipu_buf_request buf;
	unsigned long flags;
	int ret = 0;
	struct aipu_job_desc *desc = &kern_job->desc;
	struct aipu_tcb *tcb = NULL;
	struct aipu_tcb *prev = NULL;
	char *va = NULL;
	u64 encode_offset = sizeof(struct aipu_tcb) * 2;
	int malloc_flag = 0;
	bool find_htbuf = false;
	unsigned char exit_encode[] = {
		0x00, 0x80, 0x4f, 0x01,
		0xff, 0x7f, 0x0c, 0x40,
		0xff, 0x7f, 0x0c, 0x40,
		0xff, 0x7f, 0x0c, 0x40
	};

	// find idle hold tcb buffer
	spin_lock_irqsave(&mm->shlock, flags);
	list_for_each_entry(htbuf, &htbuf_head->node, node) {
		if (htbuf->status == AIPU_MEM_HOLD_TYPE_IDLE &&
		    htbuf->index != htbuf_head->hold_index) {
			htbuf->status = AIPU_MEM_HOLD_TYPE_LINKING;
			find_htbuf = true;
			dev_dbg(mm->dev, "found hold index %d tcb  0x%llx from list.\n",
				htbuf->index, htbuf->head);
			break;
		}
	}
	spin_unlock_irqrestore(&mm->shlock, flags);

	// malloc new hold tcb buffer
	if (!find_htbuf) {
		htbuf = kmem_cache_zalloc(mm->hold_tbuf_cache, GFP_KERNEL);
		if (!htbuf) {
			ret = -ENOMEM;
			dev_err(mm->dev, "alloc cache hold tcb buffer failed");
			goto CACHE_FAIL;
		}

		INIT_LIST_HEAD(&htbuf->node);
		memset(&buf, 0, sizeof(buf));
		buf.bytes = sizeof(struct aipu_tcb) + EXIT_ENCODE_BUF_SIZE;
		buf.align_in_page = 1;
		buf.region = AIPU_BUF_REGION_DEFAULT;
		buf.data_type = AIPU_MM_DATA_TYPE_TCB;
		buf.asid = AIPU_BUF_ASID_0;
		buf.alloc_mode = AIPU_DMA_BUF_MALLOC_DEFAULT;
		ret = aipu_mm_alloc(mm, &buf, NULL);
		if (ret) {
			dev_err(mm->dev, "alloc hold tcb buffer failed");
			ret = -ENOMEM;
			goto MALLOC_TCB_FAIL;
		}

		malloc_flag = 1;
		spin_lock_irqsave(&mm->shlock, flags);
		htbuf->desc = buf.desc;
		htbuf->head = buf.desc.pa;
		htbuf->next_head = 0;
		htbuf->prev_tail = 0;
		htbuf->prev_head = 0;
		htbuf->prev_hold_tcb = 0;
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINKING;
		htbuf->index = htbuf_head->nums;
		htbuf_head->nums++;
		list_add(&htbuf->node, &htbuf_head->node);
		spin_unlock_irqrestore(&mm->shlock, flags);

		va = aipu_mm_get_va(mm, htbuf->head + encode_offset);
		if (!va) {
			dev_err(mm->dev, "get encode va failed.");
			ret = -EFAULT;
			goto VA_FAIL;
		}

		memcpy(va, exit_encode, 16);
		dev_dbg(mm->dev, "malloc hold index %d\n", htbuf->index);
	}

	tcb = aipu_mm_get_tcb(mm, htbuf->head);
	if (!tcb) {
		dev_err(mm->dev, "get hold tcb buffer failed");
		ret = -EFAULT;
		goto VA_FAIL;
	}

	prev = aipu_mm_get_tcb(mm, desc->last_task_tcb_pa);
	if (!prev) {
		dev_err(mm->dev, "get prev tcb buffer failed for pa 0x%llx", desc->last_task_tcb_pa);
		ret = -EFAULT;
		goto VA_FAIL;
	}

	// Modify hold tcb data
	spin_lock_irqsave(&mm->shlock, flags);
	memcpy(tcb, prev, sizeof(struct aipu_tcb));
	tcb->flag = TCB_FLAG_TASK_TYPE_TASK;
	tcb->flag |= TCB_FLAG_END_TYPE_GROUP_END;
	tcb->flag |= TCB_FLAG_END_TYPE_GRID_END;
	tcb->next = 0;
	tcb->task.interrupt_en &= ~INTERRUPT_TEC;
	tcb->task.spc = lower_32_bits(htbuf->head + encode_offset -
					aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0));
	tcb->task.tcbp = lower_32_bits(htbuf->head - aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0));
	spin_unlock_irqrestore(&mm->shlock, flags);

	/* Find TCB buffer descriptors for hold TCB and task TCB */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->use_v3_custom_iova) {
		/* V3 Custom IOVA: search in the global V3 TCB buffer list */
		spin_lock_irqsave(&mm->slock, flags);
		curr_tbuf = aipu_mm_v3_find_tcb_buf_no_lock(mm, htbuf->head);
		if (!curr_tbuf) {
			dev_err(mm->dev, "V3: no TCB buffer found for hold iova 0x%llx",
				htbuf->head);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		prev_tbuf = aipu_mm_v3_find_tcb_buf_no_lock(mm, desc->last_task_tcb_pa);
		if (!prev_tbuf) {
			dev_err(mm->dev, "V3: no TCB buffer found for task iova 0x%llx",
				desc->last_task_tcb_pa);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		spin_unlock_irqrestore(&mm->slock, flags);
	} else {
		reg = aipu_mm_find_region(mm, htbuf->head, "find_tcb hold");
		if (!reg) {
			dev_err(mm->dev, "Can't found region at iova 0x%llx", htbuf->head);
			ret = -EFAULT;
			goto VA_FAIL;
		}

		spin_lock_irqsave(&mm->slock, flags);
		curr_tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, htbuf->head);
		if (!curr_tbuf) {
			dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx", htbuf->head);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		spin_unlock_irqrestore(&mm->slock, flags);

		reg = aipu_mm_find_region(mm, desc->last_task_tcb_pa, "find_tcb task");
		if (!reg) {
			dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx",
				desc->last_task_tcb_pa);
			ret = -EFAULT;
			goto VA_FAIL;
		}

		spin_lock_irqsave(&mm->slock, flags);
		prev_tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, desc->last_task_tcb_pa);
		if (!prev_tbuf) {
			dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx",
				desc->last_task_tcb_pa);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		spin_unlock_irqrestore(&mm->slock, flags);
	}

	spin_lock_irqsave(&mm->shlock, flags);
	curr_tbuf->tail = htbuf->head;
	curr_tbuf->tail_tcb = tcb;
	curr_tbuf->dep_job_id = 0;
	curr_tbuf->pinned = false;
	prev_tbuf->tail = desc->last_task_tcb_pa;
	prev_tbuf->tail_tcb = prev;
	prev->next = htbuf->head;
	htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_PREV;
	htbuf->hold_tcb = tcb;
	htbuf->prev_tail = desc->last_task_tcb_pa;
	htbuf->prev_head = desc->head_tcb_pa;
	kern_job->curr_hold_tcb = htbuf->head;
	htbuf->prev_tbuf = prev_tbuf;
	spin_unlock_irqrestore(&mm->shlock, flags);

	return ret;

VA_FAIL:
	spin_lock_irqsave(&mm->shlock, flags);
	if (malloc_flag == 1) {
		list_del(&htbuf->node);
		htbuf_head->nums--;
	} else {
		/* un-claim the reused buffer so the idle search finds it again */
		htbuf->status = AIPU_MEM_HOLD_TYPE_IDLE;
	}
	spin_unlock_irqrestore(&mm->shlock, flags);
	if (malloc_flag == 1) {
		aipu_mm_free(mm, &buf.desc, NULL, true);
		kmem_cache_free(mm->hold_tbuf_cache, htbuf);
	}
	return ret;
MALLOC_TCB_FAIL:
	kmem_cache_free(mm->hold_tbuf_cache, htbuf);
CACHE_FAIL:
	return ret;
}

static void aipu_mm_hold_tcb_buf_free(struct aipu_memory_manager *mm)
{
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *next = NULL;
	struct aipu_buf_desc buf;

	if (!htbuf_head)
		return;

	list_for_each_entry_safe(htbuf, next, &htbuf_head->node, node) {
		dev_dbg(mm->dev, "t %d i %d s %d f %d ch 0x%llx pht 0x%llx"
			" pt 0x%llx ph 0x%llx nh 0x%llx\n", htbuf_head->nums,
			htbuf->index,  htbuf->status, htbuf_head->hold_index,
			htbuf->head, htbuf->prev_hold_tcb, htbuf->prev_tail,
			htbuf->prev_head, htbuf->next_head);
		memset(&buf, 0, sizeof(buf));
		buf.pa = htbuf->head;
		buf.region = AIPU_BUF_REGION_DEFAULT;
		buf.asid = AIPU_BUF_ASID_0;
		aipu_mm_free(mm, &buf, NULL, true);
		list_del(&htbuf->node);
		kmem_cache_free(mm->hold_tbuf_cache, htbuf);
		htbuf_head->nums--;
	}
	htbuf_head->hold_index = -1;
	htbuf_head->hold_tcb = NULL;
}

struct aipu_hold_tcb_buf *aipu_mm_get_hold_htbuf(struct aipu_memory_manager *mm, u64 hold_tcb_pa)
{
	struct aipu_hold_tcb_buf *htbuf_head;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *ret = NULL;
	unsigned long flags = 0;

	if (!mm || !mm->hold_tcb_head)
		return NULL;

	htbuf_head = mm->hold_tcb_head;
	spin_lock_irqsave(&mm->shlock, flags);
	list_for_each_entry(htbuf, &htbuf_head->node, node) {
		if (htbuf->head == hold_tcb_pa) {
			ret = htbuf;
			break;
		}
	}
	spin_unlock_irqrestore(&mm->shlock, flags);

	return ret;
}

void aipu_mm_set_final_htbuf_index(struct aipu_memory_manager *mm, int index)
{
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	unsigned long flags = 0;

	spin_lock_irqsave(&mm->shlock, flags);
	htbuf_head->hold_index = index;
	spin_unlock_irqrestore(&mm->shlock, flags);
}

static struct aipu_tcb_buf *create_tcb_buf(struct aipu_memory_manager *mm,
					   struct aipu_mem_region *reg)
{
	struct aipu_tcb_buf *tbuf = NULL;

	tbuf = kmem_cache_zalloc(mm->tbuf_cache, GFP_KERNEL);
	if (tbuf) {
		INIT_LIST_HEAD(&tbuf->node);
		tbuf->reg = reg;
	}
	return tbuf;
}

static void destroy_tcb_buf(struct aipu_memory_manager *mm, struct aipu_tcb_buf *tbuf)
{
	unsigned long flags = 0;

	if (tbuf) {
		spin_lock_irqsave(&mm->slock, flags);
		list_del(&tbuf->node);
		spin_unlock_irqrestore(&mm->slock, flags);
		kmem_cache_free(mm->tbuf_cache, tbuf);
	}
}

static struct aipu_mem_region *aipu_mm_find_region_no_lock(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_iova_buffer *buffer = NULL;
	u32 asid;

	/* First search traditional regions */
	list_for_each_entry(obj, &mm->mem.head->list, list) {
		reg = obj->reg;
		if (reg && iova >= reg->base_iova && (iova - reg->base_iova < reg->bytes))
			return reg;
	}

	/* V3 Custom IOVA: search in IOVA buffers */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			u64 iova_addr = aipu_mm_v3_pa_to_iova(mm, iova, asid);
			if (!iova_addr)
				continue;
			buffer = aipu_mm_v3_find_buffer_by_iova(mm, iova_addr);
			if (buffer)
				return NULL; /* V3 buffer found but no traditional region */
		}
	}

	dev_err(mm->dev, "[%s] region not found: invalid address 0x%llx",
		log_str ? log_str : "find_reg", iova);
	return NULL;
}

static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm,
						   u64 iova, char *log_str)
{
	struct aipu_mem_region *reg = NULL;

	mutex_lock(&mm->lock);
	reg = aipu_mm_find_region_no_lock(mm, iova, log_str);
	mutex_unlock(&mm->lock);
	return reg;
}

static struct aipu_tcb_buf *aipu_mm_find_tbuf_in_region_no_lock(struct aipu_memory_manager *mm,
								struct aipu_mem_region *reg,
								u64 iova)
{
	struct aipu_tcb_buf *curr = NULL;

	list_for_each_entry(curr, &reg->tcb_buf_head->node, node) {
		if (iova >= curr->head && iova <= curr->tail)
			return curr;
	}

	return NULL;
}

static void aipu_mm_destroy_region(struct aipu_memory_manager *mm, struct aipu_mem_region *reg)
{
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_tcb_buf *next = NULL;

	if (!mm || !reg)
		return;

	/* release allocation to system */
	if (reg->base_va) {
		dma_free_attrs(reg->dev, reg->bytes, reg->base_va, reg->base_pa, reg->attrs);
		reg->base_va = NULL;
		reg->base_iova = 0;
	}

	if (reg->tcb_buf_head) {
		list_for_each_entry_safe(tbuf, next, &reg->tcb_buf_head->node, node)
			destroy_tcb_buf(mm, tbuf);

		destroy_tcb_buf(mm, reg->tcb_buf_head);
		reg->tcb_buf_head = NULL;
	}

	reg->bytes = 0;
	kmem_cache_free(mm->reg_cache, reg);
}

static struct aipu_mem_region *aipu_mm_create_region(struct aipu_memory_manager *mm,
						     struct aipu_mem_region_obj *obj,
						     enum aipu_mem_region_type type,
						     u64 size)
{
	int ret = 0;
	void *va = NULL;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !obj)
		return NULL;

	reg = kmem_cache_zalloc(mm->reg_cache, GFP_KERNEL);
	if (!reg)
		return NULL;

	reg->type = type;
	reg->bytes = size;
	reg->obj = obj;
	reg->dev = mm->dev;

	/* only head of the list is created; for v3; */
	reg->tcb_buf_head = create_tcb_buf(mm, reg);
	if (!reg->tcb_buf_head)
		goto err;

	/* create a list head, not a real region */
	if (!size)
		return reg;

	if (mm->has_iommu) {
		ret = dma_set_mask_and_coherent(mm->dev, DMA_BIT_MASK(mm->dma_mask));
		if (ret) {
			dev_err(reg->dev, "DMA set coherent mask failed (%d)!\n", ret);
			goto err;
		}
	}

	va = dma_alloc_attrs(reg->dev, reg->bytes, &reg->base_pa, GFP_KERNEL, reg->attrs);
	if (!va) {
		dev_err(reg->dev, "dma_alloc_attrs failed (bytes: 0x%llx, attrs %ld)\n",
			reg->bytes, reg->attrs);
		goto err;
	}
	reg->base_iova = reg->base_pa;
	reg->base_va = va;

	reg->locked = true;
	return reg;

err:
	aipu_mm_destroy_region(mm, reg);
	return NULL;
}

static struct aipu_mem_region_obj *aipu_mm_create_region_object(struct aipu_memory_manager *mm,
								enum aipu_mem_region_type type,
								u64 size, struct file *filp)
{
	struct aipu_mem_region_obj *obj = NULL;

	obj = kmem_cache_zalloc(mm->obj_cache, GFP_KERNEL);
	if (!obj)
		return NULL;

	obj->reg = aipu_mm_create_region(mm, obj, type, size);
	if (!obj->reg) {
		kmem_cache_free(mm->obj_cache, obj);
		return NULL;
	}

	obj->reg->filp = filp;
	INIT_LIST_HEAD(&obj->list);

	return obj;
}

static void aipu_mm_destroy_region_object(struct aipu_memory_manager *mm,
					  struct aipu_mem_region_obj *obj)
{
	if (obj) {
		aipu_mm_destroy_region(mm, obj->reg);
		obj->reg = NULL;
		list_del(&obj->list);
		kmem_cache_free(mm->obj_cache, obj);
	}
}

static int init_region_list(struct aipu_memory_manager *mm, struct aipu_mem_region_list *list)
{
	struct aipu_mem_region_obj *obj = NULL;

	obj = aipu_mm_create_region_object(mm, AIPU_MEM_REGION_TYPE_MEMORY, 0, NULL);
	if (!obj)
		return -ENOMEM;

	list->head = obj;
	return 0;
}

static int aipu_mm_alloc_in_region_no_lock(struct aipu_memory_manager *mm,
					   struct aipu_buf_request *buf_req,
					   struct aipu_mem_region *reg, struct file *filp,
					   struct aipu_tcb_buf **tbuf_alloc)
{
	unsigned long alloc_nr = 0;
	struct aipu_tcb_buf *tbuf = NULL;
	u64 dev_pa = 0;
	u32 dev_size = 0;

	/* filp == NULL means GM initialization or exit TCB allocation */
	if (!mm || !buf_req || !reg || !tbuf_alloc)
		return -EINVAL;

	if (buf_req->data_type == AIPU_MM_DATA_TYPE_TCB) {
		tbuf = create_tcb_buf(mm, reg);
		if (!tbuf)
			return -ENOMEM;
	}

	alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	dev_size = alloc_nr * PAGE_SIZE;
	dev_pa = reg->base_iova;

	if (tbuf) {
		/**
		 * when head/tail is updated in job manager, all the following fields
		 * should be updated together.
		 *
		 * NOTE: all the nodes of the TCB chain scheduled in a job should be within
		 * this buf!
		 */
		tbuf->head = dev_pa;
		tbuf->tail = tbuf->head + dev_size - sizeof(struct aipu_tcb);
		tbuf->tail_tcb = NULL;
		tbuf->dep_job_id = 0;
		tbuf->pinned = false;
		*tbuf_alloc = tbuf;
	}

	buf_req->desc.dev_offset = dev_pa;
	buf_req->desc.pa = dev_pa;
	buf_req->desc.bytes = dev_size;
	buf_req->desc.asid = buf_req->asid;
	buf_req->desc.region = reg->type;

	dev_dbg(reg->dev, "allocation in region done: asid %d iova 0x%llx, bytes 0x%llx, type %d\n",
		buf_req->asid, buf_req->desc.pa, buf_req->desc.bytes, buf_req->data_type);
	return 0;
}

static ssize_t aipu_gm_policy_sysfs_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;
	struct aipu_memory_manager *mm = &partition->priv->mm;

	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		return snprintf(buf, 128, "[%d] AIPU GM is shared by tasks of all QoS level.\n",
				AIPU_GM_POLICY_SHARED);
	} else if (mm->gm_policy == AIPU_GM_POLICY_HALF_DIVIDED) {
		return snprintf(buf, 128,
				"[%d] GM is divided half-by-half for QoS slow & fast tasks.\n",
				AIPU_GM_POLICY_HALF_DIVIDED);
	}

	return snprintf(buf, 128, "[%d] AIPU has no GM (or GM is disabled).\n",
			AIPU_GM_POLICY_NONE);
}

static ssize_t aipu_gm_policy_sysfs_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;
	struct aipu_memory_manager *mm = &partition->priv->mm;

	mutex_lock(&mm->lock);
	if ((strncmp(buf, "0", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_NONE;
	else if ((strncmp(buf, "1", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_SHARED;
	else if ((strncmp(buf, "2", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_HALF_DIVIDED;
	else
		dev_err(mm->dev, "[sysfs] invalid GM policy: gm_policy should be 0/1/2");
	mutex_unlock(&mm->lock);

	return count;
}

/**
 * @aipu_init_mm() - initialize mm module during driver probe phase
 * @mm:      pointer to memory manager struct to be initialized
 * @p_dev:   pointer to the platform device struct
 * @version: AIPU ISA version
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version)
{
	int ret = 0;
	struct iommu_group *group = NULL;

	if (!mm || !p_dev)
		return -EINVAL;

	memset(mm, 0, sizeof(*mm));
	mm->version = version;
	mm->dev = &p_dev->dev;
	mutex_init(&mm->lock);
	mm->importer_bufs = devm_kzalloc(mm->dev, sizeof(*mm->importer_bufs), GFP_KERNEL);
	if (!mm->importer_bufs)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->importer_bufs->node);
	INIT_LIST_HEAD(&mm->wbufs);
	INIT_LIST_HEAD(&mm->slots);
	mutex_init(&mm->wslot_lock);
	spin_lock_init(&mm->slock);
	spin_lock_init(&mm->shlock);
	mm->default_asid_base = 0;
	mm->default_asid_size = 0xC0000000;
	mm->valid_asid_cnt = 0;

	mm->obj_cache = kmem_cache_create("aipu_obj_cache", sizeof(struct aipu_mem_region_obj),
					  0, SLAB_PANIC, NULL);
	mm->reg_cache = kmem_cache_create("aipu_reg_cache", sizeof(struct aipu_mem_region),
					  0, SLAB_PANIC, NULL);
	mm->tbuf_cache = kmem_cache_create("aipu_tbuf_cache", sizeof(struct aipu_tcb_buf),
					   0, SLAB_PANIC, NULL);
	mm->hold_tbuf_cache = kmem_cache_create("aipu_hold_tbuf_cache",
						sizeof(struct aipu_hold_tcb_buf),
						0, SLAB_PANIC, NULL);
	if (!mm->obj_cache || !mm->reg_cache || !mm->tbuf_cache || !mm->hold_tbuf_cache)
		return -ENOMEM;

	ret = init_region_list(mm, &mm->mem);
	if (ret)
		goto finish;

	if (version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		mm->hold_tcb_head = kmem_cache_zalloc(mm->hold_tbuf_cache, GFP_KERNEL);
		if (!mm->hold_tcb_head) {
			ret = -ENOMEM;
			goto finish;
		}
		INIT_LIST_HEAD(&mm->hold_tcb_head->node);
		mm->hold_tcb_head->nums = 0;
	}
	/**
	 * Device tree binding for Zhouyi V3:
	 *
	 *    gm-policy = <POLICY_NO>;
	 *
	 * where POLICY_NO should be one of the following values:
	 * 0: no GM;
	 * 1: GM is shared by all tasks in 1 cluster (by default if this attribute is not provided)
	 * 2: GM is divided half-by-half: for QoS slow & fast tasks, respectively
	 */
	if (version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		ret = device_property_read_u32(mm->dev, "gm-policy", &mm->gm_policy);
		if (ret || mm->gm_policy > AIPU_GM_POLICY_HALF_DIVIDED)
			mm->gm_policy = AIPU_GM_POLICY_SHARED;

		if (IS_ERR(aipu_common_create_attr(mm->dev, &mm->gm_policy_attr, "gm_policy", 0644,
						   aipu_gm_policy_sysfs_show,
						   aipu_gm_policy_sysfs_store))) {
			mm->gm_policy_attr = NULL;
			dev_err(mm->dev, "create gm_policy attr failed");
		}

		dev_info(mm->dev, "GM policy is %s",
			 mm->gm_policy == AIPU_GM_POLICY_SHARED ? "shared" : "half-by-half");
	} else {
		mm->gm_policy = AIPU_GM_POLICY_NONE;
		mm->gm_policy_attr = NULL;
	}

	group = iommu_group_get(mm->dev);
	if (group) {
		mm->has_iommu = true;
		mm->iommu_domain = iommu_get_domain_for_dev(mm->dev);
		if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
			/*
			 * V3 + IOMMU: Use custom IOVA domain to support per-ASID 3GB range
			 * Enable discrete physical page allocation while keeping TCB
			 * relative addresses within 32-bit range.
			 */
			ret = aipu_mm_v3_init_iova_domain(mm);
			if (ret) {
				/*
				 * fail the probe instead of falling
				 * back: the fallback this branch used to limp
				 * into was deleted as dead weight, and its
				 * forbidden-zone reservation loop could hang
				 * the probe outright.
				 */
				dev_err(mm->dev, "V3: failed to init custom IOVA domain (%d)\n", ret);
			} else {
				mm->use_v3_custom_iova = true;
				mm->valid_asid_cnt = 4;
			}
		} else {
			mm->valid_asid_cnt = 1;
		}
	}
	iommu_group_put(group);
	if (ret)
		goto finish;
	dev_info(mm->dev, "AIPU is%s behind an IOMMU.\n", mm->has_iommu ? "" : " not");

finish:
	if (ret)
		aipu_deinit_mm(mm);
	return ret;
}

/**
 * @aipu_deinit_mm() - de-initialize mm module while kernel module unloading
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 */
int aipu_deinit_mm(struct aipu_memory_manager *mm)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region_obj *next = NULL;

	if (mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (mm->gm_policy_attr) {
			aipu_common_destroy_attr(mm->dev, &mm->gm_policy_attr);
			mm->gm_policy_attr = NULL;
		}
	}

	/*
	 * aipu_init_mm() creates hold_tbuf_cache unconditionally;
	 * a version guard here leaked it on non-V3.
	 */
	if (mm->hold_tbuf_cache) {
		aipu_mm_hold_tcb_buf_free(mm);
		/*
		 * hold_tcb_head is NULL when its allocation failed
		 * (or on non-V3); kmem_cache_free() does not accept NULL.
		 */
		if (mm->hold_tcb_head) {
			kmem_cache_free(mm->hold_tbuf_cache, mm->hold_tcb_head);
			mm->hold_tcb_head = NULL;
		}
		kmem_cache_destroy(mm->hold_tbuf_cache);
		mm->hold_tbuf_cache = NULL;
	}

	if (mm->mem.head) {
		list_for_each_entry_safe(obj, next, &mm->mem.head->list, list)
			aipu_mm_destroy_region_object(mm, obj);

		aipu_mm_destroy_region_object(mm, mm->mem.head);
		mm->mem.head = NULL;
	}

	aipu_mm_v3_deinit_iova_domain(mm);
	kmem_cache_destroy(mm->obj_cache);
	mm->obj_cache = NULL;
	kmem_cache_destroy(mm->reg_cache);
	mm->reg_cache = NULL;
	kmem_cache_destroy(mm->tbuf_cache);
	mm->tbuf_cache = NULL;
	mm->iommu_domain = NULL;
	return 0;
}

static int aipu_mm_direct_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
				struct file *filp, struct aipu_tcb_buf **tcb_alloc)
{
	int ret = 0;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_tcb_buf *tcb = NULL;

	obj = aipu_mm_create_region_object(mm, AIPU_MEM_REGION_TYPE_MEMORY, buf_req->bytes, filp);
	if (!obj || !obj->reg)
		return -ENOMEM;

	ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, obj->reg, filp, &tcb);
	if (ret) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_lock(&mm->lock);
	list_add(&obj->list, &mm->mem.head->list);
	mutex_unlock(&mm->lock);

	if (tcb_alloc)
		*tcb_alloc = tcb;
	return 0;

err:
	aipu_mm_destroy_region_object(mm, obj);
	return ret;
}

static bool aipu_mm_check_address_validity(struct aipu_memory_manager *mm,
					   struct aipu_buf_desc *desc)
{
	u64 asid_size = 0;
	u64 asid_base = 0;
	u64 remainder = 0;
	u64 lower_bound = 0;
	u64 upper_bound = 0;
	u64 desc_end = 0;
	u64 asid0_base = 0;
	u64 size_4G = 0x100000000ULL;
	u64 relative_offset;

	/* V3 with custom IOVA: check relative offset is within 3GB */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->use_v3_custom_iova) {
		asid_base = aipu_mm_get_asid_base(mm, desc->asid);
		
		/* Check pa is in valid ASID range */
		if (desc->pa < asid_base ||
		    desc->pa >= asid_base + mm->asid_iova[desc->asid].iova_size) {
			dev_err(mm->dev, "V3: pa 0x%llx outside ASID %d range [0x%llx, 0x%llx)\n",
				desc->pa, desc->asid, asid_base,
				asid_base + mm->asid_iova[desc->asid].iova_size);
			return false;
		}
		
		/* Check relative offset within 3GB */
		relative_offset = desc->pa - asid_base;
		if (relative_offset >= 0xC0000000ULL) {
			dev_err(mm->dev, "V3: relative offset 0x%llx >= 3GB\n", relative_offset);
			return false;
		}
		
		/* Check doesn't cross 3GB boundary */
		if (relative_offset + desc->bytes > 0xC0000000ULL) {
			dev_err(mm->dev, "V3: buffer crosses 3GB boundary\n");
			return false;
		}
		
		return true;
	}

	/* Original V3 fallback mode */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		remainder = desc->pa % size_4G;
		if ((remainder + desc->bytes) >= size_4G)
			return false;
	}

	asid_size = aipu_mm_get_asid_size(mm, desc->asid);
	if (asid_size <= SZ_2G)
		return true;

	asid_base = aipu_mm_get_asid_base(mm, desc->asid);
	if (mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		lower_bound = asid_base + size_4G - SZ_1G;
		upper_bound = asid_base + size_4G;

			if (desc->asid > AIPU_BUF_ASID_0) {
				asid0_base = aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0);
				if (asid0_base != asid_base)
					lower_bound += SZ_512M;
			}

		desc_end = desc->pa + desc->bytes;
		if (desc->pa < upper_bound && desc_end >= lower_bound)
			return false;
	}

	return true;
}

/**
 * @aipu_mm_alloc() - alloc memory buffer for user request
 * @mm:      pointer to memory manager struct initialized in aipu_init_mm()
 * @buf_req: pointer to buffer request struct from userland
 * @filp:    pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp)
{
	int ret = 0;
	unsigned long flags;
	struct aipu_tcb_buf *tbuf = NULL;
	bool allocated = false;

	if (!mm || !buf_req)
		return -EINVAL;

	if (!buf_req->bytes || !is_power_of_2(buf_req->align_in_page) ||
	    buf_req->asid >= ZHOUYI_ASID_COUNT) {
		dev_err(mm->dev, "[malloc] invalid alloc request: bytes 0x%llx, align %u, asid %u",
			buf_req->bytes, buf_req->align_in_page, buf_req->asid);
		return -EINVAL;
	}

	if (buf_req->region >= AIPU_MEM_REGION_TYPE_MAX) {
		dev_err(mm->dev, "[malloc] invalid alloc request: region type %d",
			buf_req->region);
		return -EINVAL;
	}

	/* V3 with custom IOVA: use dedicated IOVA allocation */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->use_v3_custom_iova)
		ret = aipu_alloc_dma_iova_phy(mm, buf_req, filp);
	else
		ret = aipu_mm_direct_alloc(mm, buf_req, filp, &tbuf);
	allocated = !ret;

	if (tbuf) {
		spin_lock_irqsave(&mm->slock, flags);
		list_add_tail(&tbuf->node, &tbuf->reg->tcb_buf_head->node);
		spin_unlock_irqrestore(&mm->slock, flags);
	}

	/*
	 * Only inspect desc when the allocation above actually succeeded: on a
	 * failure path buf_req->desc still holds the wholesale copy from
	 * userspace, and aipu_mm_free would then free a caller-named descriptor.
	 */
	if (!ret && allocated && mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (!aipu_mm_check_address_validity(mm, &buf_req->desc)) {
			aipu_mm_free(mm, &buf_req->desc, filp, true);
			allocated = false;
			ret = -ENOMEM;
		}
	}

	if (!allocated) {
		dev_err(mm->dev,
			"failed allocating: asid %d iova 0x%llx, bytes 0x%llx,"
			" page align %d, type %d\n",
			buf_req->asid, buf_req->desc.pa,
			buf_req->bytes, buf_req->align_in_page,
			buf_req->data_type);
	} else {
		dev_dbg(mm->dev,
			"allocate done: asid %d iova 0x%llx, type %d, "
			"bytes 0x%llx (al %d, rg %d)\n",
			buf_req->asid, buf_req->desc.pa, buf_req->data_type,
			buf_req->bytes, buf_req->align_in_page,
			buf_req->desc.region);
	}
	return ret;
}

static int aipu_mm_direct_free(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
			       bool unlock)
{
	struct aipu_tcb_buf *tbuf = NULL;
	u64 iova = 0;

	if (!mm || !reg)
		return -EINVAL;

	if (unlock)
		reg->locked = false;
	else if (reg->locked)
		return DEFERRED_FREE;

	iova = reg->base_iova;

	/* do not return tbuf for deferred free cases */
	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, iova);
	if (tbuf && (tbuf->dep_job_id || tbuf->pinned)) {
		dev_info(reg->dev, "deferred free (direct) TCB: iova 0x%llx\n", iova);
		return DEFERRED_FREE;
	}

	/* do free */
	aipu_mm_destroy_region_object(mm, reg->obj);

	dev_dbg(mm->dev, "direct free done: iova 0x%llx\n", iova);
	return 0;
}

/**
 * @aipu_mm_free() - free buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be released
 * @filp: pointer to the file struct
 * @unlock: unlock the page or not
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_free(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp,
		 bool unlock)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !buf)
		return -EINVAL;

	/* V3 Custom IOVA: use dedicated free path */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		return aipu_free_dma_iova_phy(mm, buf, filp);
	}

	reg = aipu_mm_find_region(mm, buf->pa, "free");
	if (!reg)
		return -EINVAL;

	mutex_lock(&mm->lock);
	ret = aipu_mm_direct_free(mm, reg, unlock);
	mutex_unlock(&mm->lock);

	if (ret == DEFERRED_FREE)
		ret = 0;

	return ret;
}

static void tensor_dcache_inval_poc(void *start, void *end) {
    uintptr_t line_size, addr_start, addr_end, tmp;

    line_size = cache_line_size();
    tmp = line_size - 1;

    addr_start = (uintptr_t)start;
    addr_end = (uintptr_t)end;

    if (addr_end & tmp) {
        addr_end &= ~tmp;
        __asm__ __volatile__("dc civac, %0" : : "r" (addr_end) : "memory");
    }

    if (addr_start & tmp) {
        addr_start &= ~tmp;
        __asm__ __volatile__("dc civac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
    }

    while (addr_start < addr_end) {
        __asm__ __volatile__("dc ivac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
    }

    __asm__ __volatile__("dsb sy" : : : "memory");
}

static void tensor_flush_dcache_range(void *start, void *end) {
	uintptr_t addr_start, addr_end;
	const uint64_t line_size = cache_line_size();

	addr_start = (uintptr_t)start;
	addr_end = (uintptr_t)end;

	// make sure addr_start and addr_end are cache line aligned
	addr_start = addr_start & ~(line_size - 1);
	addr_end = (addr_end + line_size - 1) & ~(line_size - 1);

	while (addr_start < addr_end) {
		__asm__ volatile ("dc cvac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
	}

	// Ensure all cache operations complete
	__asm__ volatile ("dsb sy" : : : "memory");

}

/**
 * aipu_mm_v3_block_pages_cmo_nb() - CPU cache maintenance over part of a block
 * @mm:     the memory manager
 * @block:  the physical block
 * @offset: byte offset into the block to start at
 * @size:   bytes to cover (clamped to the block)
 * @invalidate: invalidate without cleaning first, rather than clean+invalidate
 *
 * Through the LINEAR map (page_address), which is the one cacheable alias these
 * pages have. WITHOUT a completion barrier: every caller issues exactly one
 * after its last block, which is why this and aipu_mm_v3_flush_block_pages_nb
 * share it rather than emitting a full-system dsb per 4KB page.
 *
 * The user mapping is Normal-NC and the kernel vmap alias
 * is pgprot_noncached, so maintaining either -- which is what the BUF_CACHE
 * ioctl pair did until now -- is a no-op, and that is why they "made no
 * difference" wherever they were tried.
 *
 * It is not a no-op on the linear map, and the difference is a live defect: a
 * CLEAN stale line for these physical addresses survives in the CPU hierarchy
 * (the allocator's zeroing populates it, and the linear map is speculatively
 * accessible thereafter), and the host's non-cacheable reads are served from it
 * for as long as it lives -- measured alive 50 ms after the job completed, and
 * surviving a second read of the same bytes. Invalidating is what clears it;
 * cleaning alone does nothing, which is how we know the line is clean.
 *
 * Return: 0, or -EOPNOTSUPP if this arch registered no maintenance op -- never
 * a silent success, which is the failure mode this replaces.
 */
static int aipu_mm_v3_block_pages_cmo_nb(struct aipu_memory_manager *mm,
					 struct aipu_phy_block *block,
					 u64 offset, u64 size, bool invalidate)
{
	void (*op)(void *, size_t);
	u32 first, last, i;

	if (invalidate)
		op = aipu_invalidate_range_nb ? aipu_invalidate_range_nb :
						mm->cache_ops.invalidate_range;
	else
		op = mm->cache_ops.flush_range_nb ? mm->cache_ops.flush_range_nb :
						    mm->cache_ops.flush_range;
	if (!op)
		return -EOPNOTSUPP;
	if (!block->pages)
		return 0;

	if (offset >= block->size)
		return 0;
	size = min(size, block->size - offset);
	if (!size)
		return 0;

	first = offset >> PAGE_SHIFT;
	last = (offset + size - 1) >> PAGE_SHIFT;
	if (last >= block->page_count)
		last = block->page_count - 1;

	for (i = first; i <= last; i++) {
		void *va = page_address(block->pages[i]);

		if (va)
			op(va, PAGE_SIZE);
	}

	return 0;
}

/* The two above, each with the single trailing barrier its caller owes. */
static int aipu_mm_v3_block_flush(struct aipu_memory_manager *mm,
				  struct aipu_phy_block *block, u64 offset, u64 size)
{
	int ret = aipu_mm_v3_block_pages_cmo_nb(mm, block, offset, size, false);

	if (!ret && mm->cache_ops.flush_barrier)
		mm->cache_ops.flush_barrier();
	return ret;
}

static int aipu_mm_v3_block_invalidate(struct aipu_memory_manager *mm,
				       struct aipu_phy_block *block, u64 offset, u64 size)
{
	int ret = aipu_mm_v3_block_pages_cmo_nb(mm, block, offset, size, true);

	if (!ret && mm->cache_ops.flush_barrier)
		mm->cache_ops.flush_barrier();
	return ret;
}

/**
 * @aipu_mm_cache_flush() - invald buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be invalid
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_cache_flush(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_phy_block *block;
	u64 iova;
	u32 asid;

	if (!mm || !buf)
		return -EINVAL;

	/* V3 custom IOVA */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, buf->pa, asid);
			if (!iova)
				continue;

			/* only flush a buffer this fd owns */
			if (!aipu_mm_v3_find_owned_buffer_by_iova(mm, iova, filp))
				continue;

			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block) {
				/* The LINEAR alias, over the
				 * range the caller named; see
				 * aipu_mm_v3_block_pages_cmo_nb for why that
				 * is the alias with something to maintain.
				 *
				 * Owned is a success whether or not the block
				 * carries a kernel alias, so this stays
				 * invariant in how the caller described its
				 * allocation.
				 */
				return aipu_mm_v3_block_flush(mm, block,
							      iova - block->iova_start,
							      buf->bytes);
			}
		}
		/*
		 * No owned block resolved: report the failure instead of the
		 * old silent success, which left userspace to proceed on stale
		 * data.
		 */
		dev_dbg(mm->dev, "V3: cache_flush no owned block for pa 0x%llx\n", buf->pa);
		return -EINVAL;
	}

	reg = aipu_mm_find_region(mm, buf->pa, "flush");
	if (!reg)
	{
		dev_err(mm->dev, "The region of buf->pa 0x%llx not found\n", buf->pa);
		return 0;
	}

	if(mm->has_iommu) {
		mutex_lock(&mm->lock);
		tensor_flush_dcache_range(reg->base_va, (void *)reg->base_va + reg->bytes);
		mutex_unlock(&mm->lock);
	}

	return ret;
}

/**
 * @aipu_mm_cache_invalid() - invald buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be invalid
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_cache_invalid(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_phy_block *block;
	u64 iova;
	u32 asid;

	if (!mm || !buf)
		return -EINVAL;

	/* V3 custom IOVA */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, buf->pa, asid);
			if (!iova)
				continue;

			/*
			 * only invalidate a buffer this fd owns: a stray
			 * invalidate discards another process's dirty lines
			 *
			 */
			if (!aipu_mm_v3_find_owned_buffer_by_iova(mm, iova, filp))
				continue;

			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block) {
				/* As in aipu_mm_cache_flush above.
				 * Invalidate-only is what a host about to
				 * read NPU-written data wants, and it is the
				 * cheaper half; it discards any dirty line,
				 * which for these pages means the allocator's
				 * zeroing if a job overwrote it first -- the
				 * caller asking for this ioctl is asking for
				 * the device's bytes. Owned is a success
				 * either way.
				 */
				return aipu_mm_v3_block_invalidate(mm, block,
								   iova - block->iova_start,
								   buf->bytes);
			}
		}
		dev_dbg(mm->dev, "V3: cache_invalid no owned block for pa 0x%llx\n", buf->pa);
		return -EINVAL;
	}

	reg = aipu_mm_find_region(mm, buf->pa, "invalid");
	if (!reg)
	{
		dev_err(mm->dev, "The region of buf->pa 0x%llx not found\n", buf->pa);
		return 0;
	}

	if(mm->has_iommu) {
		mutex_lock(&mm->lock);
		tensor_dcache_inval_poc(reg->base_va, (void *)reg->base_va + reg->bytes);
		mutex_unlock(&mm->lock);
	}

	return ret;
}

/**
 * @aipu_mm_free_buffers() - free all the buffers allocated from one fd
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 */
void aipu_mm_free_buffers(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region_obj *next = NULL;

	/* V3 custom IOVA cleanup */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		aipu_mm_v3_free_buffers_by_filp(mm, filp);
		return;
	}

	mutex_lock(&mm->lock);
	list_for_each_entry_safe(obj, next, &mm->mem.head->list, list) {
		if (obj->reg->filp == filp)
			aipu_mm_direct_free(mm, obj->reg, true);
			/* --- reg should not be used below --- */
	}
	mutex_unlock(&mm->lock);
}

static int aipu_vma_map_pages(struct vm_area_struct *vma, struct page **pages,
				unsigned long num)
{
	unsigned long count = vma_pages(vma);
	unsigned long uaddr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff;
	int ret, i;

	/* Fail if the user requested offset is beyond the end of the object */
	if (offset >= num)
		return -ENXIO;

	/* Fail if the user requested size exceeds available object size */
	if (count > num - offset)
		return -ENXIO;

	for (i = 0; i < count; i++) {
		ret = vm_insert_page(vma, uaddr, pages[offset + i]);
		if (ret < 0)
			return ret;
		uaddr += PAGE_SIZE;
	}

	return 0;
}

/**
 * @aipu_mm_mmap_buf() - mmap an allocated buffer for user thread
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @vma: pointer to the vm_area_struct
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_mmap_buf(struct aipu_memory_manager *mm, struct vm_area_struct *vma, struct file *filp)
{
	int ret = 0;
	dma_addr_t iova = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_phy_block *block = NULL;
	dma_addr_t dev_pa;
	u32 asid;

	if (!mm || !vma)
		return -EINVAL;

	dev_pa = vma->vm_pgoff << PAGE_SHIFT;
	if ((u64)dev_pa >= AIPU_WBUF_MMAP_BASE)		/* a zero-copy weight buffer, not an NPU-view address */
		return aipu_wbuf_mmap(mm, vma, filp);

	vm_flags_set(vma, VM_IO);
	vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);

	/* V3 custom IOVA: dev_pa is NPU view, convert to IOVA */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, dev_pa, asid);
			if (!iova)
				continue;

			/* only map a buffer this fd owns */
			if (!aipu_mm_v3_find_owned_buffer_by_iova(mm, iova, filp))
				continue;

			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block)
				break;
		}

		if (!block) {
			dev_err(mm->dev, "V3 mmap: no owned block for pa 0x%llx\n", dev_pa);
			return -EINVAL;
		}

		/*
		 * Honour the offset of dev_pa within its block instead of always
		 * mapping from page 0: the caller may mmap partway into a buffer,
		 * and aipu_vma_map_pages bounds-checks this page offset against the
		 * block's page_count.
		 */
		vma->vm_pgoff = (iova - block->iova_start) >> PAGE_SHIFT;
		ret = aipu_vma_map_pages(vma, block->pages, block->page_count);
		if (ret)
			dev_err(mm->dev, "V3 mmap failed at pa 0x%llx (ret = %d)", dev_pa, ret);
	} else {
		iova = dev_pa;
		reg = aipu_mm_find_region(mm, iova, "mmap");
		if (!reg)
			return -EINVAL;
		vma->vm_pgoff = (iova - reg->base_iova) >> PAGE_SHIFT;
		ret = dma_mmap_attrs(reg->dev, vma, reg->base_va, reg->base_pa,
				reg->bytes, reg->attrs);
		if (ret)
			dev_err(mm->dev, "dma_mmap_attrs failed at iova 0x%llx (ret = %d)", iova, ret);
	}

	vma->vm_pgoff = dev_pa >> PAGE_SHIFT;

	return ret;
}

/**
 * @aipu_mm_get_va() - get the kernel virtual address of an allocated buffer.
 *                     currently do not support non-tcb va from importers.
 * @mm:     pointer to memory manager struct initialized in aipu_init_mm()
 * @dev_pa: device physical address returned by aipu_mm_alloc()
 *
 * Return: 0 on success and error code otherwise.
 */
char *aipu_mm_get_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;
	char *va;

	if (!mm)
		return NULL;

	/* V3 custom IOVA path */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		va = aipu_mm_v3_kva_by_pa(mm, dev_pa);
		if (va)
			return va;
		dev_dbg(mm->dev, "V3: get_va failed for pa 0x%llx\n", dev_pa);
		return NULL;
	}

	/* Original path */
	reg = aipu_mm_find_region(mm, dev_pa, "get_va");
	if (!reg)
		return NULL;

	return (char *)((unsigned long)reg->base_va + dev_pa - reg->base_iova);
}

/**
 * @aipu_mm_get_tcb() - get the pointer to the TCB of a TCB list.
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @pa: address of the TCB
 *
 * Return: tcb va on success or NULL otherwise.
 */
struct aipu_tcb *aipu_mm_get_tcb(struct aipu_memory_manager *mm, u64 pa)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_tcb_buf *tbuf = NULL;
	unsigned long flags;
	struct aipu_tcb *tcb = NULL;
	char *va = NULL;

	if (!mm)
		return NULL;

	/* V3 custom IOVA: pa is NPU view, need to convert to IOVA and find block */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		va = aipu_mm_v3_kva_by_pa(mm, pa);
		if (va)
			return (struct aipu_tcb *)va;
		dev_err(mm->dev, "[get_tcb] V3: no TCB buffer at pa 0x%llx\n", pa);
		return NULL;
	}

	/* Original path */
	reg = aipu_mm_find_region(mm, pa, "find_tcb");
	if (!reg)
		return NULL;
	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, pa);
	if (!tbuf) {
		dev_err(mm->dev, "[get_tcb] no TCB buffer is found at iova 0x%llx", pa);
		goto unlock;
	}

	reg = tbuf->reg;
	tcb = (struct aipu_tcb *)((char *)(reg->base_va) + pa - reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return tcb;
}

/**
 * @aipu_mm_link_tcb() - link a TCB list to the tail of an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @prev_tail:    address of the tail TCB of a previous TCB list to be linked
 * @next_head_32: head address of the next TCB list
 * @next_job_id:  Job ID of the next TCB list
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		     int next_job_id)
{
	struct aipu_hold_tcb_buf *htbuf = NULL;
	unsigned long flags;
	int ret = 0;
	u32 temp = 0;

	htbuf = aipu_mm_get_hold_htbuf(mm, prev_tail);
	if (!htbuf)
		return -EFAULT;

	spin_lock_irqsave(&mm->shlock, flags);
	if (!htbuf->hold_tcb) {
		dev_err(mm->dev, "hold TCB was not set before linking it");
		spin_unlock_irqrestore(&mm->shlock, flags);
		return -EFAULT;
	}
	htbuf->hold_tcb->next = next_head_32;
	htbuf->hold_tcb->flag &= ~TCB_FLAG_END_TYPE_GRID_END;
	temp = htbuf->hold_tcb->next;
	htbuf->next_head = next_head_32;
	if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_PREV)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINKED;
	else if (htbuf->status == AIPU_MEM_HOLD_TYPE_IDLE)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_NEXT;
	else
		dev_info(mm->dev, "ABNORMAL STATUS link hold tcb index %d status %d\n",
			 htbuf->index, htbuf->status);

	dev_dbg(mm->dev, "new tcb task link prev htcb index %d status %d\n",
		htbuf->index, htbuf->status);

	spin_unlock_irqrestore(&mm->shlock, flags);
	return ret;
}

/**
 * @aipu_mm_unlink_tcb() - unlink a TCB list from an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @curr_hold:    address of the hold TCB of the TCB list to be unlinked
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 curr_hold)
{
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *prev_htbuf = NULL;
	unsigned long flags;
	int ret = 0;
	u32 temp = 0;
	int prev_status = -1, prev_index = -1;

	if (!mm)
		return -EINVAL;

	htbuf = aipu_mm_get_hold_htbuf(mm, curr_hold);
	if (!htbuf)
		return -EFAULT;
	if (htbuf->prev_hold_tcb != 0) {
		prev_htbuf = aipu_mm_get_hold_htbuf(mm, htbuf->prev_hold_tcb);
		if (!prev_htbuf)
			dev_info(mm->dev, "PREV HOLD TCB 0x%llx IS NOT SET.", htbuf->prev_hold_tcb);
	}
	spin_lock_irqsave(&mm->shlock, flags);

	if (!htbuf->hold_tcb) {
		dev_err(mm->dev, "hold TCB was not set before unlinking it");
		ret = -EINVAL;
		goto unlock;
	}

	/*
	 * Job teardown vs IRQ bottom-half can race: prev_tbuf / tail_tcb or the
	 * previous hold buffer may already be cleared/freed. Dereferencing NULL
	 * here caused kernel Oops (e.g. NULL + 0x4) in aipu_mm_unlink_tcb.
	 */
	if (htbuf->prev_hold_tcb != 0) {
		if (!prev_htbuf || !prev_htbuf->hold_tcb) {
			dev_err(mm->dev,
				"unlink: invalid prev hold chain (prev_pa 0x%llx hold idx %d)\n",
				htbuf->prev_hold_tcb, htbuf->index);
			ret = -EINVAL;
			goto unlock;
		}
	}

	tbuf = htbuf->prev_tbuf;
	if (!tbuf || !tbuf->tail_tcb) {
		dev_err(mm->dev,
			"unlink: prev_tbuf or tail_tcb is NULL (hold idx %d)\n",
			htbuf->index);
		ret = -EINVAL;
		goto unlock;
	}

	if (prev_htbuf) {
		prev_htbuf->hold_tcb->next = 0;
		temp = prev_htbuf->hold_tcb->next;
		if (prev_htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_NEXT) {
			prev_htbuf->status = AIPU_MEM_HOLD_TYPE_IDLE;
		} else if (prev_htbuf->status == AIPU_MEM_HOLD_TYPE_LINKED) {
			prev_htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_PREV;
		} else {
			dev_info(mm->dev, "ABNORMAL UNLINK PREV HOLD TCB STATUS %d\n",
				 prev_htbuf->status);
		}
		htbuf->prev_hold_tcb = 0;
		prev_status = prev_htbuf->status;
		prev_index = prev_htbuf->index;
	}

	tbuf->tail_tcb->next = 0;
	tbuf->dep_job_id = 0;
	tbuf->pinned = false;
	temp = tbuf->tail_tcb->next;
	htbuf->prev_head = 0;
	htbuf->prev_tail = 0;

	if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINKED)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_NEXT;
	else if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_PREV)
		htbuf->status = AIPU_MEM_HOLD_TYPE_IDLE;
	else
		dev_info(mm->dev, "ABNORMAL UNLINK CUR HOLD TCB STATUS %d\n", htbuf->status);

	dev_dbg(mm->dev, "unlink prev hold index %d status %d, "
		"unlink curr hold tcb index %d status %d final holder index %d",
		prev_index, prev_status, htbuf->index, htbuf->status,
		mm->hold_tcb_head->hold_index);

unlock:
	spin_unlock_irqrestore(&mm->shlock, flags);
	htbuf->prev_tbuf = NULL;
	return ret;
}

u64 aipu_mm_get_asid_base(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	return mm->default_asid_base;
}

u64 aipu_mm_get_asid_size(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	return mm->default_asid_size;
}

void aipu_mm_get_asid(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	int i;

	if (!mm || !cap)
		return;

	for (i = 0; i < mm->valid_asid_cnt; i++)
		cap->asid_base[i] = aipu_mm_get_asid_base(mm, i);
}

u32 aipu_mm_get_asid_cnt(struct aipu_memory_manager *mm)
{
	if (!mm)
		return 0;
	return mm->valid_asid_cnt;
}

int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes)
{
	if (!mm || !bytes || mm->gm_policy == AIPU_GM_POLICY_NONE)
		return -EINVAL;

	mm->gm_bytes = bytes;
	return 0;
}

int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next)
{
	int ret = 0;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_V3)
		return -EINVAL;

	if (next == mm->gm_policy)
		return ret;

	mutex_lock(&mm->lock);
	mm->gm_policy = next;
	mutex_unlock(&mm->lock);
	return ret;
}

void aipu_mm_get_gm(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	if (!mm || !cap)
		return;

	cap->gm0_size = 0;
	cap->gm1_size = 0;

	mutex_lock(&mm->lock);
	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		cap->gm0_size = mm->gm_bytes;
	} else {
		cap->gm0_size = mm->gm_bytes >> 1;
		cap->gm1_size = cap->gm0_size;
	}
	mutex_unlock(&mm->lock);
}

static void aipu_debug_print_buffers(struct aipu_memory_manager *mm)
{
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;
	int count = 0, index = 0;

	if (!mm)
		return;
	dev_info(mm->dev, "------------------------S-------------------------------------\n");

	//mutex_lock(&mm->buffer_mutex);
	dev_info(mm->dev, "Total buffers: %u\n", mm->buffer_count);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		dev_info(mm->dev, "Buffer %d: exec id 0x%llx IOVA=0x%llx, size=0x%llx, allocated=0x%llx ref_count %d\n",
			count++, buffer->exec_id, buffer->iova_start, buffer->iova_size, buffer->allocated_size, buffer->ref_count);
		index = 0;
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			if (block) {
				dev_info(mm->dev, "block %d: IOVA=0x%llx, size=0x%llx\n",
					index++, block->iova_start, block->size);
			}
		}
	}
	//mutex_unlock(&mm->buffer_mutex);
	dev_info(mm->dev, "------------------------E-------------------------------------\n");
	return;
}
static inline size_t get_cache_line_size(void)
{
#ifdef CONFIG_ARM64
	unsigned long cache_line_size;
	asm volatile("mrs %0, ctr_el0" : "=r" (cache_line_size));
	cache_line_size = 4 << ((cache_line_size >> 16) & 0xf);
    	return cache_line_size;
#elif defined(CONFIG_ARM)
	return L1_CACHE_BYTES;
#elif defined(CONFIG_X86_64)
	return boot_cpu_data.x86_cache_alignment;
#elif defined(CONFIG_RISCV)
	return L1_CACHE_BYTES;
#else
	return L1_CACHE_BYTES;
#endif
}

// For ARM64 ISA
#ifdef CONFIG_ARM64
static void arm64_cache_barrier(void)
{
	asm volatile("dsb sy" ::: "memory");
}

/* Clean+invalidate a range WITHOUT the trailing barrier. Callers that flush
 * many ranges back-to-back (e.g. per-page loops) should call this and then
 * issue arm64_cache_barrier() exactly once, instead of paying a full-system
 * dsb per range. */
static void arm64_flush_cache_range_nb(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;
	unsigned long cache_line_size = get_cache_line_size();
	unsigned long addr;

	start &= ~(cache_line_size - 1);
	end = (end + cache_line_size - 1) & ~(cache_line_size - 1);

	for (addr = start; addr < end; addr += cache_line_size) {
		asm volatile("dc civac, %0" : : "r" (addr) : "memory");
	}
}

static void arm64_invalidate_cache_range_nb(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;
	unsigned long cache_line_size = get_cache_line_size();
	unsigned long addr;

	start &= ~(cache_line_size - 1);
	end = (end + cache_line_size - 1) & ~(cache_line_size - 1);

	for (addr = start; addr < end; addr += cache_line_size) {
		asm volatile("dc ivac, %0" : : "r" (addr) : "memory");
	}
}

static void arm64_flush_cache_range(void *vaddr, size_t size)
{
	arm64_flush_cache_range_nb(vaddr, size);
	arm64_cache_barrier();
}

static void arm64_clean_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;
	unsigned long cache_line_size = get_cache_line_size();
	unsigned long addr;

	start &= ~(cache_line_size - 1);
	end = (end + cache_line_size - 1) & ~(cache_line_size - 1);

	for (addr = start; addr < end; addr += cache_line_size) {
		asm volatile("dc cvac, %0" : : "r" (addr) : "memory");
	}
	asm volatile("dsb sy" ::: "memory");
}

static void arm64_invalidate_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;
	unsigned long cache_line_size = get_cache_line_size();
	unsigned long addr;

	//pr_info("cache_line_size %ld\n", cache_line_size);
	start &= ~(cache_line_size - 1);
	end = (end + cache_line_size - 1) & ~(cache_line_size - 1);

	for (addr = start; addr < end; addr += cache_line_size) {
		asm volatile("dc ivac, %0" : : "r" (addr) : "memory");
	}
	asm volatile("dsb sy" ::: "memory");
}

// For ARM32 ISA
#elif defined(CONFIG_ARM)
static void arm_flush_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;

	__cpuc_flush_dcache_area(vaddr, size);
	outer_flush_range(__pa(start), __pa(end));
}

static void arm_clean_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;

	__cpuc_clean_dcache_area(vaddr, size);
	outer_clean_range(__pa(start), __pa(end));
}

static void arm_invalidate_cache_range(void *vaddr, size_t size)
{
	__cpuc_flush_dcache_area(vaddr, size);
	outer_inv_range(__pa(vaddr), __pa(vaddr) + size);
}

// For x86_64 ISA
#elif defined(CONFIG_X86_64)
static void x86_flush_cache_range(void *vaddr, size_t size)
{
	clflush_cache_range(vaddr, size);
}

static void x86_clean_cache_range(void *vaddr, size_t size)
{
	clflush_cache_range(vaddr, size);
}

static void x86_invalidate_cache_range(void *vaddr, size_t size)
{
	clflush_cache_range(vaddr, size);
}

// For RISC-V ISA
#elif defined(CONFIG_RISCV)
static void riscv_flush_cache_range(void *vaddr, size_t size)
{
	flush_icache_range((unsigned long)vaddr, (unsigned long)vaddr + size);
}

static void riscv_clean_cache_range(void *vaddr, size_t size)
{
	flush_icache_range((unsigned long)vaddr, (unsigned long)vaddr + size);
}

static void riscv_invalidate_cache_range(void *vaddr, size_t size)
{
	flush_icache_range((unsigned long)vaddr, (unsigned long)vaddr + size);
}
#else
	#error "Unsupported architecture."
#endif

/*
 * not __init: this runs from probe, and once the module-global
 * aipu clears on unbind, a re-bind reaches it again after .init.text is gone.
 */
// Initialize cache operation pointer
static int init_cache_ops(struct aipu_memory_manager *mm)
{
	struct __cache_ops *cache_ops = &mm->cache_ops;
	cache_ops->cache_line_size = get_cache_line_size();

#ifdef CONFIG_ARM64
	cache_ops->flush_range = arm64_flush_cache_range;
	cache_ops->clean_range = arm64_clean_cache_range;
	cache_ops->invalidate_range = arm64_invalidate_cache_range;
	cache_ops->flush_range_nb = arm64_flush_cache_range_nb;
	cache_ops->flush_barrier = arm64_cache_barrier;
	aipu_invalidate_range_nb = arm64_invalidate_cache_range_nb;
#elif defined(CONFIG_ARM)
	cache_ops->flush_range = arm_flush_cache_range;
	cache_ops->clean_range = arm_clean_cache_range;
	cache_ops->invalidate_range = arm_invalidate_cache_range;
	cache_ops->flush_range_nb = NULL;
	cache_ops->flush_barrier = NULL;
	aipu_invalidate_range_nb = NULL;
#elif defined(CONFIG_X86_64)
	cache_ops->flush_range = x86_flush_cache_range;
	cache_ops->clean_range = x86_clean_cache_range;
	cache_ops->invalidate_range = x86_invalidate_cache_range;
	cache_ops->flush_range_nb = NULL;
	cache_ops->flush_barrier = NULL;
	aipu_invalidate_range_nb = NULL;
#elif defined(CONFIG_RISCV)
	cache_ops->flush_range = riscv_flush_cache_range;
	cache_ops->clean_range = riscv_clean_cache_range;
	cache_ops->invalidate_range = riscv_invalidate_cache_range;
	cache_ops->flush_range_nb = NULL;
	cache_ops->flush_barrier = NULL;
	aipu_invalidate_range_nb = NULL;
#else
	#error "Unsupported architecture"
#endif

	return 0;
}
//For external api
void aipu_cache_flush(struct aipu_memory_manager *mm, void *vaddr, size_t size)
{
	if (!mm || !vaddr || !size)
		return;

	if(mm->cache_ops.flush_range)
		mm->cache_ops.flush_range(vaddr, size);
	return;
}

void aipu_cache_clean(struct aipu_memory_manager *mm, void *vaddr, size_t size)
{
	if (!mm || !vaddr || !size)
		return;

	if(mm->cache_ops.clean_range)
 		mm->cache_ops.clean_range(vaddr, size);
	return;
}

void aipu_cache_invalidate(struct aipu_memory_manager *mm, void *vaddr, size_t size)
{
	if (!mm || !vaddr || !size)
		return;

	if(mm->cache_ops.invalidate_range)
		mm->cache_ops.invalidate_range(vaddr, size);
	return;
}

static struct aipu_iova_buffer *aipu_get_iova_buffer(struct aipu_memory_manager *mm, u64 iova, char* str)
{
	//struct rb_node *node = mm->buffer_tree.rb_node;
	struct aipu_iova_buffer *buffer = NULL;

	if (!mm) {
		pr_err("Invalid parameter to get iova buffer %s.\n", str);
		return NULL;
	}

	//pr_info("%s iova 0x%llx +\n", __FUNCTION__, iova);
	mutex_lock(&mm->buffer_mutex);
	buffer = aipu_buffer_tree_find(mm, iova);
	if (buffer) {
		mutex_unlock(&mm->buffer_mutex);
		return buffer;
	}
	dev_err(mm->dev, "tid %d pid %d can't find iova 0x%llx %s\n",
		task_tgid_nr(current), task_pid_nr(current), iova, str);
	aipu_debug_print_buffers(mm);
	mutex_unlock(&mm->buffer_mutex);
	return NULL;
}

/*
 * aipu_get_block_buffer() lived here. It dropped buffer_mutex before walking
 * the buffer's phy_blocks, so both the buffer (kfree'd by
 * aipu_mm_v3_destroy_buffer_now) and the list itself (mutated under
 * phy_mutex by the same function) were unprotected across the walk. Its sole
 * caller was the dma-buf exporter, which now uses aipu_mm_v3_find_block_by
 * _iova -- the correct sibling, which holds buffer_mutex across the search
 * and nests phy_mutex for the inner walk, and returns the same thing.
 */

static struct aipu_iova_buffer *aipu_get_iova_buffer_by_exec_id(struct aipu_memory_manager *mm, u64 exec_id, char* str)
{
	struct aipu_iova_buffer *buffer = NULL;

	if (!mm) {
		pr_err("Invalid parameter to get iova buffer %s.\n", str);
		return NULL;
	}

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (buffer->exec_id == exec_id) {
			mutex_unlock(&mm->buffer_mutex);
			return buffer;
		}
	}
	dev_err(mm->dev, "tid %d pid %d can't find exec_id 0x%llx %s\n",
		 task_tgid_nr(current), task_pid_nr(current), exec_id, str);
	mutex_unlock(&mm->buffer_mutex);
	return NULL;
}

static int aipu_insert_buffer_safe(struct aipu_memory_manager *mm,
                                   struct aipu_iova_buffer *new_buffer)
{
	if (!mm || !new_buffer) {
		if (mm && mm->dev)
		dev_err(mm->dev, "Invalid parameters for buffer insertion\n");
		return -EINVAL;
	}

	mutex_lock(&mm->buffer_mutex);
	list_add_tail(&new_buffer->node, &mm->buffer_list);
	aipu_buffer_tree_insert(mm, new_buffer);
	mm->buffer_count++;
	mutex_unlock(&mm->buffer_mutex);
	//dev_info(mm->dev, "Successfully inserted buffer: IOVA=0x%llx\n",
	//	new_buffer->iova_start);

	return 0;
}

static struct aipu_iova_buffer* aipu_alloc_iova(struct aipu_memory_manager *mm,
						struct aipu_buf_request *buf_req,
			   			struct file *filp)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct aipu_iova_buffer* buffer = NULL;
	unsigned long pfn_hi;
	u64 reserved_iova;
	u64 iova_addr;
	u64 iova_size = 0;
	struct iova *iova;

	//Parameter check
	iova_size = ALIGN(buf_req->bytes, PAGE_SIZE);
	reserved_iova = ALIGN(buf_req->reserve_iova_size, PAGE_SIZE);
	pfn_hi = (mm->iova_base + mm->iova_size - 1) >> PAGE_SHIFT;

	if ((iova_size + reserved_iova) > mm->iova_size) {
		dev_err(mm->dev, "iova_size is larger than total size.\n");
		return NULL;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return NULL;

	iova = alloc_iova(iovad, (iova_size + reserved_iova) >> PAGE_SHIFT, pfn_hi, true);
	if (!iova) {
		mutex_lock(&mm->buffer_mutex);
		aipu_debug_print_buffers(mm);
		mutex_unlock(&mm->buffer_mutex);
		kfree(buffer);
		dev_err(mm->dev, "Failed to allocate IOVA\n");
		return NULL;
	}

	// init iova buffer
	iova_addr = iova->pfn_lo << PAGE_SHIFT;
	buffer->iova_start = iova_addr;
	buffer->iova_size = iova_size + reserved_iova;
	buffer->allocated_size = 0;
	buffer->region = buf_req->region;
	buffer->asid = buf_req->asid;
	buffer->data_type = buf_req->data_type;
	buffer->ref_count = 0;
	buffer->filp = filp;
	buffer->tgid = task_tgid_nr(current);
	buffer->pid = task_pid_nr(current);
	buffer->exec_id = buf_req->exec_id;
	INIT_LIST_HEAD(&buffer->phy_blocks);
	RB_CLEAR_NODE(&buffer->rb);
	mutex_init(&buffer->phy_mutex);

	if(aipu_insert_buffer_safe(mm, buffer)){
		dev_err(mm->dev, "Failed to insert IOVA buffer.\n");
		free_iova(iovad, iova->pfn_lo);
		kfree(buffer);
		return NULL;
	}

	buf_req->desc.pa = iova_addr;
	buf_req->desc.region = AIPU_BUF_REGION_DEFAULT;
	buf_req->desc.dev_offset = iova_addr;
	buf_req->desc.asid = buf_req->asid;
	buf_req->desc.bytes = (iova->pfn_hi - iova->pfn_lo + 1) << PAGE_SHIFT;
	buf_req->desc.exec_id = buf_req->exec_id;
	//dev_info(mm->dev, "Allocated IOVA: 0x%llx, size: 0x%llx\n", iova_addr, buf_req->desc.bytes);

	return buffer;
}

static void aipu_free_iova(struct aipu_memory_manager *mm, struct aipu_iova_buffer* buffer)
{
	unsigned long pfn_lo = buffer->iova_start >> PAGE_SHIFT;

	mutex_lock(&mm->buffer_mutex);
	if (buffer->ref_count) {
		dev_info(mm->dev, "Warning, buffer 0x%llx ref_count %d still is using buffer.\n", buffer->iova_start, buffer->ref_count);
	}

	list_del(&buffer->node);
	aipu_buffer_tree_erase(mm, buffer);
	mm->buffer_count--;
	free_iova(&mm->iova_domain, pfn_lo);
	kfree(buffer);
	mutex_unlock(&mm->buffer_mutex);
	//dev_info(mm->dev, "Freed IOVA: 0x%llx, size: 0x%llx ref_count %d\n", iova_addr, size, buffer->ref_count);
}

static void aipu_dma_free_pages(struct page **pages, int count)
{
	if (!pages)
		return;

	while (count--)
		__free_page(pages[count]);
	kvfree(pages);
}

static struct page **aipu_dma_alloc_pages(struct device *dev,
		unsigned int count, unsigned long order_mask, gfp_t gfp)
{
	struct page **pages;
	unsigned int i = 0, nid = dev_to_node(dev);

	order_mask &= (2U << MAX_PAGE_ORDER) - 1;
	if (!order_mask)
		return NULL;

	pages = kvzalloc(count * sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;

	/* IOMMU can map any pages, so himem can also be used here */
	gfp |= __GFP_NOWARN | GFP_KERNEL;

	/* It makes no sense to muck about with huge pages */
	gfp &= ~__GFP_COMP;

	while (count) {
		struct page *page = NULL;
		unsigned int order_size;

		/*
		 * Higher-order allocations are a convenience rather
		 * than a necessity, hence using __GFP_NORETRY until
		 * falling back to minimum-order allocations.
		 */
		for (order_mask &= (2U << __fls(count)) - 1;
		     order_mask; order_mask &= ~order_size) {
			unsigned int order = __fls(order_mask);
			gfp_t alloc_flags = gfp;

			order_size = 1U << order;
			if (order_mask > order_size)
				alloc_flags |= __GFP_NORETRY;

			page = alloc_pages_node(nid, alloc_flags, order);
			if (!page)
				continue;
			if (order)
				split_page(page, order);
			break;
		}
		if (!page) {
			aipu_dma_free_pages(pages, i);
			return NULL;
		}
		count -= order_size;
		while (order_size--)
			pages[i++] = page++;
	}

	return pages;
}

static int aipu_add_block_into_buffer(struct aipu_iova_buffer *buffer, struct aipu_phy_block *block)
{
	struct aipu_dma_buf_priv *priv;
	if (!buffer || !block)
		return -EINVAL;

	mutex_lock(&buffer->phy_mutex);
	buffer->ref_count++;
	block->iova_start = buffer->iova_start + buffer->allocated_size;
	buffer->allocated_size += block->size;
	if (block->dmabuf) {
		priv = (struct aipu_dma_buf_priv *)block->dmabuf->priv;
		priv->dev_pa = block->iova_start;
		priv->dma_pa = block->iova_start;
		priv->bytes = block->size;
	}
	list_add_tail(&block->list, &buffer->phy_blocks);
	mutex_unlock(&buffer->phy_mutex);

	return 0;
}

static struct aipu_phy_block *aipu_remove_block_node(struct aipu_iova_buffer *buffer, u64 pa)
{
	struct aipu_phy_block *block = NULL;
	struct aipu_phy_block *tmp;

	if (!buffer)
		return NULL;

	mutex_lock(&buffer->phy_mutex);
	list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
		if (block->iova_start == pa) {
			list_del(&block->list);
			buffer->ref_count--;
			buffer->allocated_size -= block->size;
			mutex_unlock(&buffer->phy_mutex);
			return block;
		}
	}
	mutex_unlock(&buffer->phy_mutex);

	return NULL;
}

static struct aipu_phy_block *aipu_alloc_phy_block(struct aipu_memory_manager *mm,
						   struct aipu_iova_buffer *buffer,
						   struct aipu_buf_request *buf_req)
{
	struct aipu_phy_block *block;
	u32 page_count;
	u64 mem_size = 0;
	u64 iova_addr = 0;
	size_t alloc_sizes = PAGE_SIZE;

	iova_addr = buffer->iova_start + buffer->allocated_size;
	mem_size = ALIGN(buf_req->bytes, PAGE_SIZE);
	page_count = mem_size >> PAGE_SHIFT;

	/* if (page_count < 32) //0-128K
		alloc_sizes = PAGE_SIZE; //4KB
	else //if (page_count < 4096) //128k-16MB
		alloc_sizes = PAGE_SIZE*16; //128KB
	else //if (page_count < 16384) //16MB-64MB
		alloc_sizes = PAGE_SIZE*256; //1MB
	else //above 64MB
		alloc_sizes = PAGE_SIZE*512; //2MB
	*/

	if ((buffer->allocated_size + buf_req->bytes) > buffer->iova_size) {
		dev_err(mm->dev, "iova space is not enough for phy memory allcation.");
		return NULL;
	}

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return NULL;

	// alloc physical pages
	block->pages = aipu_dma_alloc_pages(mm->dev, page_count, alloc_sizes>>PAGE_SHIFT, GFP_KERNEL | __GFP_ZERO);
	if (!block->pages) {
		dev_err(mm->dev, "Failed to allocate %u pages\n", page_count);
		kfree(block);
		return NULL;
	}

	//block->iova_start = iova_addr;
	block->size = mem_size;
	block->page_count = page_count;
	block->sgt.nents = block->page_count;
	block->dmabuf = NULL;
	INIT_LIST_HEAD(&block->list);
	block->bind_memory = false;

	//Add block into buffer's list
	aipu_add_block_into_buffer(buffer, block);

	//back map pa and size
	buf_req->desc.pa = iova_addr;
	buf_req->desc.dev_offset = iova_addr;
	buf_req->desc.bytes = mem_size;

	//dev_info(mm->dev, "Allocated phy block: iova=0x%llx size=0x%llx\n",
	//	iova_addr, mem_size);

	return block;
}

static void aipu_free_phy_block(struct aipu_memory_manager *mm, struct aipu_phy_block *block)
{
	if (block) {
		vunmap(block->va);
		if(!block->bind_memory)
			aipu_dma_free_pages(block->pages, block->page_count);
		kfree(block);
	}
}

static int aipu_map_sg_to_iommu(struct aipu_memory_manager *mm,
                                struct aipu_phy_block *block)
{
	int nents_mapped;

	if (!mm || !block)
		return -EINVAL;

	// build scatter-gather table
	if (sg_alloc_table_from_pages(&block->sgt, block->pages, block->page_count, 0, block->size, GFP_KERNEL)) {
		dev_err(mm->dev, "Failed to allocate sg table from pages.\n");
		return -EINVAL;
	}

	// map to iommu
	//mapped_size = iommu_map_sg(mm->iommu_domain, block->iova_start, block->sgt.sgl, block->sgt.nents, IOMMU_READ | IOMMU_WRITE);
	nents_mapped = dma_map_sg(mm->dev, block->sgt.sgl, block->sgt.nents, DMA_BIDIRECTIONAL);
	if (nents_mapped <=0 ) {
		dev_err(mm->dev, "IOMMU mapping failed: nents_mapped=%d\n", nents_mapped);
		sg_free_table(&block->sgt);
		return -ENOMEM;
	}

	block->iova_start = sg_dma_address(block->sgt.sgl);
	block->va = vmap(block->pages, block->page_count, VM_MAP, pgprot_noncached(PAGE_KERNEL));
	sg_free_table(&block->sgt);
	//dev_info(mm->dev, "Mapped 0x%llx bytes to IOVA 0x%llx\n",
	//	block->size, block->iova_start);
	return 0;
}

int aipu_alloc_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp)
{
	struct aipu_iova_buffer *buffer = NULL;
	struct aipu_phy_block *block = NULL;
	struct iommu_domain *domain;
	struct iova_domain *iovad;
	unsigned long flags;
	int ret = 0;

	if (!mm || !buf_req)
		return -EINVAL;

	/* V3 + IOMMU with custom IOVA domain */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		dev_dbg(mm->dev, "V3: alloc_dma_iova_phy mode=%d asid=%d bytes=0x%llx type=%d\n",
			buf_req->alloc_mode, buf_req->asid, buf_req->bytes, buf_req->data_type);
		/*
		 * REQ_BUF reaches this custom-IOVA allocator directly, bypassing
		 * aipu_mm_alloc's input validation. Re-assert the checks that
		 * matter on this path: a zero-length request otherwise reaches
		 * alloc_iova/iommu_map_sg with an inverted/empty range, and an
		 * out-of-range ASID indexes mm->asid_iova out of bounds.
		 */
		if (!buf_req->bytes || buf_req->asid >= ZHOUYI_ASID_COUNT) {
			dev_err(mm->dev, "V3: invalid alloc request: bytes 0x%llx asid %u\n",
				buf_req->bytes, buf_req->asid);
			return -EINVAL;
		}
		switch (buf_req->alloc_mode) {
		case AIPU_DMA_BUF_MALLOC_DEFAULT:
		case AIPU_DMA_BUF_MALLOC_IOVA_PHY:
			// 1. Allocate IOVA in ASID range
			buffer = aipu_mm_v3_alloc_iova(mm, buf_req, filp);
			if (!buffer) {
				ret = -ENOMEM;
				goto OUT;
			}
			
			// 2. Allocate physical pages and map
			ret = aipu_mm_v3_alloc_phy_and_map(mm, buffer, buf_req);
			if (ret) {
				/* alloc-failure path: buffer never became visible to
				 * the NPU, so destroy immediately (no stale-ref risk). */
				aipu_mm_v3_destroy_buffer_now(mm, buffer);
				goto OUT;
			}
			
			// 3. Convert IOVA to NPU view PA (asid_base + offset)
			buf_req->desc.pa = aipu_mm_v3_iova_to_pa(mm, buffer->iova_start, buf_req->asid);
			buf_req->desc.dev_offset = buf_req->desc.pa;
			buf_req->desc.exec_id = buf_req->exec_id;
			buf_req->desc.asid = buf_req->asid;
			break;
			
		case AIPU_DMA_BUF_MALLOC_IOVA:
			// IOVA only
			buffer = aipu_mm_v3_alloc_iova(mm, buf_req, filp);
			if (!buffer) {
				ret = -ENOMEM;
				goto OUT;
			}
			// Return NPU view PA
			buf_req->desc.pa = aipu_mm_v3_iova_to_pa(mm, buffer->iova_start, buf_req->asid);
			buf_req->desc.dev_offset = buf_req->desc.pa;
			buf_req->desc.asid = buf_req->asid;
			break;
			
		case AIPU_DMA_BUF_MALLOC_PHY:
			// Find buffer by exec_id and allocate physical
			buffer = aipu_get_iova_buffer_by_exec_id(mm, buf_req->exec_id, "v3 malloc phy");
			if (!buffer) {
				ret = -EINVAL;
				goto OUT;
			}
			ret = aipu_mm_v3_alloc_phy_and_map(mm, buffer, buf_req);
			if (ret)
				goto OUT;
			buf_req->desc.pa = aipu_mm_v3_iova_to_pa(mm, 
				buffer->iova_start + buffer->allocated_size - buf_req->desc.bytes,
				buf_req->asid);
			buf_req->desc.asid = buf_req->asid;
			break;
			
		default:
			ret = -EINVAL;
			goto OUT;
		}

		/* Register TCB buffer for TCB-type allocations so hold_tcb can find it */
		if (buf_req->data_type == AIPU_MM_DATA_TYPE_TCB && mm->v3_tcb_buf_head) {
			struct aipu_tcb_buf *tbuf;

			tbuf = kmem_cache_zalloc(mm->tbuf_cache, GFP_KERNEL);
			if (tbuf) {
				INIT_LIST_HEAD(&tbuf->node);
				RB_CLEAR_NODE(&tbuf->rb);
				tbuf->reg = NULL;
				tbuf->head = buf_req->desc.pa;
				tbuf->tail = buf_req->desc.pa + buf_req->desc.bytes -
					     sizeof(struct aipu_tcb);
				tbuf->tail_tcb = NULL;
				tbuf->dep_job_id = 0;
				tbuf->pinned = false;

				spin_lock_irqsave(&mm->slock, flags);
				list_add_tail(&tbuf->node, &mm->v3_tcb_buf_head->node);
				aipu_tcb_tree_insert(mm, tbuf);
				spin_unlock_irqrestore(&mm->slock, flags);
			}
		}

		return 0;
	}

	/* V3.2 path (original) */
	if (buf_req->alloc_mode != AIPU_DMA_BUF_MALLOC_DEFAULT) {
		domain = mm->iommu_domain;
		iovad = &mm->iova_domain;
	}

	switch (buf_req->alloc_mode) {
	case AIPU_DMA_BUF_MALLOC_DEFAULT:
		//support standard dma api and reserved memory
		ret = aipu_mm_alloc(mm, buf_req, filp);
		break;

	case AIPU_DMA_BUF_MALLOC_IOVA:
		// alloc iova adress only
		buffer = aipu_alloc_iova(mm, buf_req, filp);
		if (!buffer) {
			dev_err(mm->dev, "Malloc iova FAILED size 0x%llx\n", buf_req->bytes + buf_req->reserve_iova_size);
			ret = -ENOMEM;
			goto OUT;
		}
		break;

	case AIPU_DMA_BUF_MALLOC_PHY:
		// alloc phy memory and map allocated iova address
		buffer = aipu_get_iova_buffer_by_exec_id(mm, buf_req->exec_id, "malloc phy");
		if (!buffer) {
			dev_err(mm->dev, "Can't find pa 0x%llx in iova domain.\n", buf_req->exec_id);
			ret = -EINVAL;
			goto OUT;
		}

		block = aipu_alloc_phy_block(mm, buffer, buf_req);
		if (!block) {
			dev_err(mm->dev, "OOM iova 0x%llx phy size 0x%llx FAILED.\n", buf_req->exec_id, buf_req->bytes);
			ret = -ENOMEM;
			goto OUT;
		}

		//map sg table to iommu
		ret = aipu_map_sg_to_iommu(mm, block);
		if (ret) {
			dev_err(mm->dev, "Map block iova 0x%llx size 0x%llx FAILED.\n", block->iova_start, block->size);
			aipu_free_phy_block(mm, block);
			goto OUT;
		}
		/* the LINEAR alias: block->va is pgprot_noncached,
		 * so this was a no-op exactly as the BUF_CACHE ioctls were.
		 */
		aipu_mm_v3_block_invalidate(mm, block, 0, block->size);
		break;

	case AIPU_DMA_BUF_MALLOC_IOVA_PHY:
		buffer = aipu_alloc_iova(mm, buf_req, filp);
		if (!buffer) {
			dev_err(mm->dev, "Malloc iova size 0x%llx FAILED.\n",  buf_req->bytes + buf_req->reserve_iova_size);
			ret = -ENOMEM;
			goto OUT;
		}

		block = aipu_alloc_phy_block(mm, buffer, buf_req);
		if (!block) {
			dev_err(mm->dev, "OOM iova 0x%llx phy size 0x%llx FAILD.\n", buffer->iova_start, buf_req->bytes);
			ret = -ENOMEM;
			goto err_free_iova;
		}

		//map iova and phy memory
		ret = aipu_map_sg_to_iommu(mm, block);
		if (ret) {
			dev_err(mm->dev, "Map block iova 0x%llx size 0x%llx FAILED.\n", block->iova_start, block->size);
			goto err_free_phy;
		}
		/* the LINEAR alias, as above. */
		aipu_mm_v3_block_invalidate(mm, block, 0, block->size);
		break;

	default:
		ret = -EINVAL;
		goto OUT;
	}

	return 0;

err_free_phy:
	aipu_free_phy_block(mm, block);

err_free_iova:
	aipu_free_iova(mm, buffer);
OUT:
	return ret;
}

/**
 * aipu_mm_v3_find_buffer_by_iova - Find IOVA buffer by IOVA
 * @mm: memory manager
 * @iova: IOVA address
 * 
 * Return: buffer pointer or NULL
 */
static struct aipu_iova_buffer *aipu_mm_v3_find_buffer_by_iova(struct aipu_memory_manager *mm,
							      u64 iova)
{
	struct aipu_iova_buffer *buffer;

	mutex_lock(&mm->buffer_mutex);
	buffer = aipu_buffer_tree_find(mm, iova);
	mutex_unlock(&mm->buffer_mutex);

	return buffer;
}

/**
 * aipu_mm_v3_find_owned_buffer_by_iova - Find IOVA buffer, but only for its owner
 * @mm: memory manager
 * @iova: IOVA address
 * @filp: the fd the caller is acting on behalf of
 *
 * The buffer_list is a single global list shared by every fd on the 0666 node,
 * so resolving an address to a buffer proves nothing about who owns it. This
 * variant returns the buffer only when it belongs to @filp, and is the gate for
 * every user-facing operation that takes a caller-supplied address (free, mmap,
 * cache flush/invalidate).
 *
 * @filp == NULL identifies the kernel itself as the owner and matches only
 * buffers allocated with a NULL owner: the cross-fd hold-TCB pool and the
 * dma-buf exporter's backing allocation, both of which outlive any single fd
 * by construction and are reclaimed by their own paths. That regime is not
 * reachable from userspace -- every ioctl and mmap entry point passes the
 * caller's struct file, which the VFS guarantees is non-NULL -- so a user
 * still cannot resolve a kernel-owned buffer.
 * An earlier version expressed this as "reject NULL owners" and so refused the three
 * in-kernel callers that use NULL as their identity: FREE_DMA_BUF, mmap of an
 * exported dma-buf, and hold-TCB pool reclaim at deinit.
 *
 * Return: owned buffer pointer, or NULL when no buffer spans the IOVA or the
 * spanning buffer belongs to a different owner.
 */
static struct aipu_iova_buffer *aipu_mm_v3_find_owned_buffer_by_iova(struct aipu_memory_manager *mm,
								     u64 iova, struct file *filp)
{
	struct aipu_iova_buffer *buffer;

	mutex_lock(&mm->buffer_mutex);
	buffer = aipu_buffer_tree_find(mm, iova);
	/* address resolves, but not to a buffer this owner may touch */
	if (buffer && buffer->filp != filp)
		buffer = NULL;
	mutex_unlock(&mm->buffer_mutex);

	return buffer;
}

/**
 * aipu_mm_v3_pa_owned_by - does an NPU-view PA fall inside a buffer owned by @filp?
 * @mm: memory manager
 * @pa: NPU-view physical address (asid_base + offset)
 * @filp: the fd the caller is acting on behalf of
 *
 * The submit path uses this to reject job descriptors whose TCB addresses point
 * outside the caller's own buffers before those addresses reach the TSM dispatch
 * registers. Runs in process context only: it takes buffer_mutex,
 * so it must not be called from the completion/dispatch path under manager->lock.
 *
 * Only the V3 custom-IOVA path is validated; on any other configuration this
 * returns true (no per-fd IOVA buffers to check against).
 *
 * Return: true if @pa is owned by @filp (or validation does not apply), false
 * if @pa resolves to no owned buffer.
 */
bool aipu_mm_v3_pa_owned_by(struct aipu_memory_manager *mm, u64 pa, struct file *filp)
{
	u64 iova;
	u32 asid;

	if (!mm || !filp)
		return false;

	if (!(mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	      mm->has_iommu && mm->use_v3_custom_iova))
		return true;

	for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
		iova = aipu_mm_v3_pa_to_iova(mm, pa, asid);
		if (!iova)
			continue;
		if (aipu_mm_v3_find_owned_buffer_by_iova(mm, iova, filp))
			return true;
	}

	return false;
}

/**
 * aipu_mm_v3_find_tcb_buf_no_lock - Find TCB buffer by NPU view PA in V3 custom IOVA list
 * @mm: memory manager
 * @pa: NPU view physical address
 *
 * Must be called with mm->slock held.
 * Return: tcb_buf pointer or NULL
 */
static struct aipu_tcb_buf *aipu_mm_v3_find_tcb_buf_no_lock(struct aipu_memory_manager *mm,
							    u64 pa)
{
	struct rb_node *n = mm->v3_tcb_tree.rb_node;
	struct aipu_tcb_buf *curr, *floor = NULL;

	if (!mm->v3_tcb_buf_head)
		return NULL;

	/* the descriptor with the greatest head <= pa; its tail is live state (hold_tcb_buf_alloc moves it) */
	while (n) {
		curr = rb_entry(n, struct aipu_tcb_buf, rb);
		if (pa < curr->head) {
			n = n->rb_left;
		} else {
			floor = curr;
			n = n->rb_right;
		}
	}
	if (floor && pa <= floor->tail)
		return floor;

	return NULL;
}

int aipu_free_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_desc *desc, struct file *filp)
{
	u64 exec_id = desc->exec_id;
	struct aipu_iova_buffer* buffer = NULL;
	struct aipu_phy_block *block, *tmp;
	bool free_iova = true;
	u64 iova;
	u32 asid;

	if (!desc || !mm)
		return -EINVAL;

	/* V3 custom IOVA path */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		/*
		 * Work out the ASID window instead of asking the caller which
		 * one to look in. Nothing in the frozen UAPI obliges a free to
		 * populate desc->asid -- every aipu_buf_desc field is marked
		 * [kmd] -- and the vendor userspace does not: its free path
		 * writes pa, bytes and exec_id only, leaving asid as whatever
		 * was on the heap. One observed run left 166 there and every
		 * buffer it freed took -EINVAL.
		 *
		 * The sweep is the shape aipu_mm_mmap_buf, aipu_mm_cache_flush
		 * and aipu_mm_cache_invalid already use, and it is safe for the
		 * same reason they are: a PA that happens to translate in more
		 * than one window still has to resolve to a buffer THIS fd owns
		 * before anything is freed. Without that check sweeping
		 * would widen the set of buffers a PA can name, so this fix
		 * depends on that ownership check, which the vendor driver lacks.
		 */
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, desc->pa, asid);
			if (!iova)
				continue;

			buffer = aipu_mm_v3_find_owned_buffer_by_iova(mm, iova, filp);
			if (buffer)
				break;
		}
		if (!buffer) {
			dev_err(mm->dev, "V3: no owned buffer for PA 0x%llx in any ASID window\n",
				desc->pa);
			return -EINVAL;
		}

		/* Remove associated TCB buffer descriptor if any */
		{
			struct aipu_tcb_buf *tbuf;
			unsigned long tflags;

			spin_lock_irqsave(&mm->slock, tflags);
			tbuf = aipu_mm_v3_find_tcb_buf_no_lock(mm, desc->pa);
			if (tbuf)
				aipu_tcb_buf_remove_no_lock(mm, tbuf);
			spin_unlock_irqrestore(&mm->slock, tflags);

		}

		aipu_mm_v3_free_iova_buffer(mm, buffer);
		return 0;
	}

	/* Original V3.2 path */
	if (!desc->exec_id)
		return aipu_mm_free(mm, desc, filp, true);

	if (desc->exec_id >= AIPU_EXEC_ID)
		buffer = aipu_get_iova_buffer_by_exec_id(mm, exec_id, "free dma iova phy");

	if(!buffer) {
		buffer = aipu_get_iova_buffer(mm, desc->pa, "free iova phy");
		if(!buffer) {
			dev_err(mm->dev, "can't find iova buffer by iova 0x%llx exec id 0x%llx\n", desc->pa, exec_id);
			return -EINVAL;
		}
	}

	//rebind/bind dma buffer free one by one in iova buffer
	if (desc->exec_id == DMA_BUF_EXEC_ID){
		list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
			if (block->iova_start == desc->pa) {
				list_del(&block->list);
				buffer->ref_count--;
				buffer->allocated_size -= block->size;
				iommu_unmap(mm->iommu_domain, block->iova_start, block->size);
				if (block->bind_memory)
					free_iova = false;
				aipu_free_phy_block(mm, block);
				break;
			}
		}
	//free all for job buffer once
	} else {
		list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
			if (!block->bind_memory) {
				list_del(&block->list);
				buffer->ref_count--;
				buffer->allocated_size -= block->size;
				iommu_unmap(mm->iommu_domain, block->iova_start, block->size);
				aipu_free_phy_block(mm, block);
			} else {
				free_iova = false;
			}
		}
	}

	if (free_iova || !buffer->ref_count)
		aipu_free_iova(mm, buffer);

	return 0;
}

int aipu_rebind_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_rebind_buf_desc *desc, struct file *filp)
{
	u64 exec_id = desc->exec_id;
	u64 pa_unmap = desc->pa_unmap;
	struct aipu_iova_buffer* buffer;
	struct aipu_phy_block *block;
	int ret = 0;
	size_t mapped_size;

	if (!mm || !desc)
		return -EINVAL;

	/*
	 * REBIND_DMA_BUF is V3.2-shaped logic with no V3-custom branch, and on
	 * custom-IOVA hardware it cannot do anything useful: it unmaps from the
	 * driver's own domain (iommu_unmap on mm->iommu_domain) but re-maps
	 * through aipu_map_sg_to_iommu, i.e. the DMA API, whose IOVAs come from
	 * the kernel's default DMA-IOMMU cookie and fall outside every ASID
	 * window -- the exact reason the import path was routed through
	 * aipu_mm_v3_map_imported_sgt instead. A "successful" rebind therefore
	 * returns an address the NPU cannot translate, leaks the new mapping
	 * (nothing ever calls dma_unmap_sg, and the scatterlist an unmap would
	 * need is freed immediately after mapping), and on a short unmap leaks
	 * the block itself, which has already been unlinked from its buffer.
	 *
	 * Refuse the command rather than write a custom-domain rebind that does
	 * not exist: the treatment BIND_DMA_BUF already has.
	 */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		dev_err_once(mm->dev,
			     "REBIND_DMA_BUF is not supported on V3 custom-IOVA hardware\n");
		return -EINVAL;
	}

	//handle before iova buffer;
	buffer = aipu_get_iova_buffer(mm, pa_unmap, "rebind dma buffer");
	if (!buffer) {
		dev_err(mm->dev, "Cant find iova buffer 0x%llx.\n", desc->pa_unmap);
		return -EINVAL;
	}

	/* refuse to re-home a buffer this fd does not own */
	if (!buffer->filp || buffer->filp != filp) {
		dev_err(mm->dev, "rebind: buffer 0x%llx not owned by caller\n", desc->pa_unmap);
		return -EINVAL;
	}

	block = aipu_remove_block_node(buffer, pa_unmap);
	if (!block) {
		dev_err(mm->dev, "Can't unbind block.\n");
		return -ENXIO;
	}

	mapped_size = iommu_unmap(mm->iommu_domain, block->iova_start, block->size);
	 if (mapped_size != block->size) {
	 	dev_err(mm->dev, "Can't unmap block 0x%llx ret = .\n", block->iova_start);
		return -ENXIO;
	}

	//iova buffer free
	if (!buffer->ref_count)
		aipu_free_iova(mm, buffer);

	//rebind new iova buffer
	buffer = aipu_get_iova_buffer_by_exec_id(mm, exec_id, "rebind dma buf");
	if (!buffer)
		return -ENXIO;

	aipu_add_block_into_buffer(buffer, block);
	block->bind_memory = true;
	ret = aipu_map_sg_to_iommu(mm, block);
	if (ret) {
		dev_err(mm->dev, "Map block to new iova 0x%llx size 0x%llx FAILED.\n", block->iova_start, block->size);
		aipu_free_phy_block(mm, block);
		goto OUT;
	}

	desc->pa = block->iova_start;
	/* the LINEAR alias, as in aipu_alloc_dma_buf. */
	aipu_mm_v3_block_invalidate(mm, block, 0, block->size);

OUT:
	return ret;
}

/* ============================================================================
 * V3 Custom IOVA Domain Support
 * 
 * Enable per-ASID 3GB IOVA ranges for V3 with IOMMU:
 * - ASID 0: IOVA [0, 3GB)      -> NPU view: asid_base + offset
 * - ASID 1: IOVA [3GB, 6GB)    -> NPU view: asid_base + offset
 * - ASID 2: IOVA [6GB, 9GB)    -> NPU view: asid_base + offset
 * - ASID 3: IOVA [9GB, 12GB)   -> NPU view: asid_base + offset
 * ============================================================================ */

/**
 * aipu_mm_v3_init_iova_domain - Initialize IOVA domain for V3 per-ASID support
 * @mm: memory manager
 * 
 * Return: 0 on success, error code otherwise
 */
int aipu_mm_v3_init_iova_domain(struct aipu_memory_manager *mm)
{
	int asid;
	u64 total_iova_size;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_V3)
		return -EINVAL;

	/*
	 * Per-ASID layout: 4GB slot, of which 3GB is usable (cacheable region)
	 * and the top 1GB is reserved as never-map (non-cacheable/core/cluster).
	 * The 1GB never-map zone also serves as a barrier preventing alloc_iova
	 * (which has no lower-bound parameter) from leaking into adjacent ASIDs.
	 *
	 * ASID 0: IOVA [0,    3GB) usable + [3GB,  4GB) never-map
	 * ASID 1: IOVA [4GB,  7GB) usable + [7GB,  8GB) never-map
	 * ASID 2: IOVA [8GB, 11GB) usable + [11GB,12GB) never-map
	 * ASID 3: IOVA [12GB,15GB) usable + [15GB,16GB) never-map
	 */
	#define V3_ASID_SLOT_SIZE	0x100000000ULL	/* 4GB per ASID slot */
	#define V3_ASID_USABLE_SIZE	0xC0000000ULL	/* 3GB usable per ASID */

	/* Initialize per-ASID IOVA info */
	for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
		mm->asid_iova[asid].iova_base = (u64)asid * V3_ASID_SLOT_SIZE;
		mm->asid_iova[asid].iova_size = V3_ASID_USABLE_SIZE;
		mm->asid_iova[asid].iova_used = 0;
		/* NPU view base: 0, 4GB, 8GB, 12GB for ASID 0-3 */
		mm->asid_iova[asid].asid_base = (u64)asid * V3_ASID_SLOT_SIZE;

		dev_dbg(mm->dev, "V3 ASID %d: IOVA [0x%llx, 0x%llx), NPU base 0x%llx\n",
			asid, mm->asid_iova[asid].iova_base,
			mm->asid_iova[asid].iova_base + V3_ASID_USABLE_SIZE,
			mm->asid_iova[asid].asid_base);
	}

	/* Total IOVA space: 16GB for 4 ASIDs (4GB slots) */
	mm->iova_base = 0;
	mm->iova_size = (u64)ZHOUYI_ASID_COUNT * V3_ASID_SLOT_SIZE;
	total_iova_size = mm->iova_size;

	/* Initialize IOVA domain */
	init_iova_domain(&mm->iova_domain, PAGE_SIZE, 0);

	/*
	 * Reserve page 0 so IOVA 0x0 is never allocated.
	 * PA 0x0 (ASID 0 base + offset 0) would be indistinguishable from the
	 * error sentinel used by aipu_mm_v3_pa_to_iova() and callers.
	 */
	if (!reserve_iova(&mm->iova_domain, 0, 0))
		dev_warn(mm->dev, "V3: failed to reserve IOVA page 0\n");

	/* Reserve "never map" zones at each 3GB boundary to prevent wrap-around */
	/* These are the top 1GB of each 4GB window: [3GB,4GB), [7GB,8GB), etc. */
	if (aipu_mm_v3_reserve_never_map_zones(mm))
		dev_warn(mm->dev, "V3: failed to reserve some never-map zones\n");

	/* Initialize buffer management */
	mm->buffer_count = 0;
	mutex_init(&mm->buffer_mutex);
	spin_lock_init(&mm->asid_lock);
	INIT_LIST_HEAD(&mm->buffer_list);
	mm->buffer_tree = RB_ROOT;
	mm->v3_tcb_tree = RB_ROOT;
	/* 16GB total IOVA needs 34-bit DMA mask */
	mm->dma_mask = 34;
	init_cache_ops(mm);

	/* Initialize V3 TCB buffer tracking list */
	mm->v3_tcb_buf_head = kmem_cache_zalloc(mm->tbuf_cache, GFP_KERNEL);
	if (!mm->v3_tcb_buf_head)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->v3_tcb_buf_head->node);
	mm->v3_tcb_buf_head->reg = NULL;

	dev_info(mm->dev, "V3 IOVA domain: total 0x%llx (%d ASIDs x 4GB slots, 3GB usable)\n",
		 total_iova_size, ZHOUYI_ASID_COUNT);

	return 0;
}

/**
 * aipu_mm_v3_reserve_never_map_zones - Reserve zones that should never be mapped
 * @mm: memory manager
 *
 * Each ASID has a 4GB IOVA slot.  The top 1GB [3GB, 4GB) of each slot
 * corresponds to the NPU's non-cacheable / core / cluster address range
 * and must never be used for DMA.  Reserving it also acts as a barrier
 * that prevents alloc_iova (top-down, no lower-bound) from leaking into
 * the adjacent ASID's usable range.
 */
static int aipu_mm_v3_reserve_never_map_zones(struct aipu_memory_manager *mm)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct iova *iova;
	u64 never_map_start, never_map_end;
	int asid;
	int ret = 0;

	for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
		/* Top 1GB of each 4GB slot: [asid*4GB + 3GB, asid*4GB + 4GB) */
		never_map_start = (u64)asid * V3_ASID_SLOT_SIZE + V3_ASID_USABLE_SIZE;
		never_map_end   = (u64)(asid + 1) * V3_ASID_SLOT_SIZE;

		iova = reserve_iova(iovad,
				    never_map_start >> PAGE_SHIFT,
				    (never_map_end - 1) >> PAGE_SHIFT);
		if (!iova) {
			dev_warn(mm->dev, "V3: failed to reserve never-map zone at 0x%llx\n",
				 never_map_start);
			ret = -ENOMEM;
		} else {
			dev_dbg(mm->dev, "V3: ASID %d never-map [0x%llx, 0x%llx)\n",
				asid, never_map_start, never_map_end);
		}
	}

	return ret;
}

/**
 * aipu_mm_v3_deinit_iova_domain - Cleanup V3 IOVA domain
 * @mm: memory manager
 */
void aipu_mm_v3_deinit_iova_domain(struct aipu_memory_manager *mm)
{
	struct aipu_tcb_buf *tbuf, *tmp;
	unsigned long flags;

	if (!mm || !mm->use_v3_custom_iova)
		return;

	/* Free V3 TCB buffer list */
	if (mm->v3_tcb_buf_head) {
		spin_lock_irqsave(&mm->slock, flags);
		list_for_each_entry_safe(tbuf, tmp, &mm->v3_tcb_buf_head->node, node) {
			list_del(&tbuf->node);
			kmem_cache_free(mm->tbuf_cache, tbuf);
		}
		mm->v3_tcb_tree = RB_ROOT;
		spin_unlock_irqrestore(&mm->slock, flags);
		kmem_cache_free(mm->tbuf_cache, mm->v3_tcb_buf_head);
		mm->v3_tcb_buf_head = NULL;
	}

	put_iova_domain(&mm->iova_domain);
	mm->use_v3_custom_iova = false;
}

/*
 * aipu_mm_v3_dump_asid_state - dump per-ASID accounting summary.
 *
 * Diagnostic only, called when the IOVA-counter check rejects an
 * allocation. Walks mm->buffer_list and compares the live in-list
 * buffers belonging to @asid (count + summed iova_size) against
 * asid_iova[asid].iova_used and mm->buffer_count. If counter > sum,
 * the leak is on the counter side (a free path decremented the list
 * but not the counter, or never ran at all). If counter ~= sum, the
 * buffers are genuinely still on the list because some fd never
 * closed (or its release path skipped cleanup).
 *
 * Tgid breakdown (up to 16 distinct PIDs) is printed so it is obvious
 * which user-space process owns the surviving buffers.
 */
static void __maybe_unused
aipu_mm_v3_dump_asid_state(struct aipu_memory_manager *mm, u32 asid)
{
	struct aipu_iova_buffer *buffer;
	struct { int tgid; int count; u64 total; } tg[16];
	int n_tg = 0, i;
	int list_total = 0, asid_count = 0;
	u64 asid_total = 0;

	if (!mm || !mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return;

	memset(tg, 0, sizeof(tg));

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		list_total++;
		if (buffer->asid != asid)
			continue;
		asid_count++;
		asid_total += buffer->iova_size;

		for (i = 0; i < n_tg; i++)
			if (tg[i].tgid == buffer->tgid)
				break;
		if (i == n_tg && n_tg < (int)ARRAY_SIZE(tg)) {
			tg[n_tg].tgid = buffer->tgid;
			n_tg++;
		}
		if (i < (int)ARRAY_SIZE(tg)) {
			tg[i].count++;
			tg[i].total += buffer->iova_size;
		}
	}
	mutex_unlock(&mm->buffer_mutex);

	dev_info(mm->dev,
		 "V3 ASID %u accounting dump: counter=0x%llx, list=%d buffers on this ASID totalling 0x%llx (mm->buffer_count=%d, list_walk=%d)\n",
		 asid, mm->asid_iova[asid].iova_used,
		 asid_count, asid_total, mm->buffer_count, list_total);

	for (i = 0; i < n_tg; i++)
		dev_info(mm->dev,
			 "  tgid=%d: %d buffers, total 0x%llx\n",
			 tg[i].tgid, tg[i].count, tg[i].total);
}

/**
 * aipu_mm_v3_alloc_iova - Allocate IOVA in specific ASID range
 * @mm: memory manager
 * @buf_req: buffer request (asid, bytes, align_in_page)
 * @filp: file pointer
 * 
 * Return: allocated buffer or NULL on failure
 */
/*
 * aipu_mm_v3_iova_search_from_top - start the next alloc_iova() at the domain's top
 *
 * alloc_iova() searches downward from the domain's cached last 64-bit insertion,
 * which is device-wide. After anything was placed in ASID 1 that node sits inside
 * ASID 1, so a later request for ASID 2 or 3 is placed below it -- in ASID 1's
 * range -- and refused by the range check ("IOVA ... outside ASID n range"), and
 * the higher ASIDs stay unusable until the driver is reloaded. Resetting the
 * cache to the anchor (what the allocator itself does when the cached node is
 * freed) makes the search start at the top; the never-map zones bound each ASID
 * slot from below, so the request lands in its own slot. ASID 0's limit is under
 * 4 GiB and takes the 32-bit cache, which this leaves alone.
 */
static void aipu_mm_v3_iova_search_from_top(struct iova_domain *iovad, u32 asid)
{
	unsigned long flags;

	if (!asid)
		return;
	spin_lock_irqsave(&iovad->iova_rbtree_lock, flags);
	iovad->cached_node = &iovad->anchor.node;
	spin_unlock_irqrestore(&iovad->iova_rbtree_lock, flags);
}

struct aipu_iova_buffer *aipu_mm_v3_alloc_iova(struct aipu_memory_manager *mm,
						struct aipu_buf_request *buf_req,
						struct file *filp)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct aipu_iova_buffer *buffer = NULL;
	struct aipu_asid_iova_info *asid_info;
	struct iova *iova;
	u64 asid_iova_base, asid_iova_size;
	u64 iova_size;
	unsigned long pfn_lo, pfn_hi;
	u32 asid = buf_req->asid;

	if (asid >= ZHOUYI_ASID_COUNT) {
		dev_err(mm->dev, "V3: invalid ASID %d\n", asid);
		return NULL;
	}

	asid_info = &mm->asid_iova[asid];
	asid_iova_base = asid_info->iova_base;
	asid_iova_size = asid_info->iova_size;

	/*
	 * Atomically check and reserve the accounting quota. iova_used is a
	 * process-wide counter shared by every fd that maps this ASID; the
	 * check-then-increment must be a single critical section, otherwise
	 * concurrent allocers/freers race on a non-atomic read-modify-write,
	 * lost updates accumulate, and the counter drifts upward until this
	 * gate rejects allocations even though the IOVA domain still has room
	 * ("V3: ASID N out of IOVA space" under multi-process stress).
	 *
	 * Reserve up front, then roll back on any failure below. The actual
	 * (sleeping) alloc_iova/kzalloc run outside the spinlock.
	 */
	iova_size = ALIGN(buf_req->bytes, PAGE_SIZE);
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used + iova_size > asid_iova_size) {
		spin_unlock(&mm->asid_lock);
		dev_err(mm->dev, "V3: ASID %d out of IOVA space (used 0x%llx, need 0x%llx)\n",
			asid, asid_info->iova_used, iova_size);
		/*
		 * Diagnostic dump kept available but disabled by default.
		 * Re-enable if "out of IOVA space" reappears, to tell a
		 * counter-accounting drift apart from a real per-fd leak:
		 *   aipu_mm_v3_dump_asid_state(mm, asid);
		 */
		return NULL;
	}
	asid_info->iova_used += iova_size;
	spin_unlock(&mm->asid_lock);

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		goto err_unreserve;

	/*
	 * Calculate pfn range for this ASID's usable region.
	 * The never-map zone at [asid_base + 3GB, asid_base + 4GB) acts as
	 * a hard barrier so alloc_iova cannot leak into the next ASID.
	 */
	pfn_lo = asid_iova_base >> PAGE_SHIFT;
	pfn_hi = ((asid_iova_base + asid_iova_size) >> PAGE_SHIFT) - 1;
	
	dev_dbg(mm->dev, "V3: alloc_iova ASID %d, pfn_lo=0x%lx, pfn_hi=0x%lx, size=0x%llx\n",
		asid, pfn_lo, pfn_hi, (u64)(iova_size >> PAGE_SHIFT));

	/*
	 * Allocate IOVA from domain.
	 * size_aligned=false: large NPU buffers only need PAGE_SIZE alignment.
	 * With size_aligned=true, the start address would be rounded to the next
	 * power-of-2 >= size, wasting huge amounts of IOVA space (e.g. a 1.5GB
	 * alloc would require a 2GB-aligned start, failing in a 3GB range).
	 */
	aipu_mm_v3_iova_search_from_top(iovad, asid);
	iova = alloc_iova(iovad, iova_size >> PAGE_SHIFT, pfn_hi, false);
	if (!iova) {
		dev_err(mm->dev, "V3: alloc_iova failed for ASID %d, size 0x%llx\n",
			asid, iova_size);
		goto err_free_buffer;
	}

	/* Verify IOVA is within ASID range (pfn_hi is now inclusive) */
	if (iova->pfn_lo < pfn_lo || iova->pfn_hi > pfn_hi) {
		dev_err(mm->dev, "V3: IOVA 0x%lx-0x%lx outside ASID %d range [0x%lx, 0x%lx]\n",
			iova->pfn_lo, iova->pfn_hi, asid, pfn_lo, pfn_hi);
		__free_iova(iovad, iova);
		goto err_free_buffer;
	}

	/* Initialize buffer structure */
	buffer->iova_start = (u64)iova->pfn_lo << PAGE_SHIFT;
	buffer->iova_size = iova_size;
	buffer->allocated_size = 0;
	buffer->region = buf_req->region;
	buffer->asid = asid;
	buffer->data_type = buf_req->data_type;
	buffer->ref_count = 1;
	/*
	 * filp==NULL is reserved for the hold-TCB allocation path
	 * (aipu_mm_hold_tcb_buf_alloc -> aipu_mm_alloc(..., NULL) with
	 * data_type == AIPU_MM_DATA_TYPE_TCB). Such a buffer is intentionally
	 * cross-fd: the exit-encode tail is shared by every job's TCB chain,
	 * so it must outlive any single fd. It is reclaimed at module deinit
	 * by aipu_mm_hold_tcb_buf_free, NOT by aipu_mm_v3_free_buffers_by_filp
	 * (which can never match a NULL filp).
	 *
	 * The dma-buf exporter (aipu_alloc_dma_buf) is the second such caller,
	 * and equally deliberate: an exported buffer is owned by the dma_buf's
	 * refcount, not by the fd that asked for it, because the fd may be
	 * passed to another process or closed while importers still hold it.
	 * It reclaims in aipu_dma_release and marks itself with the private
	 * AIPU_MM_DATA_TYPE_KMD_DMA_BUF so it can be told apart here. While it
	 * had no reclaim path at all, this tripwire fired on
	 * every shipped workload, correctly.
	 *
	 * Any NEW caller passing filp==NULL without naming itself would
	 * silently leak iova_used for the module lifetime — guard against
	 * that here so the leak is observable in dmesg.
	 */
	WARN_ONCE(!filp && buf_req->data_type != AIPU_MM_DATA_TYPE_TCB &&
		  buf_req->data_type != AIPU_MM_DATA_TYPE_KMD_DMA_BUF,
		"aipu_mm: V3 IOVA alloc filp==NULL with unclaimed data_type=%d "
		"asid=%d size=0x%llx — caller will leak iova_used until module deinit\n",
		buf_req->data_type, asid, iova_size);
	buffer->filp = filp;
	buffer->tgid = task_tgid_nr(current);
	buffer->pid = task_pid_nr(current);
	buffer->exec_id = buf_req->exec_id;
	buffer->data_type = buf_req->data_type;
	INIT_LIST_HEAD(&buffer->phy_blocks);
	RB_CLEAR_NODE(&buffer->rb);
	mutex_init(&buffer->phy_mutex);

	/* Return IOVA as pa (this will be adjusted later if needed) */
	buf_req->desc.pa = buffer->iova_start;
	buf_req->desc.dev_offset = buffer->iova_start;
	buf_req->desc.bytes = iova_size;
	buf_req->desc.asid = asid;
	buf_req->desc.region = AIPU_BUF_REGION_DEFAULT;

	/* Add to buffer list */
	mutex_lock(&mm->buffer_mutex);
	list_add_tail(&buffer->node, &mm->buffer_list);
	aipu_buffer_tree_insert(mm, buffer);
	mm->buffer_count++;
	mutex_unlock(&mm->buffer_mutex);

	dev_dbg(mm->dev, "V3: allocated IOVA 0x%llx size 0x%llx for ASID %d\n",
		buffer->iova_start, iova_size, asid);

	return buffer;

err_free_buffer:
	kfree(buffer);
err_unreserve:
	/* Release the quota reserved above so a failed alloc never leaks it. */
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used >= iova_size)
		asid_info->iova_used -= iova_size;
	else
		asid_info->iova_used = 0;
	spin_unlock(&mm->asid_lock);
	return NULL;
}

/**
 * aipu_mm_v3_flush_block_pages_nb - clean+invalidate every page of a block
 * @mm: memory manager
 * @block: physical block whose backing pages are flushed
 *
 * Flushes on the cacheable linear alias (page_address) WITHOUT issuing a
 * completion barrier. The caller MUST issue exactly one barrier
 * (cache_ops.flush_barrier() or dsb(sy)) after the last block it flushes. This
 * replaces the old per-page flush_range(), which emitted a full-system dsb
 * per 4KB page -- thousands of barriers when flushing a large working set.
 *
 * Falls back to the barrier-carrying flush_range on arches that did not opt
 * in to flush_range_nb (behaviour then matches the original code).
 */
static void aipu_mm_v3_flush_block_pages_nb(struct aipu_memory_manager *mm,
					    struct aipu_phy_block *block)
{
	aipu_mm_v3_block_pages_cmo_nb(mm, block, 0, block->size, false);
}

/**
 * aipu_mm_v3_alloc_phy_and_map - Allocate physical pages and map to IOVA
 * @mm: memory manager
 * @buffer: IOVA buffer
 * @buf_req: buffer request for size info
 *
 * Return: 0 on success, error code otherwise
 */
int aipu_mm_v3_alloc_phy_and_map(struct aipu_memory_manager *mm,
					struct aipu_iova_buffer *buffer,
					struct aipu_buf_request *buf_req)
{
	struct aipu_phy_block *block;
	u32 page_count;
	u64 iova_addr;
	int ret;

	if (!mm || !buffer || !buf_req)
		return -EINVAL;

	iova_addr = buffer->iova_start + buffer->allocated_size;
	page_count = buf_req->bytes >> PAGE_SHIFT;
	if (buf_req->bytes & (PAGE_SIZE - 1))
		page_count++;

	dev_dbg(mm->dev, "V3: alloc_phy_and_map iova 0x%llx, allocated 0x%llx\n",
		iova_addr, buffer->allocated_size);

	/* Check space in buffer */
	if (buffer->allocated_size + (page_count << PAGE_SHIFT) > buffer->iova_size) {
		dev_err(mm->dev, "V3: IOVA buffer full (allocated 0x%llx, need %d pages)\n",
			buffer->allocated_size, page_count);
		return -ENOMEM;
	}

	/* Allocate physical pages */
	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return -ENOMEM;

	block->pages = aipu_dma_alloc_pages(mm->dev, page_count,
					    PAGE_SIZE >> PAGE_SHIFT,
					    GFP_KERNEL | __GFP_ZERO);
	if (!block->pages) {
		dev_err(mm->dev, "V3: failed to allocate %d physical pages\n", page_count);
		ret = -ENOMEM;
		goto err_free_block;
	}

	block->size = (u64)page_count << PAGE_SHIFT;
	block->page_count = page_count;
	block->dmabuf = NULL;
	block->bind_memory = false;
	INIT_LIST_HEAD(&block->list);

	/* Build scatter-gather table and map to IOMMU */
	ret = sg_alloc_table_from_pages(&block->sgt, block->pages,
					block->page_count, 0, block->size,
					GFP_KERNEL);
	if (ret) {
		dev_err(mm->dev, "V3: failed to allocate SG table\n");
		goto err_free_pages;
	}

	/* Check if already mapped (should not happen) */
	if (iommu_iova_to_phys(mm->iommu_domain, iova_addr)) {
		dev_err(mm->dev, "V3: IOVA 0x%llx already mapped!\n", iova_addr);
		/* Try to unmap first */
		iommu_unmap(mm->iommu_domain, iova_addr, block->size);
	}

	/* Map to IOMMU at specific IOVA */
	{
		ssize_t mapped = iommu_map_sg(mm->iommu_domain, iova_addr, block->sgt.sgl,
				      block->sgt.nents, IOMMU_READ | IOMMU_WRITE,
				      GFP_KERNEL);
		if (mapped <= 0) {
			dev_err(mm->dev, "V3: IOMMU map failed at IOVA 0x%llx (ret=%zd)\n",
				iova_addr, mapped);
			ret = -ENOMEM;
			goto err_free_sg;
		}
	}

	block->iova_start = iova_addr;
	block->dma_addr = iova_addr;

	/* No kernel alias is created here, for any class. We measured what
	 * an unconditional vmap costs -- ~0.5 GB of vmalloc space and ~127k
	 * kernel PTEs per live graph on a 96 MB working set, plus a
	 * vmalloc-range TLB flush per free -- and the saving is kept,
	 * but the decision no longer comes from buf_req->data_type. That field
	 * is the caller's, the interface obliges it to be populated and
	 * nowhere obliges it to be accurate about how the buffer will be used,
	 * and such a claim must not decide a guarantee the
	 * vendor gives unconditionally (the vendor vmaps every block here).
	 *
	 * Instead the alias is created on demand, by aipu_mm_v3_block_kva(),
	 * when something in the kernel first resolves an address inside the
	 * block -- so the set of aliased blocks is decided by what the kernel
	 * does, not by what the caller said. A block nobody reads still costs
	 * nothing.
	 *
	 * There is consequently nothing after the IOMMU mapping above that can
	 * fail, which is why this function has no err_iommu_unmap label: the
	 * vmap was the only such step, and its failure handling moved with it.
	 */

	/* Add block to buffer */
	mutex_lock(&buffer->phy_mutex);
	buffer->ref_count++;
	buffer->allocated_size += block->size;
	list_add_tail(&block->list, &buffer->phy_blocks);
	mutex_unlock(&buffer->phy_mutex);

	/* Flush (clean+invalidate) cache for the new pages using linear mapping.
	 * __GFP_ZERO fills pages via Normal Cacheable mapping, leaving dirty cache
	 * lines.  On ARM64, DC operations on Device-mapped VA are CONSTRAINED
	 * UNPREDICTABLE, so page_address() (Normal Cacheable) must be used.
	 * Without this flush, dirty cache lines can be evicted after NPU DMA writes,
	 * overwriting NPU output data in DRAM ("cache stomp").
	 *
	 * Use the no-barrier per-page flush and a single trailing barrier rather
	 * than a full-system dsb per page.
	 */
	aipu_mm_v3_flush_block_pages_nb(mm, block);
	if (mm->cache_ops.flush_barrier)
		mm->cache_ops.flush_barrier();

	dev_dbg(mm->dev, "V3: mapped IOVA 0x%llx to %d phys pages\n",
		iova_addr, page_count);

	return 0;

err_free_sg:
	sg_free_table(&block->sgt);
err_free_pages:
	aipu_dma_free_pages(block->pages, block->page_count);
err_free_block:
	kfree(block);
	return ret;
}

/**
 * aipu_mm_v3_block_kva - the block's kernel alias, created if it has none yet
 * @mm:    memory manager
 * @block: the block to alias
 *
 * Replaces the buf_req->data_type gate that used to decide this at allocation
 * time. Nothing a
 * caller wrote in its allocation request takes part; the block is aliased if
 * and only if the kernel dereferences it.
 *
 * Context: MAY SLEEP -- vmap does. Callers must hold buffer->phy_mutex for the
 * buffer owning @block, which is what keeps the block alive across the mapping
 * and serializes two first-touchers; aipu_mm_v3_kva_by_pa() is the only caller
 * and holds it.
 *
 * Concurrency with the plain readers of block->va elsewhere in this file (the
 * cache ioctls, the teardown paths): block->va transitions NULL -> address
 * exactly once in a block's life and never back, and the store below is a
 * release, so an unlocked reader either sees NULL or sees a fully established
 * mapping. A doubled load cannot see two different non-NULL values, which is
 * why `if (block->va) ... block->va` stays correct where it already stood.
 *
 * Failure is reported as NULL -- the same word the callers already handle, all
 * of which turn it into -EFAULT -- rather than as the -ENOMEM the allocation
 * gate raised. An earlier hazard survives the move -- a hold-TCB block
 * reaching aipu_mm_link_tcb with a NULL kva and being dereferenced under
 * mm->shlock. That is still impossible, and now by two independent guards --
 * aipu_mm_hold_tcb_buf_alloc fails the allocation outright if aipu_mm_get_tcb
 * returns NULL, so htbuf->hold_tcb is never set from a failed mapping, and
 * aipu_mm_link_tcb NULL-checks htbuf->hold_tcb under the lock before touching
 * it.
 *
 * Return: the block's kernel virtual address, or NULL if it cannot be mapped.
 */
static void *aipu_mm_v3_block_kva(struct aipu_memory_manager *mm,
				  struct aipu_phy_block *block)
{
	void *va;

	if (!mm || !block || !block->pages || !block->page_count)
		return NULL;

	if (block->va)
		return block->va;

	va = vmap(block->pages, block->page_count, VM_MAP,
		  pgprot_noncached(PAGE_KERNEL));
	if (!va) {
		dev_err(mm->dev, "V3: vmap failed for IOVA 0x%llx size 0x%llx\n",
			block->iova_start, block->size);
		return NULL;
	}

	smp_store_release(&block->va, va);
	return va;
}

/**
 * aipu_mm_v3_kva_by_pa - resolve a device PA to a kernel address, mapping on demand
 * @mm:     memory manager
 * @dev_pa: device physical address as returned by aipu_mm_alloc()
 *
 * Sweeps the ASID windows for one that translates @dev_pa, walks to the block
 * holding the translated IOVA, and returns a kernel address for it -- creating
 * the block's alias if this is the first time anything has asked.
 *
 * The sweep is bounded by ZHOUYI_ASID_COUNT and cannot be ambiguous: the
 * windows are disjoint (asid_base = asid * 4 GB, window 3 GB), so at most one
 * translates a given PA and the rest return 0.
 *
 * This looks the buffer up itself (mm->buffer_tree) rather than calling
 * aipu_mm_v3_find_block_by_iova, and the difference is the point:
 * that function returns a block with both mutexes already dropped, which is
 * sound for a caller that only reads block->va but not for one that WRITES it.
 * Mapping has to happen while the block is still held, or a concurrent teardown
 * (aipu_mm_v3_destroy_buffer_now, which vunmaps and kfrees under phy_mutex)
 * could free the block between the lookup and the store -- and unlike the
 * pre-existing read-after-unlock, that would be a write into freed memory and a
 * leaked mapping.
 *
 * Context: MAY SLEEP, and takes mm->buffer_mutex then buffer->phy_mutex -- the
 * same two, in the same order, that aipu_mm_v3_find_block_by_iova always took
 * here, so no caller acquires a requirement it did not have.
 * ⚠️ One caller does not satisfy that and did not before either: print_core_id()
 * is reached from aipu_job_manager_irq_bottom_half() under
 * spin_lock_irqsave(&manager->lock). It is #ifdef DEBUG, so it is not built; it
 * already slept on these mutexes; it now also reclaims through vmap. Whoever
 * enables DEBUG must move that call out of the lock first.
 *
 * Return: a kernel address for @dev_pa, or NULL if it resolves to nothing or
 * cannot be mapped.
 */
static char *aipu_mm_v3_kva_by_pa(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;
	char *va = NULL;
	u64 iova = 0;
	u32 asid;

	if (!mm || !mm->use_v3_custom_iova)
		return NULL;

	mutex_lock(&mm->buffer_mutex);
	for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
		iova = aipu_mm_v3_pa_to_iova(mm, dev_pa, asid);
		if (!iova)
			continue;

		buffer = aipu_buffer_tree_find(mm, iova);
		if (!buffer || buffer->asid != asid)
			continue;

		mutex_lock(&buffer->phy_mutex);
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			void *kva;

			if (iova < block->iova_start ||
			    iova >= block->iova_start + block->size)
				continue;

			kva = aipu_mm_v3_block_kva(mm, block);
			if (kva)
				va = (char *)kva + (iova - block->iova_start);
			break;
		}
		mutex_unlock(&buffer->phy_mutex);
		if (va)
			break;
	}

	mutex_unlock(&mm->buffer_mutex);

	return va;
}

/**
 * aipu_mm_v3_find_block_by_iova - Find physical block by IOVA address
 * @mm: memory manager
 * @iova: IOVA address
 * @asid: ASID (for validation)
 * 
 * Return: block pointer or NULL
 */
struct aipu_phy_block *aipu_mm_v3_find_block_by_iova(struct aipu_memory_manager *mm,
							      u64 iova, u32 asid)
{
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;

	if (!mm || !mm->use_v3_custom_iova)
		return NULL;

	mutex_lock(&mm->buffer_mutex);
	buffer = aipu_buffer_tree_find(mm, iova);
	if (buffer && buffer->asid == asid) {
		mutex_lock(&buffer->phy_mutex);
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			if (iova >= block->iova_start &&
			    iova < block->iova_start + block->size) {
				mutex_unlock(&buffer->phy_mutex);
				mutex_unlock(&mm->buffer_mutex);
				return block;
			}
		}
		mutex_unlock(&buffer->phy_mutex);
	}
	mutex_unlock(&mm->buffer_mutex);

	return NULL;
}

/**
 * aipu_mm_v3_pa_to_iova - Convert device PA to IOVA
 * @mm: memory manager
 * @dev_pa: device physical address (asid_base + offset)
 * @asid: ASID
 * 
 * Return: IOVA address or 0 on error
 */
u64 aipu_mm_v3_pa_to_iova(struct aipu_memory_manager *mm, u64 dev_pa, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 asid_base;

	if (!mm || !mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	asid_info = &mm->asid_iova[asid];
	asid_base = asid_info->asid_base;

	/* Check if dev_pa is in valid range */
	if (dev_pa < asid_base ||
	    dev_pa >= asid_base + asid_info->iova_size) {
		return 0;
	}

	/* IOVA = offset = dev_pa - asid_base + iova_base */
	return (dev_pa - asid_base) + asid_info->iova_base;
}

/**
 * aipu_mm_v3_iova_to_pa - Convert IOVA to device PA
 * @mm: memory manager
 * @iova: IOVA address
 * @asid: ASID
 * 
 * Return: device PA or 0 on error
 */
u64 aipu_mm_v3_iova_to_pa(struct aipu_memory_manager *mm, u64 iova, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 offset;

	if (!mm || !mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	asid_info = &mm->asid_iova[asid];

	/* Check if IOVA is in valid range */
	if (iova < asid_info->iova_base ||
	    iova >= asid_info->iova_base + asid_info->iova_size) {
		return 0;
	}

	/* PA = asid_base + offset = asid_base + (iova - iova_base) */
	offset = iova - asid_info->iova_base;
	return asid_info->asid_base + offset;
}

/**
 * aipu_mm_v3_get_tcb_addr - Get TCB address (relative offset)
 * @mm: memory manager
 * @dev_pa: device physical address
 * @asid: ASID
 * 
 * Return: 32-bit relative offset for TCB
 */
u32 aipu_mm_v3_get_tcb_addr(struct aipu_memory_manager *mm, u64 dev_pa, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 offset;

	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	asid_info = &mm->asid_iova[asid];

	if (dev_pa < asid_info->asid_base) {
		dev_err(mm->dev, "V3: dev_pa 0x%llx < asid_base 0x%llx\n",
			dev_pa, asid_info->asid_base);
		return 0;
	}

	offset = dev_pa - asid_info->asid_base;

	/* Check offset is within 32-bit range */
	if (offset >= 0x100000000ULL) {
		dev_err(mm->dev, "V3: offset 0x%llx exceeds 32-bit\n", offset);
		return 0;
	}

	return lower_32_bits(offset);
}

/**
 * aipu_mm_v3_map_imported_sgt - Map an imported dma-buf into the V3 custom IOVA domain
 * @mm:       memory manager
 * @sgt:      scatter-gather table of the imported dma-buf (physical pages)
 * @asid:     target ASID (the default context uses ASID 0)
 * @out_pa:   [out] NPU-visible address (asid_base + offset) to hand back to UMD
 * @out_size: [out] page-aligned size that was mapped
 *
 * The standard DMA API (dma_buf_map_attachment/sg_dma_address) maps into the
 * kernel's default DMA-IOMMU cookie allocator, whose IOVAs do NOT lie inside a
 * V3 ASID's usable window once the custom IOVA domain is active. Route imported
 * dma-bufs through the same custom domain that normal NPU buffers use so the NPU
 * can translate them. Mirrors aipu_mm_v3_alloc_iova()/alloc_phy_and_map().
 *
 * Return: 0 on success, negative errno otherwise.
 */
int aipu_mm_v3_map_imported_sgt(struct aipu_memory_manager *mm, struct sg_table *sgt,
				u32 asid, u64 *out_pa, u64 *out_size)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct aipu_asid_iova_info *asid_info;
	struct scatterlist *sg;
	struct iova *iova;
	u64 total = 0, size, iova_addr;
	unsigned long pfn_lo, pfn_hi;
	ssize_t mapped;
	int i;

	if (!mm || !sgt || !out_pa || !out_size)
		return -EINVAL;

	if (!mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return -EINVAL;

	asid_info = &mm->asid_iova[asid];

	/* Physical view: sum the original page-granular segment lengths. */
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i)
		total += sg->length;

	size = ALIGN(total, PAGE_SIZE);
	if (!size)
		return -EINVAL;

	/* Atomically check + reserve quota (see aipu_mm_v3_alloc_iova). */
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used + size > asid_info->iova_size) {
		spin_unlock(&mm->asid_lock);
		dev_err(mm->dev, "V3: ASID %d out of IOVA space for import (used 0x%llx, need 0x%llx)\n",
			asid, asid_info->iova_used, size);
		return -ENOMEM;
	}
	asid_info->iova_used += size;
	spin_unlock(&mm->asid_lock);

	pfn_lo = asid_info->iova_base >> PAGE_SHIFT;
	pfn_hi = ((asid_info->iova_base + asid_info->iova_size) >> PAGE_SHIFT) - 1;

	/* size_aligned=false: page alignment is enough (see aipu_mm_v3_alloc_iova). */
	aipu_mm_v3_iova_search_from_top(iovad, asid);
	iova = alloc_iova(iovad, size >> PAGE_SHIFT, pfn_hi, false);
	if (!iova) {
		dev_err(mm->dev, "V3: alloc_iova failed for import (ASID %d, size 0x%llx)\n",
			asid, size);
		goto err_unreserve;
	}

	if (iova->pfn_lo < pfn_lo || iova->pfn_hi > pfn_hi) {
		dev_err(mm->dev, "V3: import IOVA 0x%lx-0x%lx outside ASID %d range\n",
			iova->pfn_lo, iova->pfn_hi, asid);
		__free_iova(iovad, iova);
		goto err_unreserve;
	}

	iova_addr = (u64)iova->pfn_lo << PAGE_SHIFT;

	mapped = iommu_map_sg(mm->iommu_domain, iova_addr, sgt->sgl, sgt->orig_nents,
			      IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (mapped < (ssize_t)total) {
		dev_err(mm->dev, "V3: iommu_map_sg failed for import at IOVA 0x%llx (ret=%zd, need 0x%llx)\n",
			iova_addr, mapped, total);
		if (mapped > 0)
			iommu_unmap(mm->iommu_domain, iova_addr, mapped);
		__free_iova(iovad, iova);
		goto err_unreserve;
	}

	*out_pa = aipu_mm_v3_iova_to_pa(mm, iova_addr, asid);
	*out_size = size;

	dev_dbg(mm->dev, "V3: imported dma-buf mapped IOVA 0x%llx size 0x%llx ASID %d, pa 0x%llx\n",
		iova_addr, size, asid, *out_pa);

	return 0;

err_unreserve:
	/* Release the quota reserved above so a failed import never leaks it. */
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used >= size)
		asid_info->iova_used -= size;
	else
		asid_info->iova_used = 0;
	spin_unlock(&mm->asid_lock);
	return -ENOMEM;
}

/**
 * aipu_mm_v3_unmap_imported_sgt - Tear down a mapping made by aipu_mm_v3_map_imported_sgt
 * @mm:     memory manager
 * @dev_pa: the NPU pa (out_pa) returned by the map helper
 * @size:   the page-aligned size returned by the map helper
 * @asid:   ASID the import was mapped into
 *
 * Only unmaps/frees the custom-domain IOVA. The dma-buf attachment (and thus
 * the backing pages) is released by the caller via dma_buf_unmap_attachment().
 */
void aipu_mm_v3_unmap_imported_sgt(struct aipu_memory_manager *mm, u64 dev_pa,
				   u64 size, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 dev_iova;

	if (!mm || !mm->use_v3_custom_iova || !dev_pa || !size)
		return;
	if (asid >= ZHOUYI_ASID_COUNT)
		return;

	dev_iova = aipu_mm_v3_pa_to_iova(mm, dev_pa, asid);
	if (!dev_iova) {
		dev_warn(mm->dev, "V3: cannot unmap import, bad pa 0x%llx ASID %d\n",
			 dev_pa, asid);
		return;
	}

	iommu_unmap(mm->iommu_domain, dev_iova, size);
	free_iova(&mm->iova_domain, dev_iova >> PAGE_SHIFT);

	asid_info = &mm->asid_iova[asid];
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used >= size)
		asid_info->iova_used -= size;
	else
		asid_info->iova_used = 0;
	spin_unlock(&mm->asid_lock);

	dev_dbg(mm->dev, "V3: unmapped imported dma-buf pa 0x%llx (IOVA 0x%llx) size 0x%llx ASID %d\n",
		dev_pa, dev_iova, size, asid);
}

/**
 * aipu_mm_v3_destroy_buffer_now - Immediately tear down a V3 IOVA buffer.
 * @mm: memory manager
 * @buffer: buffer to destroy
 *
 * Unmaps from IOMMU, frees the IOVA span, frees backing pages, and removes the
 * buffer from buffer_list. Used by the alloc-failure error path (buffer never
 * became visible to the NPU) and by aipu_mm_v3_free_buffers_by_filp (fd close,
 * at which point no NPU work for this fd can still be in flight).
 */
void aipu_mm_v3_destroy_buffer_now(struct aipu_memory_manager *mm,
				   struct aipu_iova_buffer *buffer)
{
	struct aipu_phy_block *block, *tmp;
	struct aipu_asid_iova_info *asid_info;
	struct iova_domain *iovad = &mm->iova_domain;

	if (!mm || !buffer)
		return;

	/*
	 * The buffer's V3 TCB descriptors go with it (kmd-walk). FREE_BUF
	 * always dropped the one at the freed address; fd close dropped none,
	 * so they stayed on the list for the module's lifetime and shadowed
	 * any later buffer given the same IOVA.
	 */
	{
		u64 pa = aipu_mm_v3_iova_to_pa(mm, buffer->iova_start, buffer->asid);

		if (pa)
			aipu_tcb_bufs_drop_range(mm, pa, pa + buffer->iova_size);
	}

	/* Unmap and free all physical blocks */
	mutex_lock(&buffer->phy_mutex);
	list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
		if (block->iova_start && block->size) {
			/* Unmap from IOMMU */
			size_t _un = iommu_unmap(mm->iommu_domain,
						 block->iova_start, block->size);
			(void)_un;

		}

		if (block->va)
			vunmap(block->va);

		if (block->sgt.nents)
			sg_free_table(&block->sgt);

		if (block->pages) {
			if (!block->bind_memory)
				aipu_dma_free_pages(block->pages, block->page_count);
		}

		list_del(&block->list);
		kfree(block);
	}
	mutex_unlock(&buffer->phy_mutex);

	/* Update ASID used count */
	if (buffer->asid < ZHOUYI_ASID_COUNT) {
		asid_info = &mm->asid_iova[buffer->asid];
		spin_lock(&mm->asid_lock);
		if (asid_info->iova_used >= buffer->iova_size)
			asid_info->iova_used -= buffer->iova_size;
		else
			asid_info->iova_used = 0;
		spin_unlock(&mm->asid_lock);
	}

	/*
	 * Remove from buffer list (and tree) BEFORE the IOVA goes back to the
	 * domain: a concurrent allocation may be handed the same range at once,
	 * and the address lookups rely on the published ranges being disjoint.
	 */
	mutex_lock(&mm->buffer_mutex);
	list_del(&buffer->node);
	aipu_buffer_tree_erase(mm, buffer);
	mm->buffer_count--;
	mutex_unlock(&mm->buffer_mutex);

	/* Free IOVA from domain */
	free_iova(iovad, buffer->iova_start >> PAGE_SHIFT);

	dev_dbg(mm->dev, "V3: freed buffer IOVA 0x%llx size 0x%llx ASID %d\n",
		buffer->iova_start, buffer->iova_size, buffer->asid);

	kfree(buffer);
}

/**
 * aipu_mm_v3_free_iova_buffer - Free a V3 IOVA buffer (immediate).
 * @mm: memory manager
 * @buffer: buffer to free
 *
 * Runtime free path (aipu_free_dma_iova_phy). Tears down immediately.
 *
 * The use-after-unmap (DPTSW-23453 SMMU event 0x10) is now fixed at the source
 * in the UMD: reuse/feature-map buffers are persisted at graph level (no
 * per-job free), so no NPU-walked state references a freed IOVA. The KMD no
 * longer needs to defer iommu_unmap. If the UMD persistence is ever disabled,
 * re-enable the size-gated defer (keep aipu_mm_v3_destroy_buffer_now) — see
 * git history for the deferred version.
 */
void aipu_mm_v3_free_iova_buffer(struct aipu_memory_manager *mm,
					struct aipu_iova_buffer *buffer)
{
	if (!mm || !buffer)
		return;

	aipu_mm_v3_destroy_buffer_now(mm, buffer);
}

/**
 * aipu_mm_v3_free_buffers_by_filp - Free all buffers belonging to a file
 * @mm: memory manager
 * @filp: file pointer
 *
 * Called when fd is closed
 *
 * Buffers with buffer->filp == NULL are intentionally NOT
 * reclaimed here: the hold-TCB cross-fd pool (aipu_mm_hold_tcb_buf_alloc),
 * reclaimed at deinit by aipu_mm_hold_tcb_buf_free, and the dma-buf
 * exporter's backing allocation (aipu_alloc_dma_buf), reclaimed by the
 * export's own refcount in aipu_dma_release. The filp != NULL guard at
 * entry keeps both out of this sweep; the WARN_ONCE in
 * aipu_mm_v3_alloc_iova catches any new caller that enters the NULL-owner
 * regime without naming itself and so has no reclaim path at all.
 */
void aipu_mm_v3_free_buffers_by_filp(struct aipu_memory_manager *mm,
					     struct file *filp)
{
	struct aipu_iova_buffer *buffer, *tmp;
	struct aipu_iova_buffer **buffers_to_free;
	int count = 0, i = 0;

	if (!mm || !mm->use_v3_custom_iova || !filp)
		return;

	/* First pass: count matching buffers */
	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (buffer->filp == filp)
			count++;
	}
	mutex_unlock(&mm->buffer_mutex);

	if (count == 0)
		return;

	/* Allocate temp array */
	buffers_to_free = kcalloc(count, sizeof(*buffers_to_free), GFP_KERNEL);
	if (!buffers_to_free)
		return;

	/* Second pass: collect buffers */
	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry_safe(buffer, tmp, &mm->buffer_list, node) {
		if (buffer->filp == filp && i < count) {
			buffers_to_free[i] = buffer;
			i++;
		}
	}
	mutex_unlock(&mm->buffer_mutex);

	/* Free them.
	 * At fd close no NPU work for this fd can still be in flight, so it is
	 * safe to actually tear down the mappings now. Deferred buffers (freed
	 * earlier via aipu_free_dma_iova_phy) and any still-live buffers both
	 * get destroyed immediately here. */
	for (i = 0; i < count; i++) {
		if (buffers_to_free[i])
			aipu_mm_v3_destroy_buffer_now(mm, buffers_to_free[i]);
	}

	kfree(buffers_to_free);
	dev_dbg(mm->dev, "V3: freed %d buffers for filp\n", count);
}


/*
 * Zero-copy weights (V3 custom IOVA).
 *
 * A model whose weights exceed the NPU's 3 GB window used to copy each layer into window buffers every
 * token: the CPU read and wrote every weight byte, competing with the NPU for DDR. Here the weights live
 * once in host memory (aipu_wbuf: 2 MiB chunks, outside the window, filled by the process through a
 * write-combine mmap), and a window range (aipu_slot) is re-pointed at a layer's chunks per token: one
 * SMMU block entry per chunk and one TLB invalidation, no data movement, no per-token cache maintenance.
 *
 * Ordering is the caller's: a slot must not be re-mapped while a submitted job still reads it. iommu_unmap
 * returns after the SMMU's TLB invalidation has completed, so a job submitted after SLOT_MAP returns sees
 * only the new mapping.
 */
/* Testing: every weight-buffer unit as 512 order-0 pages (the fallback when no 2 MiB block is free). */
static bool wbuf_force_small;
module_param(wbuf_force_small, bool, 0644);
MODULE_PARM_DESC(wbuf_force_small, "zero-copy weight buffers: always use 4 KiB pages (tests the fallback)");

static bool aipu_wslot_supported(struct aipu_memory_manager *mm)
{
	return mm && mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->has_iommu && mm->use_v3_custom_iova;
}

static struct aipu_wbuf *aipu_wbuf_find(struct aipu_memory_manager *mm, u64 id, struct file *filp)
{
	struct aipu_wbuf *w;

	list_for_each_entry(w, &mm->wbufs, node)
		if (w->id == id && w->filp == filp)
			return w;
	return NULL;
}

static struct aipu_slot *aipu_slot_find(struct aipu_memory_manager *mm, u64 id, struct file *filp)
{
	struct aipu_slot *sl;

	list_for_each_entry(sl, &mm->slots, node)
		if (sl->id == id && sl->filp == filp)
			return sl;
	return NULL;
}

#define AIPU_WUNIT_PAGES	(AIPU_WCHUNK >> PAGE_SHIFT)

static void aipu_wbuf_destroy(struct aipu_wbuf *w)
{
	u32 i, j;

	for (i = 0; i < w->nchunks; i++) {
		if (w->chunks[i])
			__free_pages(w->chunks[i], get_order(AIPU_WCHUNK));
		if (w->small && w->small[i]) {
			for (j = 0; j < AIPU_WUNIT_PAGES; j++)
				if (w->small[i][j])
					__free_page(w->small[i][j]);
			kvfree(w->small[i]);
		}
	}
	kvfree(w->small);
	kvfree(w->chunks);
	kfree(w);
}

/* A unit of 512 order-0 pages when no 2 MiB block is available: mapped with page entries instead of one block entry. */
static int aipu_wbuf_small_unit(struct aipu_wbuf *w, u32 i, void (*op)(void *, size_t))
{
	u32 j;

	w->small[i] = kvcalloc(AIPU_WUNIT_PAGES, sizeof(struct page *), GFP_KERNEL);
	if (!w->small[i])
		return -ENOMEM;
	for (j = 0; j < AIPU_WUNIT_PAGES; j++) {
		w->small[i][j] = alloc_page(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN);
		if (!w->small[i][j])
			return -ENOMEM;
		if (op)
			op(page_address(w->small[i][j]), PAGE_SIZE);
	}
	return 0;
}

int aipu_wbuf_alloc(struct aipu_memory_manager *mm, struct aipu_wbuf_req *req, struct file *filp)
{
	void (*op)(void *, size_t);
	struct aipu_wbuf *w;
	u64 n;
	u32 i;

	if (!aipu_wslot_supported(mm))
		return -EOPNOTSUPP;
	if (!req->bytes)
		return -EINVAL;
	n = DIV_ROUND_UP(req->bytes, (u64)AIPU_WCHUNK);
	if (n > (1ULL << (AIPU_WBUF_MMAP_SHIFT - 21)))	/* one mmap stride: 64 GiB */
		return -EINVAL;

	w = kzalloc(sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->chunks = kvcalloc(n, sizeof(*w->chunks), GFP_KERNEL);
	w->small = kvcalloc(n, sizeof(*w->small), GFP_KERNEL);
	if (!w->chunks || !w->small) {
		kvfree(w->chunks);
		kvfree(w->small);
		kfree(w);
		return -ENOMEM;
	}
	w->nchunks = n;
	atomic_set(&w->mmaps, 0);

	/*
	 * Zeroed (nothing of the kernel's may reach the process) through the cacheable linear alias, then
	 * cleaned+invalidated: a dirty zero line evicted later would overwrite the weights the process
	 * writes through its non-cacheable mapping. Once per buffer, never per token.
	 */
	op = mm->cache_ops.flush_range_nb ? mm->cache_ops.flush_range_nb : mm->cache_ops.flush_range;
	for (i = 0; i < n; i++) {
		/* a 2 MiB block if one is free right now (no reclaim or compaction: those stall for long), else 512 pages */
		w->chunks[i] = wbuf_force_small ? NULL : alloc_pages(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN | __GFP_NORETRY,
									  get_order(AIPU_WCHUNK));
		if (w->chunks[i]) {
			w->nbig++;
			if (op)
				op(page_address(w->chunks[i]), AIPU_WCHUNK);
		} else if (aipu_wbuf_small_unit(w, i, op)) {
			dev_info(mm->dev, "WBUF_ALLOC: out of memory at unit %u of %llu\n", i, n);
			aipu_wbuf_destroy(w);
			return -ENOMEM;
		}
		cond_resched();
	}
	if (mm->cache_ops.flush_barrier)
		mm->cache_ops.flush_barrier();

	mutex_lock(&mm->wslot_lock);
	w->id = ++mm->wslot_next_id;
	w->filp = filp;
	list_add_tail(&w->node, &mm->wbufs);
	mutex_unlock(&mm->wslot_lock);

	req->id = w->id;
	req->mmap_offset = AIPU_WBUF_MMAP_BASE + (w->id << AIPU_WBUF_MMAP_SHIFT);
	if (w->nbig < w->nchunks)
		dev_dbg(mm->dev, "WBUF_ALLOC: %u of %u units as 2 MiB blocks, the rest 4 KiB pages\n", w->nbig, w->nchunks);
	return 0;
}

int aipu_wbuf_free(struct aipu_memory_manager *mm, u64 id, struct file *filp)
{
	struct aipu_wbuf *w;
	int ret = 0;

	mutex_lock(&mm->wslot_lock);
	w = aipu_wbuf_find(mm, id, filp);
	if (!w)
		ret = -EINVAL;
	else if (w->slot_refs || atomic_read(&w->mmaps))
		ret = -EBUSY;		/* still mapped in a slot, or into the process */
	else
		list_del(&w->node);
	mutex_unlock(&mm->wslot_lock);
	if (!ret)
		aipu_wbuf_destroy(w);
	return ret;
}

static void aipu_wbuf_vm_open(struct vm_area_struct *vma)
{
	struct aipu_wbuf *w = vma->vm_private_data;

	atomic_inc(&w->mmaps);
}

static void aipu_wbuf_vm_close(struct vm_area_struct *vma)
{
	struct aipu_wbuf *w = vma->vm_private_data;

	atomic_dec(&w->mmaps);
}

static const struct vm_operations_struct aipu_wbuf_vm_ops = {
	.open = aipu_wbuf_vm_open,
	.close = aipu_wbuf_vm_close,
};

int aipu_wbuf_mmap(struct aipu_memory_manager *mm, struct vm_area_struct *vma, struct file *filp)
{
	u64 off = (u64)vma->vm_pgoff << PAGE_SHIFT, size = vma->vm_end - vma->vm_start, done = 0;
	struct aipu_wbuf *w;
	int ret = 0;
	u32 i;

	if (off < AIPU_WBUF_MMAP_BASE || (off - AIPU_WBUF_MMAP_BASE) & ((1ULL << AIPU_WBUF_MMAP_SHIFT) - 1))
		return -EINVAL;		/* a whole buffer from its start, nothing else */
	mutex_lock(&mm->wslot_lock);
	w = aipu_wbuf_find(mm, (off - AIPU_WBUF_MMAP_BASE) >> AIPU_WBUF_MMAP_SHIFT, filp);
	if (!w || size > (u64)w->nchunks * AIPU_WCHUNK) {
		mutex_unlock(&mm->wslot_lock);
		return -EINVAL;
	}
	vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);	/* Normal-NC, as every buffer of this driver */
	for (i = 0; done < size && !ret; i++) {
		u64 len = min_t(u64, AIPU_WCHUNK, size - done), p;

		if (w->chunks[i])
			ret = remap_pfn_range(vma, vma->vm_start + done, page_to_pfn(w->chunks[i]), len, vma->vm_page_prot);
		else
			for (p = 0; p < len && !ret; p += PAGE_SIZE)
				ret = remap_pfn_range(vma, vma->vm_start + done + p, page_to_pfn(w->small[i][p >> PAGE_SHIFT]),
						      PAGE_SIZE, vma->vm_page_prot);
		done += len;
	}
	if (!ret) {
		vma->vm_private_data = w;
		vma->vm_ops = &aipu_wbuf_vm_ops;
		atomic_inc(&w->mmaps);
	}
	mutex_unlock(&mm->wslot_lock);
	return ret;
}

static void aipu_slot_unmap_locked(struct aipu_memory_manager *mm, struct aipu_slot *sl)
{
	if (sl->mapped) {
		size_t un = iommu_unmap(mm->iommu_domain, sl->iova_start, sl->mapped);

		if (un != sl->mapped)
			dev_warn(mm->dev, "SLOT_MAP: unmapped 0x%zx of 0x%llx at IOVA 0x%llx\n",
				 un, sl->mapped, sl->iova_start);
	}
	if (sl->wbuf)
		sl->wbuf->slot_refs--;
	sl->mapped = 0;
	sl->wbuf = NULL;
}

int aipu_slot_alloc(struct aipu_memory_manager *mm, struct aipu_slot_req *req, struct file *filp)
{
	struct aipu_asid_iova_info *ai;
	unsigned long pfn_hi;
	struct aipu_slot *sl;
	u64 bytes, total;

	if (!aipu_wslot_supported(mm))
		return -EOPNOTSUPP;
	if (!req->bytes)
		return -EINVAL;
	bytes = ALIGN(req->bytes, (u64)AIPU_WCHUNK);
	total = bytes + AIPU_WCHUNK;		/* slack to align the start to a chunk */
	ai = &mm->asid_iova[0];

	spin_lock(&mm->asid_lock);
	if (ai->iova_used + total > ai->iova_size) {
		spin_unlock(&mm->asid_lock);
		return -ENOMEM;
	}
	ai->iova_used += total;
	spin_unlock(&mm->asid_lock);

	sl = kzalloc(sizeof(*sl), GFP_KERNEL);
	if (!sl)
		goto err_unreserve;
	pfn_hi = ((ai->iova_base + ai->iova_size) >> PAGE_SHIFT) - 1;
	sl->iova = alloc_iova(&mm->iova_domain, total >> PAGE_SHIFT, pfn_hi, false);
	if (!sl->iova || ((u64)sl->iova->pfn_lo << PAGE_SHIFT) < ai->iova_base) {
		if (sl->iova)
			__free_iova(&mm->iova_domain, sl->iova);
		kfree(sl);
		goto err_unreserve;
	}
	sl->iova_start = ALIGN((u64)sl->iova->pfn_lo << PAGE_SHIFT, (u64)AIPU_WCHUNK);
	sl->bytes = bytes;
	sl->reserved = total;

	mutex_lock(&mm->wslot_lock);
	sl->id = ++mm->wslot_next_id;
	sl->filp = filp;
	list_add_tail(&sl->node, &mm->slots);
	mutex_unlock(&mm->wslot_lock);

	req->id = sl->id;
	req->pa = aipu_mm_v3_iova_to_pa(mm, sl->iova_start, 0);
	return 0;

err_unreserve:
	spin_lock(&mm->asid_lock);
	ai->iova_used -= total;
	spin_unlock(&mm->asid_lock);
	return -ENOMEM;
}

static void aipu_slot_destroy_locked(struct aipu_memory_manager *mm, struct aipu_slot *sl)
{
	aipu_slot_unmap_locked(mm, sl);
	list_del(&sl->node);
	__free_iova(&mm->iova_domain, sl->iova);
	spin_lock(&mm->asid_lock);
	mm->asid_iova[0].iova_used -= sl->reserved;
	spin_unlock(&mm->asid_lock);
	kfree(sl);
}

int aipu_slot_free(struct aipu_memory_manager *mm, u64 id, struct file *filp)
{
	struct aipu_slot *sl;
	int ret = 0;

	mutex_lock(&mm->wslot_lock);
	sl = aipu_slot_find(mm, id, filp);
	if (sl)
		aipu_slot_destroy_locked(mm, sl);
	else
		ret = -EINVAL;
	mutex_unlock(&mm->wslot_lock);
	return ret;
}

int aipu_slot_map(struct aipu_memory_manager *mm, struct aipu_slot_map *map, struct file *filp)
{
	struct aipu_slot *sl;
	struct aipu_wbuf *w = NULL;
	u64 bytes, first, i, done;
	int ret = 0;

	if (!aipu_wslot_supported(mm))
		return -EOPNOTSUPP;
	bytes = ALIGN(map->bytes, (u64)AIPU_WCHUNK);

	mutex_lock(&mm->wslot_lock);
	sl = aipu_slot_find(mm, map->slot_id, filp);
	if (!sl) {
		ret = -EINVAL;
		goto out;
	}
	if (bytes) {
		w = aipu_wbuf_find(mm, map->wbuf_id, filp);
		if (!w || map->offset & (AIPU_WCHUNK - 1) || bytes > sl->bytes ||
		    map->offset + bytes > (u64)w->nchunks * AIPU_WCHUNK) {
			ret = -EINVAL;
			goto out;
		}
	}
	aipu_slot_unmap_locked(mm, sl);		/* the old mapping and its TLB entries are gone before the new one */
	first = map->offset / AIPU_WCHUNK;
	done = 0;				/* bytes mapped from the slot's start, exactly (the unwind unmaps these) */
	for (i = 0; i < bytes / AIPU_WCHUNK && !ret; i++) {
		u64 iova = sl->iova_start + i * AIPU_WCHUNK;
		u32 j;

		if (w->chunks[first + i]) {	/* one block entry */
			ret = iommu_map(mm->iommu_domain, iova, page_to_phys(w->chunks[first + i]), AIPU_WCHUNK,
					IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
			if (!ret)
				done += AIPU_WCHUNK;
			continue;
		}
		for (j = 0; j < AIPU_WUNIT_PAGES && !ret; ) {	/* 512 page entries, physically adjacent pages merged */
			phys_addr_t pa = page_to_phys(w->small[first + i][j]);
			u32 run = 1;

			while (j + run < AIPU_WUNIT_PAGES &&
			       page_to_phys(w->small[first + i][j + run]) == pa + ((u64)run << PAGE_SHIFT))
				run++;
			ret = iommu_map(mm->iommu_domain, iova + ((u64)j << PAGE_SHIFT), pa, (size_t)run << PAGE_SHIFT,
					IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
			if (!ret)
				done += (u64)run << PAGE_SHIFT;
			j += run;
		}
	}
	if (ret) {
		dev_err(mm->dev, "SLOT_MAP: iommu_map failed at IOVA 0x%llx (%d)\n", sl->iova_start + done, ret);
		sl->mapped = done;
		sl->wbuf = NULL;
		aipu_slot_unmap_locked(mm, sl);
		goto out;
	}
	sl->mapped = bytes;
	if (bytes) {
		sl->wbuf = w;
		w->slot_refs++;
	}
out:
	mutex_unlock(&mm->wslot_lock);
	return ret;
}

/* fd close: every slot (unmapping it), then every weight buffer. mmap()s hold the file, so none is left. */
void aipu_wslot_release(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_slot *sl, *sn;
	struct aipu_wbuf *w, *wn;

	if (!mm || !mm->slots.next)
		return;
	mutex_lock(&mm->wslot_lock);
	list_for_each_entry_safe(sl, sn, &mm->slots, node)
		if (sl->filp == filp)
			aipu_slot_destroy_locked(mm, sl);
	list_for_each_entry_safe(w, wn, &mm->wbufs, node)
		if (w->filp == filp) {
			list_del(&w->node);
			aipu_wbuf_destroy(w);
		}
	mutex_unlock(&mm->wslot_lock);
}
