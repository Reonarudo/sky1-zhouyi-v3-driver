// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/irqreturn.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "aipu_priv.h"
#include "zhouyi.h"
#include "v3.h"
#include "aipu_io.h"
#include "config.h"

static void zhouyi_v3_set_partition(struct aipu_partition *partition, u32 cluster_id)
{
	u32 val = 0;

	aipu_read32(partition->reg, CLUSTER_CONFIG_REG(cluster_id), &val);
	aipu_write32(partition->reg, CLUSTER_CONTROL_REG(cluster_id),
		     ENABLE_CLUSTER(partition->id, GET_NUMS(val)));
}

static void zhouyi_v3_enable_core_cnt(struct aipu_partition *partition, u32 cluster_id,
				      u32 en_core_cnt)
{
	u32 val = 0;
	u32 nums = 0;
	u32 config = 0;
	u32 status = 0;

	aipu_read32(partition->reg, CLUSTER_CONFIG_REG(cluster_id), &val);
	nums = GET_NUMS(val);
	config = CONFIG_CLUSTER(partition->id, en_core_cnt, GET_AIFF_NUM(nums),
				GET_TEC_NUM(nums));

	aipu_write32(partition->reg, CLUSTER_CONTROL_REG(cluster_id), config);

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (!en_core_cnt && IS_CMD_FAIL(status))
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));

	atomic_set(&partition->clusters[cluster_id].en_core_cnt, en_core_cnt);
	dev_info(partition->dev, "configure cluster #%u done: en_core_cnt %u (0x%x)\n",
		 cluster_id, en_core_cnt, config);
}

static void zhouyi_v3_enable_interrupt(struct aipu_partition *partition, bool en_tec_intr)
{
	u32 cmd_pool_id = partition->id;
	u32 flag = EN_CORE_INTR | EN_CLUSTER_INTR | EN_ALL_TYPE_INTRS_V3;

	if (en_tec_intr)
		flag |= EN_TEC_INTR;

	dev_dbg(partition->dev, "configure interrupt flag 0x%x\n", flag);
	aipu_write32(partition->reg, CMD_POOL_INTR_CTRL_REG(cmd_pool_id), flag);
}

static void zhouyi_v3_config_partition_cmd_pool(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;

	aipu_write32(partition->reg, CMD_POOL_CONFIG_REG(cmd_pool_id),
		     CONFIG_COMMAND_POOL(partition->id));
	aipu_write32(partition->reg, CMD_POOL_SECURE_REG_REG(cmd_pool_id), SET_NONSECURE_MODE);
}

static void zhouyi_v3_disable_interrupt(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;

	aipu_write32(partition->reg, CMD_POOL_INTR_CTRL_REG(cmd_pool_id), DISABLE_ALL_INTRS);
}

static void zhouyi_v3_trigger(struct aipu_partition *partition)
{
	/* no operation here */
}

static void zhouyi_v3_destroy_command_pool_internal(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_HIGH_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_LOW_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_INFO_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DESTROY_CMD_POOL(partition->id));
}

static int zhouyi_v3_destroy_create_command_pool(struct aipu_partition *partition)
{
	u32 status = 0;

	zhouyi_v3_destroy_command_pool_internal(partition);

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (IS_CMD_FAIL(status)) {
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		dev_err(partition->dev, "destroy-create command pool #%d failed: destroy\n",
			partition->id);
		return -EFAULT;
	}

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_CREATE_CMD_POOL(partition->id, TSM_MAP_ALL));

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (IS_CMD_FAIL(status)) {
		dev_err(partition->dev, "destroy-create command pool #%d failed: create\n",
			partition->id);
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		return -EFAULT;
	}

	dev_dbg(partition->dev, "destroy-create command pool #%d successfully\n", partition->id);
	return 0;
}

static int zhouyi_v3_create_command_pool(struct aipu_partition *partition)
{
	u32 status = 0;
	u32 pool_status = 0;
	int ret = 0;

	aipu_read32(partition->reg, CMD_POOL_STATUS_REG(partition->id), &pool_status);
	if (IS_CMD_POOL_BUSY(pool_status))
		return ret;

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_CREATE_CMD_POOL(partition->id, TSM_MAP_ALL));

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (IS_CMD_FAIL(status)) {
		dev_dbg(partition->dev, "create command pool #%d failed, try to destroy-create\n",
			partition->id);
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		ret = zhouyi_v3_destroy_create_command_pool(partition);
		if (ret)
			return ret;
	}

	dev_dbg(partition->dev, "command pool #%d was created\n", partition->id);
	return ret;
}

static int zhouyi_v3_abort_command_pool(struct aipu_partition *partition, int pool)
{
	u32 status = 0;

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_ABORT_CMD_POOL(partition->id));

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (IS_CMD_FAIL(status)) {
		dev_err(partition->dev, "abort command pool #%d failed (cmd 0x%x)\n",
			partition->id, TSM_ABORT_CMD_POOL(partition->id));
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		return -EFAULT;
	}

	udelay(partition->priv->reset_delay_us);
	dev_dbg(partition->dev, "command pool #%d was aborted\n", partition->id);
	return 0;
}

static int zhouyi_v3_destroy_command_pool(struct aipu_partition *partition, int pool)
{
	u32 status = 0;

	zhouyi_v3_destroy_command_pool_internal(partition);
	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (IS_CMD_FAIL(status)) {
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		zhouyi_v3_abort_command_pool(partition, ZHOUYI_COMMAND_POOL_DEFAULT);
		zhouyi_v3_destroy_command_pool_internal(partition);
		aipu_read32(partition->reg, TSM_STATUS_REG, &status);
		if (IS_CMD_FAIL(status)) {
			aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
			return -EFAULT;
		}
	}

	/* disable tec interrupts by default */
	zhouyi_v3_enable_interrupt(partition, false);
	dev_dbg(partition->dev, "command pool #%d was destroyed\n", partition->id);
	return 0;
}

static int zhouyi_v3_reserve(struct aipu_partition *partition, struct aipu_job_desc *udesc,
			     int trigger_type, int pool)
{
	int ret = 0;
	u32 status = 0;
	u32 intr_ctrl = 0;

	if (unlikely(!partition || !udesc))
		return -EINVAL;

	/*
	 * Tripwire: runtime resume restores this mask and
	 * disable_interrupt has no callers, so zero here means something
	 * not yet identified cleared it (power-down is not proven to be
	 * the only cause). Repair, but loudly, so it shows in dmesg.
	 */
	aipu_read32(partition->reg, CMD_POOL_INTR_CTRL_REG(partition->id), &intr_ctrl);
	if (unlikely(!intr_ctrl)) {
		dev_err(partition->dev,
			"cmd pool #%d intr mask found cleared at dispatch: re-arming\n",
			partition->id);
		zhouyi_v3_enable_interrupt(partition, false);
	}

	ret = zhouyi_v3_create_command_pool(partition);
	if (ret)
		return ret;

	aipu_read32(partition->reg, CMD_POOL_STATUS_REG(partition->id), &status);
	if (IS_CMD_POOL_IDLE(status)) {
		dev_err(partition->dev, "create command pool #%d failed (pool idle)\n",
			partition->id);
		return -EFAULT;
	}

	if (trigger_type == ZHOUYI_TRIGGER_TYPE_CREATE          ||
	    trigger_type == ZHOUYI_TRIGGER_TYPE_UPDATE_DISPATCH ||
	    trigger_type == ZHOUYI_TRIGGER_TYPE_DEBUG_DISPATCH) {
		aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_HIGH_REG, udesc->head_tcb_pa >> 32);
		aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_LOW_REG, (u32)udesc->head_tcb_pa);
	}

	dev_dbg(partition->dev, "[Job 0x%llx] scheduler: TCB head 0x%llx\n",
		udesc->job_id, udesc->head_tcb_pa);

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_INFO_REG, (u16)udesc->job_id);

	if (trigger_type == ZHOUYI_TRIGGER_TYPE_DEBUG_DISPATCH) {
		aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
			     TSM_DBG_DISPATCH_CMD_POOL(partition->id, get_qos(udesc->exec_flag),
						       udesc->core_id));
		dev_dbg(partition->dev, "debug-dispatch user job 0x%llx", udesc->job_id);
	} else {
		aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
			     TSM_DISPATCH_CMD_POOL(partition->id, get_qos(udesc->exec_flag)));
		dev_dbg(partition->dev, "dispatch user job 0x%llx (0x%llx - 0x%llx, tcbp 0x%llx)",
			udesc->job_id, udesc->head_tcb_pa, udesc->tail_tcb_pa,
			udesc->last_task_tcb_pa);
	}

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (IS_CMD_FAIL(status)) {
		dev_err(partition->dev, "dispatch command failed: job tail 0x%llx\n",
			udesc->tail_tcb_pa);
		return -EFAULT;
	}

	return ret;
}

static int zhouyi_v3_exit_dispatch(struct aipu_partition *partition, u32 job_flag, u64 tcb_pa)
{
	u32 status = 0;

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DISPATCH_CMD_POOL(partition->id, get_qos(job_flag)));

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	if (IS_CMD_FAIL(status)) {
		dev_err(partition->dev, "dispatch init TCB failed: job tail 0x%llx\n",
			tcb_pa);
		return -EFAULT;
	}

	dev_dbg(partition->dev, "dispatch init TCB head 0x%llx\n", tcb_pa);
	return 0;
}

static bool zhouyi_v3_is_idle(struct aipu_partition *partition)
{
	u32 status = 0;

	aipu_read32(partition->reg, TSM_STATUS_REG, &status);
	return !IS_CMD_POOL_FULL(status);
}

static void zhouyi_v3_print_hw_id_info(struct aipu_partition *partition)
{
	struct aipu_priv *aipu = partition->priv;
	u32 iter = 0;

	dev_info(aipu->dev, "############# ZHOUYI V3 AIPU #############");
	dev_info(aipu->dev, "# Maximum Partition Count: %d", aipu->max_partition_cnt);
	dev_info(aipu->dev, "# Maximum Command Pool Count: %d", aipu->max_cmd_pool_cnt);
	dev_info(aipu->dev, "# Enabled Partition Count: %d", aipu->partition_cnt);
	dev_info(aipu->dev, "# Enabled Cluster Count: %d", aipu->cluster_cnt);

	for (iter = 0; iter < aipu->partition_cnt; iter++) {
		partition = &aipu->partitions[iter];
		dev_info(aipu->dev, "#\n");
		dev_info(aipu->dev, "# -- Partition #%u", partition->id);
		dev_info(aipu->dev, "# Cluster Count: %d", partition->cluster_cnt);
		dev_info(aipu->dev, "# Core Count per Cluster: %d",
			 partition->clusters[0].core_cnt);
		dev_info(aipu->dev, "# TEC Count per Core: %d",
			 partition->clusters[0].tec_cnt);
	}
	dev_info(aipu->dev, "##########################################");
}

static int zhouyi_v3_io_rw(struct aipu_partition *partition, struct aipu_io_req *io_req)
{
	if (unlikely(!io_req))
		return -EINVAL;

	if (!partition || io_req->offset > ZHOUYI_V3_MAX_REG_OFFSET)
		return -EINVAL;

	/*
	 * the registers are word-sized: an odd offset reaches
	 * readl/writel on a Device-nGnRE mapping as an unaligned access --
	 * a synchronous external abort on arm64, from a 0666 node.
	 * The sysfs store path always had this check; the ioctl did not.
	 */
	if (io_req->offset & 0x3)
		return -EINVAL;

	/*
	 * nothing ever read reg_type: a caller asking for the
	 * debug window silently got the host window at the same offset.
	 * No debug window exists (the never-populated dbg_reg
	 * fields were deleted as dead weight), so refuse rather than
	 * pretend.
	 */
	if (io_req->reg_type != AIPU_HOST_REG)
		return -EINVAL;

	return zhouyi_io_rw(partition->reg, io_req);
}

static void zhouyi_v3_disable_tick_counter(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TICK_COUNTER_CONTROL_STATUS_REG, DISABLE_COUNTER);
}

static void zhouyi_v3_enable_tick_counter(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TICK_COUNTER_CONTROL_STATUS_REG, ENABLE_COUNTER);
}

static void zhouyi_v3_initialize(struct aipu_partition *partition)
{
	int iter = 0;

	for (iter = 0; iter < partition->cluster_cnt; iter++)
		zhouyi_v3_set_partition(partition, partition->clusters[iter].id);

	zhouyi_v3_config_partition_cmd_pool(partition);
	zhouyi_v3_enable_interrupt(partition, false);
	zhouyi_v3_disable_tick_counter(partition);
}

static irqreturn_t partition_upper_half(struct aipu_partition *partition)
{
	struct aipu_priv *aipu = partition->priv;
	struct job_irq_info info;
	u32 status = 0;
	u32 tag_id = 0;
	u32 val = 0;
	u32 tick_hi = 0;
	u32 tick_lo = 0;
	u16 sig_num = 0;

	/*
	 * this used to ignore its argument and rescan every
	 * partition itself (N² scans per interrupt, and the loop left
	 * `partition` pointing at the last entry when nothing matched);
	 * the caller owns the loop now. On a shared line, "nothing
	 * to handle" must be IRQ_NONE, not IRQ_HANDLED, or the kernel can
	 * neither route the interrupt to its real owner nor detect a
	 * screaming line.
	 */
	aipu_read32(&aipu->reg, CMD_POOL_INTR_STATUS_REG(partition->id), &status);
	if (!IS_IRQ_TO_HANDLE(status))
		return IRQ_NONE;

	aipu_read32(&aipu->reg, CMD_POOL_IRQ_SIG_REG(partition->id), &val);
	sig_num = GET_IRQ_SIGNAL_NUM(val);
	aipu_read32(&aipu->reg, CMD_POOL_INTR_TCB_PTR_REG(partition->id), &val);
	info.tail_tcbp = val;
	aipu_read32(&aipu->reg, CMD_POOL_IRQ_SIGNAL_FLAG_REG(partition->id), &val);
	info.sig_flag = val;
	aipu_read32(&aipu->reg, TICK_COUNTER_HIGH_REG, &tick_hi);
	aipu_read32(&aipu->reg, TICK_COUNTER_LOW_REG, &tick_lo);
	info.tick_counter = ((u64)tick_hi << 32) + tick_lo;
	info.cluster_id = GET_INTR_CLUSTER_ID(status);
	info.core_id = GET_INTR_CORE_ID(status);
	info.tec_id = GET_INTR_TEC_ID(status);

	if (IS_ABNORMAL(status)) {
		aipu_read32(partition->reg, CMD_POOL_FIRST_BAD_CMD_REG(partition->id), &tag_id);
		info.tag_id = IS_BAD_TAG_ID_VALID(tag_id) ? GET_FIRST_BAD_TAG_ID(tag_id) : 0;
		aipu_write32(partition->reg, CMD_POOL_FIRST_BAD_CMD_REG(partition->id), 0);
	} else {
		info.tag_id = 0;
	}

	if (IS_FAULT_IRQ(status)) {
		/* fault & done come in together */
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_FAULT);
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_DONE);
		status &= ~CLEAR_CMD_POOL_FAULT;
	} else if (IS_DONE_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_DONE);
	} else if (IS_ERROR_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_ERROR);
	} else if (IS_EXCEPTION_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_EXCEPTION);
	}

	if (!IS_TEC_IRQ(status) || IS_SIGNAL_IRQ(status)) {
		aipu_job_manager_irq_upper_half(partition, GET_INTR_TYPE(status), &info);
		if (IS_SIGNAL_IRQ(status)) {
			WARN_ON(sig_num != 1);
			aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
				     CLEAR_CMD_POOL_SIGNAL);
		}
		aipu_irq_schedulework(partition->irq_obj);
	}

	return IRQ_HANDLED;
}

static irqreturn_t zhouyi_v3_upper_half(void *data)
{
	struct aipu_partition *partition = (struct aipu_partition *)data;
	struct aipu_priv *aipu = partition->priv;
	irqreturn_t ret = IRQ_NONE;
	int iter = 0;

	for (iter = 0; iter < aipu->partition_cnt; iter++) {
		if (partition_upper_half(&aipu->partitions[iter]) == IRQ_HANDLED)
			ret = IRQ_HANDLED;
	}

	return ret;
}

static void zhouyi_v3_bottom_half(void *data)
{
	aipu_job_manager_irq_bottom_half(data);
}

static int zhouyi_v3_sysfs_show(struct aipu_partition *partition, char *buf, int len)
{
	int ret = 0;

	if (unlikely(!partition || !buf))
		return 0;

	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "Cluster config",
				     CLUSTER_CONFIG_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "Cluster control",
				     CLUSTER_CONTROL_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM schedule ctrl handle",
				     TSM_CMD_SCHD_CTRL_HANDLE_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM schedule ctrl info",
				     TSM_CMD_SCHD_CTRL_INFO_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM schedule addr high",
				     TSM_CMD_SCHD_ADDR_HIGH_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM schedule addr low",
				     TSM_CMD_SCHD_ADDR_LOW_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM config",
				     TSM_CONFIG_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM build info",
				     TSM_BUILD_INFO_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM status",
				     TSM_STATUS_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM soft reset",
				     TSM_SOFT_RESET_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "TSM revision",
				     TSM_REVISION_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "Tick counter high",
				     TICK_COUNTER_HIGH_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "Tick counter low",
				     TICK_COUNTER_LOW_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "Tick counter control",
				     TICK_COUNTER_CONTROL_STATUS_REG);
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "CMD pool config",
				     CMD_POOL_CONFIG_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "CMD pool status",
				     CMD_POOL_STATUS_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "CMD pool intr control",
				     CMD_POOL_INTR_CTRL_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "CMD pool intr status",
				     CMD_POOL_INTR_STATUS_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "CMD pool signal",
				     CMD_POOL_IRQ_SIG_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "CMD pool signal flag",
				     CMD_POOL_IRQ_SIGNAL_FLAG_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "CMD pool TCBP",
				     CMD_POOL_INTR_TCB_PTR_REG(0));
	ret += zhouyi_print_reg_info(partition->reg, buf, len + ret, "Debug page selection",
				     DEBUG_PAGE_SELECTION_REG);

	return ret;
}

static int zhouyi_v3_soft_reset(struct aipu_partition *partition, bool init_regs)
{
	int ret = 0;
	struct aipu_priv *aipu = partition->priv;

	/* NOTE: this is a global soft-reset, not per-partition! */
	ret = aipu->ops->global_soft_reset(aipu);
	if (ret)
		return ret;

	partition->ops->initialize(partition);
	return 0;
}

u64 get_gm_size(u32 val)
{
	u8 _val = _GET_GM_SIZE(val);

	return _val ? (SZ_1M << (_val - 1)) : 512 * SZ_1K;
}

static struct aipu_operations zhouyi_v3_ops = {
	.get_config = NULL,
	.enable_interrupt = zhouyi_v3_enable_interrupt,
	.disable_interrupt = zhouyi_v3_disable_interrupt,
	.trigger = zhouyi_v3_trigger,
	.reserve = zhouyi_v3_reserve,
	.is_idle = zhouyi_v3_is_idle,
	.print_hw_id_info = zhouyi_v3_print_hw_id_info,
	.io_rw = zhouyi_v3_io_rw,
	.upper_half = zhouyi_v3_upper_half,
	.bottom_half = zhouyi_v3_bottom_half,
	.sysfs_show = zhouyi_v3_sysfs_show,
	.soft_reset = zhouyi_v3_soft_reset,
	.initialize = zhouyi_v3_initialize,
	.destroy_command_pool = zhouyi_v3_destroy_command_pool,
	.abort_command_pool = zhouyi_v3_abort_command_pool,
	.exit_dispatch = zhouyi_v3_exit_dispatch,
	.disable_tick_counter = zhouyi_v3_disable_tick_counter,
	.enable_tick_counter = zhouyi_v3_enable_tick_counter,
	.enable_core_cnt = zhouyi_v3_enable_core_cnt,
};

struct aipu_operations *get_zhouyi_v3_ops(void)
{
	return &zhouyi_v3_ops;
}
