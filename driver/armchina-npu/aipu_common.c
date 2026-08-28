// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>
#include "aipu_common.h"
#include "aipu_partition.h"
#ifdef CONFIG_SKY1
#include "cix_sky1_soc.h"
#endif

#define MAX_CHAR_SYSFS 4096
#define KBUF_MAX_SIZE 1024
#define ADDR_VALUE 2
#define ADDR_ONLY  1

ssize_t aipu_common_ext_register_sysfs_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	int ret = 0;
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;

	if (unlikely(!partition))
		return 0;

#ifdef CONFIG_SKY1
       sky1_npu_pm_runtime_get_sync(aipu->dev);
#endif

	/*
	 * this chain used to snprintf into a 512-byte stack
	 * buffer and strcat onto the page with no PAGE_SIZE accounting;
	 * sysfs_emit_at owns the bound now.
	 */
	ret += sysfs_emit_at(buf, ret, "----------------------------------------------\n");
	ret += sysfs_emit_at(buf, ret, "   AIPU External Register Values\n");
	ret += sysfs_emit_at(buf, ret, "----------------------------------------------\n");
	ret += sysfs_emit_at(buf, ret, "%-*s%-*s%-*s\n", 8, "Offset", 28, "Name", 10, "Value");
	ret += sysfs_emit_at(buf, ret, "----------------------------------------------\n");
	ret += partition->ops->sysfs_show(partition, buf, ret);
	ret += sysfs_emit_at(buf, ret, "----------------------------------------------\n");

#ifdef CONFIG_SKY1
       sky1_npu_pm_runtime_put(aipu->dev);
#endif
	return ret;
}

ssize_t aipu_common_ext_register_sysfs_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;
	int ret = count;
	int value[2] = { 0 };
	struct aipu_io_req io_req;

	/* the show sibling guards this; store dereferenced NULL */
	if (unlikely(!partition))
		return count;

#ifdef CONFIG_SKY1
       sky1_npu_pm_runtime_get_sync(aipu->dev);
#endif

	/*
	 * every exit below drops the runtime-PM reference through
	 * out_put: the malformed-input branches used to return -EINVAL
	 * directly, leaking a reference each until the NPU could never
	 * runtime-suspend again -- a handful of typos and the cores stay
	 * powered for the module's life.
	 */
	if (sscanf(buf, "%x %x", &value[0], &value[1]) != ADDR_VALUE) {
		dev_info(dev, "echo \"addr_hex  value_hex\" > ext_registers\n");
		ret = -EINVAL;
		goto out_put;
	}

	/*
	 * io_req is stack memory: partition_id and reg_type were
	 * left uninitialized, and io_rw now refuses reg_type != AIPU_HOST_REG
	 * -- garbage here would randomly reject root's writes.
	 */
	io_req.partition_id = 0;
	io_req.reg_type = AIPU_HOST_REG;
	io_req.rw = AIPU_IO_WRITE;
	io_req.offset = value[0];
	io_req.value = value[1];
	dev_info(dev, "[SYSFS] write 0x%x into register offset 0x%x", value[1], value[0]);
	if (io_req.offset % 4 != 0) {
		dev_err(dev, "0x%x invalid address! 4 bytes alignment!\n", value[0]);
		ret = -EINVAL;
		goto out_put;
	}
	partition->ops->io_rw(partition, &io_req);

out_put:
#ifdef CONFIG_SKY1
       sky1_npu_pm_runtime_put(aipu->dev);
#endif

	return ret;











}

struct device_attribute *aipu_common_create_attr(struct device *dev,
						 struct device_attribute **attr,
						 const char *name, int mode,
						 sysfs_show_t show, sysfs_store_t store)
{
	int ret;

	if (!dev || !attr || !name)
		return ERR_PTR(-EINVAL);

	*attr = kzalloc(sizeof(**attr), GFP_KERNEL);
	if (!*attr)
		return ERR_PTR(-ENOMEM);

	/* dynamically allocated: lockdep needs the init */
	sysfs_attr_init(&(*attr)->attr);
	(*attr)->attr.name = name;
	(*attr)->attr.mode = mode;
	(*attr)->show = show;
	(*attr)->store = store;

	/*
	 * unchecked, a failed registration left *attr pointing
	 * at an attribute that is not in sysfs, which destroy would then
	 * device_remove_file().
	 */
	ret = device_create_file(dev, *attr);
	if (ret) {
		kfree(*attr);
		*attr = NULL;
		return ERR_PTR(ret);
	}

	return *attr;
}

void aipu_common_destroy_attr(struct device *dev, struct device_attribute **attr)
{
	if (!dev || !attr || !*attr)
		return;

	device_remove_file(dev, *attr);
	kfree(*attr);
	*attr = NULL;
}

int aipu_common_init_reg_irq(struct platform_device *p_dev, struct aipu_partition *partition,
			     struct io_region *reg, struct aipu_irq_object **irq_obj)
{
	int ret = 0;
	struct resource *res = NULL;
	u64 base = 0;
	u64 size = 0;
	int irqnum = 0;

	if (!p_dev || !reg || !irq_obj)
		return -EINVAL;

	res = platform_get_resource(p_dev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&p_dev->dev, "get aipu IO region failed\n");
		return -EINVAL;
	}
	base = res->start;
	size = res->end - res->start + 1;
	dev_dbg(&p_dev->dev, "get aipu IO region: [0x%llx, 0x%llx]\n", base, res->end);

	ret = init_aipu_ioregion(reg, base, size);
	if (ret) {
		dev_err(&p_dev->dev,
			"create aipu IO region failed: base 0x%llx, size 0x%llx\n", base, size);
		return ret;
	}
	dev_dbg(&p_dev->dev, "init aipu IO region done: [0x%llx, 0x%llx]\n", base, res->end);

	irqnum = platform_get_irq(p_dev, 0);
	if (irqnum < 0) {
		dev_err(&p_dev->dev, "get aipu IRQ number failed\n");
		ret = irqnum;
		goto init_irq_fail;
	}
	dev_dbg(&p_dev->dev, "get aipu IRQ number: 0x%x\n", irqnum);

	*irq_obj = aipu_create_irq_object(&p_dev->dev, irqnum, partition, "aipu");
	if (!*irq_obj) {
		dev_err(&p_dev->dev, "create IRQ object failed: IRQ 0x%x\n", (int)res->start);
		ret = -EFAULT;
		goto init_irq_fail;
	}
	dev_dbg(&p_dev->dev, "init aipu IRQ done\n");

	/* success */
	goto finish;

init_irq_fail:
	deinit_aipu_ioregion(reg);
	reg->kern = NULL;
	reg->phys = 0;
	reg->size = 0;

finish:
	return ret;
}
