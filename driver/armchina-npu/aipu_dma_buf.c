// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/module.h>
#include "aipu_dma_buf.h"

static int aipu_dma_buf_attach(struct dma_buf *dmabuf, struct dma_buf_attachment *attach)
{
	return 0;
}

static void aipu_dma_buf_detach(struct dma_buf *dmabuf, struct dma_buf_attachment *attach)
{
}

static struct sg_table *aipu_map_dma_buf(struct dma_buf_attachment *attach,
					 enum dma_data_direction dir)
{
	struct sg_table *sgt;
	struct aipu_dma_buf_priv *priv = attach->dmabuf->priv;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	mutex_lock(&priv->lock);
	if (priv->reclaimed) {
		dev_err(priv->mm->dev, "map of a dma-buf whose memory was freed\n");
		goto err_unlock;
	}

	ret = sg_alloc_table_from_pages(sgt, priv->pages, priv->page_count,
					0, (u64)priv->page_count << PAGE_SHIFT,
					GFP_KERNEL);
	if (ret < 0) {
		dev_err(priv->mm->dev, "failed to build sgtable from pages\n");
		goto err_unlock;
	}

	ret = dma_map_sgtable(attach->dev, sgt, dir, 0);
	if (ret) {
		dev_err(priv->mm->dev, "failed to map sgtable for attached dev\n");
		sg_free_table(sgt);
		goto err_unlock;
	}
	mutex_unlock(&priv->lock);

	return sgt;

err_unlock:
	mutex_unlock(&priv->lock);
	kfree(sgt);
	return NULL;
}

static void aipu_unmap_dma_buf(struct dma_buf_attachment *attach, struct sg_table *sgt,
			       enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sgt, dir, 0);
	sg_free_table(sgt);
	kfree(sgt);
}

static int aipu_dma_buf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	int ret = 0;
	unsigned long vm_pgoff = 0;
	struct aipu_dma_buf_priv *priv = (struct aipu_dma_buf_priv *)dmabuf->priv;

	mutex_lock(&priv->lock);
	if (priv->reclaimed) {
		mutex_unlock(&priv->lock);
		return -EINVAL;
	}

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = priv->dev_pa >> PAGE_SHIFT;
	/*
	 * NULL filp: the backing buffer is owned by this export, not by any fd,
	 * and aipu_mm_v3_find_owned_buffer_by_iova matches a NULL owner only for
	 * an in-kernel caller like this one.
	 */
	ret = aipu_mm_mmap_buf(priv->mm, vma, NULL);
	vma->vm_pgoff = vm_pgoff;
	mutex_unlock(&priv->lock);
	return ret;
}

static int aipu_dma_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct aipu_dma_buf_priv *priv = (struct aipu_dma_buf_priv *)dmabuf->priv;

	mutex_lock(&priv->lock);
	if (priv->reclaimed) {
		mutex_unlock(&priv->lock);
		return -EINVAL;
	}
	map->is_iomem = false;
	map->vaddr = priv->va;
	mutex_unlock(&priv->lock);
	return 0;
}

static void aipu_dma_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
}

/* Reclaim the backing allocation. Caller holds priv->lock, or is the release
 * path, where the refcount is zero and no op can be in flight.
 */
static int aipu_dma_buf_free_backing(struct aipu_dma_buf_priv *priv)
{
	struct aipu_buf_desc buf;

	memset(&buf, 0, sizeof(buf));
	buf.pa = priv->dev_pa;
	buf.dev_offset = priv->dev_pa;
	buf.bytes = priv->bytes;
	buf.region = AIPU_BUF_REGION_DEFAULT;
	buf.asid = AIPU_BUF_ASID_0;
	buf.exec_id = 0;

	return aipu_free_dma_iova_phy(priv->mm, &buf, NULL);
}

static void aipu_dma_release(struct dma_buf *dmabuf)
{
	struct aipu_dma_buf_priv *priv = (struct aipu_dma_buf_priv *)dmabuf->priv;

	if (!priv)
		return;

	/*
	 * This hook was empty, and FREE_DMA_BUF was the only reclaim path in
	 * the driver. So a process that allocated a dma-buf and exited -- a
	 * crash, a SIGKILL, or an ordinary early return past its own cleanup --
	 * leaked the IOVA span, its IOMMU mapping and its pages for the life of
	 * the module, with asid_iova[].iova_used never decremented: the second
	 * unfixed source of "V3: ASID 0 out of IOVA space". fd close
	 * could not cover it either, because the backing buffer deliberately
	 * has no owning filp.
	 *
	 * The last reference is gone by the time we are called, so no importer,
	 * mapping or vma survives and the reclaim is unconditionally safe.
	 */
	if (!priv->reclaimed)
		aipu_dma_buf_free_backing(priv);

	mutex_destroy(&priv->lock);
	kfree(priv);
}

static struct dma_buf_ops aipu_dma_buf_ops = {
	.attach = aipu_dma_buf_attach,
	.detach = aipu_dma_buf_detach,
	.map_dma_buf = aipu_map_dma_buf,
	.unmap_dma_buf = aipu_unmap_dma_buf,
	.mmap   = aipu_dma_buf_mmap,
	.vmap   = aipu_dma_vmap,
	.vunmap = aipu_dma_vunmap,
	.release = aipu_dma_release,
};

int aipu_alloc_dma_buf(struct aipu_memory_manager *mm, struct aipu_dma_buf_request *request)
{
	int ret = 0;
	struct aipu_buf_request inter_req;
	struct aipu_dma_buf_priv *priv = NULL;
	char *va = NULL;
	struct dma_buf *dmabuf = NULL;
	struct aipu_phy_block *block = NULL;

	DEFINE_DMA_BUF_EXPORT_INFO(exp);

	if (!mm || !request || !request->bytes)
		return -EINVAL;
	memset(&inter_req, 0, sizeof(struct aipu_buf_request));
	inter_req.bytes = request->bytes;
	inter_req.align_in_page = 1;
	inter_req.region = AIPU_BUF_REGION_DEFAULT;
	inter_req.asid = AIPU_BUF_ASID_0;
	inter_req.exec_id = 0;
	inter_req.alloc_mode = AIPU_DMA_BUF_MALLOC_DEFAULT;
	/*
	 * NULL owner, deliberately: the allocation belongs to the dma_buf we
	 * are about to export, which may outlive the fd asking for it. Name
	 * ourselves so the memory manager's NULL-owner tripwire knows this
	 * caller has a reclaim path (aipu_dma_release) -- see aipu_mm.h.
	 *
	 * Naming ourselves is ALL this does. It used to double as the request
	 * for a kernel alias, which made the export path's correctness depend
	 * on a data_type travelling intact through the allocator -- and the
	 * vmap-restriction change records the near-miss where it nearly did not.
	 * The alias is now taken below, by asking for it.
	 */
	inter_req.data_type = AIPU_MM_DATA_TYPE_KMD_DMA_BUF;

	ret = aipu_alloc_dma_iova_phy(mm, &inter_req, NULL);
	if (ret) {
		dev_err(mm->dev, "alloc phy memory fail.");
		return ret;
	}

	va = aipu_mm_get_va(mm, inter_req.desc.pa);
	if (!va) {
		ret = -EFAULT;
		goto fail;
	}

	block = aipu_mm_v3_find_block_by_iova(mm,
					      aipu_mm_v3_pa_to_iova(mm, inter_req.desc.pa,
								   inter_req.asid),
					      inter_req.asid);
	if (!block) {
		dev_err(mm->dev,
			"failed to find block for dma-buf export\n");
		ret = -EFAULT;
		goto fail;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto fail;
	}
	mutex_init(&priv->lock);

	priv->mm = mm;
	priv->dev_pa = inter_req.desc.pa;
	priv->dma_pa = inter_req.desc.pa;
	priv->bytes = inter_req.desc.bytes;
	priv->va = va;
	priv->pages = block->pages;
	priv->page_count = block->page_count;

	exp.ops = &aipu_dma_buf_ops;
	exp.size = inter_req.desc.bytes;
	exp.flags = O_RDWR | O_CLOEXEC;
	exp.priv = priv;

	dmabuf = dma_buf_export(&exp);
	if (IS_ERR(dmabuf)) {
		ret = -EINVAL;
		goto fail;
	}

	request->fd = dma_buf_fd(dmabuf, exp.flags);
	if (request->fd < 0) {
		/*
		 * The export owns priv and the backing allocation from here on,
		 * so dropping the last reference reclaims both through
		 * aipu_dma_release. Returning 0 with a negative fd (as this did)
		 * leaked the dma_buf and its memory.
		 */
		ret = request->fd;
		dma_buf_put(dmabuf);
		return ret;
	}
	return 0;

fail:
	if (priv) {
		mutex_destroy(&priv->lock);
		kfree(priv);
	}
	aipu_mm_free(mm, &inter_req.desc, NULL, true);
	return ret;
}

/*
 * aipu_dma_buf_get_own - resolve an fd to a dma-buf THIS driver exported
 *
 * dma_buf_get() returns an ERR_PTR -- ERR_PTR(-EBADF) for a non-existent fd,
 * ERR_PTR(-EINVAL) for an fd that is not a dma-buf -- which every caller here
 * used to test for NULL and then dereference. Worse, a successful get was
 * enough: dmabuf->priv was cast to ours with no check of who exported it, so
 * a dma-buf fd from any other exporter on the system (DRM, v4l2, udmabuf)
 * arrived here and had that exporter's private struct reinterpreted as this
 * driver's -- read for a "pa" and "bytes" to free, and finally passed to
 * devm_kfree() against a device it was never allocated on. Both halves are
 * reachable by an unprivileged caller with any junk fd.
 *
 * Return: a referenced dma_buf exported by this driver, or an ERR_PTR.
 */
static struct dma_buf *aipu_dma_buf_get_own(int fd)
{
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return dmabuf;

	if (dmabuf->ops != &aipu_dma_buf_ops) {
		dma_buf_put(dmabuf);
		return ERR_PTR(-EINVAL);
	}

	return dmabuf;
}

int aipu_free_dma_buf(struct aipu_memory_manager *mm, int fd)
{
	int ret = 0;
	struct dma_buf *dmabuf = NULL;
	struct aipu_dma_buf_priv *priv = NULL;

	if (!mm || fd <= 0)
		return -EINVAL;

	dmabuf = aipu_dma_buf_get_own(fd);
	if (IS_ERR(dmabuf)) {
		dev_err(mm->dev, "can't get an aipu dmabuf by fd %d (%ld).\n",
			fd, PTR_ERR(dmabuf));
		return PTR_ERR(dmabuf);
	}

	priv = (struct aipu_dma_buf_priv *)dmabuf->priv;

	/*
	 * Free the memory but not the descriptor. This used to devm_kfree(priv)
	 * and free the pages on the strength of an fd number alone, dropping
	 * only the reference dma_buf_get() had just taken: every other holder --
	 * the original fd, any process it was passed to, any attached device --
	 * was left with a dma_buf whose priv was freed memory and whose pages
	 * were gone, so the next mmap/vmap/map_dma_buf on it faulted.
	 * priv now dies with the dma_buf, in aipu_dma_release.
	 */
	mutex_lock(&priv->lock);
	if (priv->reclaimed) {
		ret = -EINVAL;
	} else {
		ret = aipu_dma_buf_free_backing(priv);
		if (!ret)
			priv->reclaimed = true;
	}
	mutex_unlock(&priv->lock);

	dma_buf_put(dmabuf);
	return ret;
}

int aipu_get_dma_buf_info(struct aipu_dma_buf *dmabuf_info)
{
	struct dma_buf *dmabuf = NULL;
	struct aipu_dma_buf_priv *priv = NULL;

	if (!dmabuf_info || dmabuf_info->fd <= 0)
		return -EINVAL;

	dmabuf = aipu_dma_buf_get_own(dmabuf_info->fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	priv = (struct aipu_dma_buf_priv *)dmabuf->priv;
	mutex_lock(&priv->lock);
	if (priv->reclaimed) {
		/* the address would name memory that is no longer ours */
		mutex_unlock(&priv->lock);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}
	dmabuf_info->pa = priv->dev_pa;
	dmabuf_info->bytes = priv->bytes;
	mutex_unlock(&priv->lock);

	dma_buf_put(dmabuf);
	return 0;
}

int aipu_attach_dma_buf(struct aipu_memory_manager *mm, struct aipu_dma_buf *dmabuf_info,
			struct file *filp)
{
	struct dma_buf *dmabuf = NULL;
	struct dma_buf_attachment *attach = NULL;
	struct sg_table *table = NULL;
	struct aipu_dma_buf_importer *im_buf = NULL;
	int ret = -EINVAL;

	if (!mm || !dmabuf_info || dmabuf_info->fd <= 0)
		return -EINVAL;

	/*
	 * Import, so any exporter is legitimate here -- but all three of these
	 * return ERR_PTR, not NULL, and were tested for NULL.
	 */
	dmabuf = dma_buf_get(dmabuf_info->fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	attach = dma_buf_attach(dmabuf, mm->dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		goto err_put;
	}

	table = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(table)) {
		ret = PTR_ERR(table);
		goto err_detach;
	}

	im_buf = devm_kzalloc(mm->dev, sizeof(*im_buf), GFP_KERNEL);
	if (!im_buf) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	if (mm->use_v3_custom_iova) {
		/*
		 * V3 custom IOVA domain is active: sg_dma_address() would return
		 * an IOVA from the kernel's default DMA-IOMMU cookie, which does
		 * not fall inside an ASID's usable window and the NPU cannot
		 * translate it. Re-map the imported pages into the custom domain
		 * (default context = ASID 0) so the NPU sees a valid address.
		 */
		u64 pa = 0, size = 0;

		ret = aipu_mm_v3_map_imported_sgt(mm, table, 0, &pa, &size);
		if (ret)
			goto err_free;
		dmabuf_info->pa = pa;
		dmabuf_info->bytes = size;
		im_buf->dev_pa = pa;
		im_buf->map_size = size;
		im_buf->asid = 0;
	} else {
		dmabuf_info->pa = sg_dma_address(table->sgl);
		dmabuf_info->bytes = sg_dma_len(table->sgl);
		im_buf->map_size = 0;
	}

	im_buf->fd = dmabuf_info->fd;
	im_buf->attach = attach;
	im_buf->table = table;
	/* Keep our own dmabuf ref for the attachment's lifetime (released in
	 * aipu_importer_teardown_locked); do NOT dma_buf_put() here.
	 */
	im_buf->dmabuf = dmabuf;
	im_buf->filp = filp;
	mutex_lock(&mm->lock);
	list_add(&im_buf->node, &mm->importer_bufs->node);
	mutex_unlock(&mm->lock);

	return 0;

err_free:
	devm_kfree(mm->dev, im_buf);
err_unmap:
	dma_buf_unmap_attachment(attach, table, DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(dmabuf, attach);
err_put:
	dma_buf_put(dmabuf);
	return ret;
}

/* Detach and free one importer entry. Caller must hold mm->lock. */
static void aipu_importer_teardown_locked(struct aipu_memory_manager *mm,
					  struct aipu_dma_buf_importer *im_buf)
{
	if (im_buf->map_size)
		aipu_mm_v3_unmap_imported_sgt(mm, im_buf->dev_pa,
					      im_buf->map_size, im_buf->asid);
	dma_buf_unmap_attachment(im_buf->attach, im_buf->table, DMA_BIDIRECTIONAL);
	dma_buf_detach(im_buf->dmabuf, im_buf->attach);
	list_del(&im_buf->node);
	dma_buf_put(im_buf->dmabuf);
	devm_kfree(mm->dev, im_buf);
}

int aipu_detach_dma_buf(struct aipu_memory_manager *mm, int fd)
{
	struct aipu_dma_buf_importer *im_buf = NULL;
	struct aipu_dma_buf_importer *next = NULL;
	int ret = -EINVAL;

	if (!mm || fd <= 0)
		return -EINVAL;

	mutex_lock(&mm->lock);
	list_for_each_entry_safe(im_buf, next, &mm->importer_bufs->node, node) {
		if (fd == im_buf->fd) {
			aipu_importer_teardown_locked(mm, im_buf);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&mm->lock);

	return ret;
}

/*
 * aipu_detach_dma_buf_by_filp - reclaim imports an owner never detached
 *
 * Called from aipu_release() (the fd-close path, which always runs on process
 * exit). Without this, an attach without a matching detach leaked the import's
 * custom-IOVA forever and inflated asid_iova[].iova_used, eventually starving
 * normal allocations with "V3: ASID 0 out of IOVA space".
 */
void aipu_detach_dma_buf_by_filp(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_dma_buf_importer *im_buf = NULL;
	struct aipu_dma_buf_importer *next = NULL;

	if (!mm || !mm->importer_bufs)
		return;

	mutex_lock(&mm->lock);
	list_for_each_entry_safe(im_buf, next, &mm->importer_bufs->node, node) {
		if (im_buf->filp == filp)
			aipu_importer_teardown_locked(mm, im_buf);
	}
	mutex_unlock(&mm->lock);
}

MODULE_IMPORT_NS("DMA_BUF");
