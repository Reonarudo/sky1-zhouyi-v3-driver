/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_MM_H__
#define __AIPU_MM_H__

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/iova.h>
#include <linux/sizes.h>
#include <linux/atomic.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <asm/cacheflush.h>
#include <armchina_aipu.h>
#include "aipu_tcb.h"
#include "zhouyi.h"

#define DEFERRED_FREE  1
#define DMA_BUF_EXEC_ID 1

/*
 * Driver-private aipu_buf_request.data_type value, above the UAPI enum
 * aipu_mm_data_type (armchina_aipu.h, whose values are kept stable). It marks a
 * request the driver builds for itself rather than one copied from
 * userspace: the dma-buf exporter's backing allocation, which is owned by
 * the exported dma_buf's refcount instead of by an fd (aipu_alloc_dma_buf).
 *
 * Only the NULL-owner tripwire in aipu_mm_v3_alloc_iova reads it, and only
 * in combination with filp == NULL, which no user-facing path can produce
 * -- so a userspace REQ_BUF naming this value gains nothing.
 */
#define AIPU_MM_DATA_TYPE_KMD_DMA_BUF	0x80

enum aipu_gm_policy {
	AIPU_GM_POLICY_NONE         = 0,
	AIPU_GM_POLICY_SHARED       = 1,
	AIPU_GM_POLICY_HALF_DIVIDED = 2,
};

enum aipu_mem_region_type {
	AIPU_MEM_REGION_TYPE_MEMORY = 0,
	AIPU_MEM_REGION_TYPE_SRAM   = 1,
	AIPU_MEM_REGION_TYPE_DTCM   = 2,
	AIPU_MEM_REGION_TYPE_GM     = 3,
	AIPU_MEM_REGION_TYPE_MAX    = 4,
};

enum aipu_mem_hold_tcb_status {
	AIPU_MEM_HOLD_TYPE_IDLE     = 0,
	AIPU_MEM_HOLD_TYPE_LINKING  = 1,
	AIPU_MEM_HOLD_TYPE_LINK_PREV = 2,
	AIPU_MEM_HOLD_TYPE_LINK_NEXT = 3,
	AIPU_MEM_HOLD_TYPE_LINKED   = 4,
	AIPU_MEM_HOLD_TYPE_MAX      = 6,
};

struct aipu_mem_region_obj;
struct aipu_job;
struct aipu_hold_tcb_buf {
	u64 head;
	int status;
	struct aipu_buf_desc desc;
	struct list_head node;
	struct aipu_tcb *hold_tcb;
	u64 prev_head;
	u64 prev_tail;
	u64 next_head;
	u64 prev_hold_tcb;
	struct aipu_mem_region *reg;
	int nums;
	int index;
	int hold_index;
	struct aipu_tcb_buf *prev_tbuf;
};

struct __cache_ops {
    void (*flush_range)(void *vaddr, size_t size);
    void (*clean_range)(void *vaddr, size_t size);
    void (*invalidate_range)(void *vaddr, size_t size);
    /*
     * flush_range_nb: same as flush_range but WITHOUT the trailing barrier,
     * for use in per-page loops that issue a single flush_barrier() at the end.
     * flush_barrier: the deferred completion barrier (e.g. dsb sy). Both may be
     * NULL on arches that have not opted in; callers must fall back to
     * flush_range (which carries its own barrier) when flush_range_nb is NULL.
     */
    void (*flush_range_nb)(void *vaddr, size_t size);
    void (*flush_barrier)(void);
    size_t cache_line_size;
};

struct aipu_phy_block {
	struct list_head list;
	struct page **pages;
	struct sg_table sgt;
	dma_addr_t dma_addr;
	u64 iova_start;
	u64 size;
	u32 page_count;
	/*
	 * Kernel alias of this block, or NULL. Created lazily, on the first
	 * kernel-side resolve of an address inside the block, by
	 * aipu_mm_v3_block_kva() under the owning buffer's phy_mutex -- NOT at
	 * allocation, and never from anything the caller said.
	 * Transitions NULL -> address exactly once and
	 * never back, published with a release store, so the unlocked readers
	 * are sound. ⚠️ aipu_map_sg_to_iommu() on the non-custom-IOVA path
	 * still assigns it directly and ignores vmap failure; that path is
	 * unreachable on a V3 part with the custom-IOVA domain and was left
	 * alone rather than dragged into this change's perimeter.
	 */
	void* va;
	struct dma_buf *dmabuf;
	bool bind_memory;
};

struct aipu_iova_buffer {
	u64 iova_start;
	u64 iova_size;
	u64 allocated_size;
	struct list_head phy_blocks;
	struct mutex phy_mutex;
	u32 ref_count;
	u8 region;
	u8 asid;
	struct file *filp;
	int tgid;
	int pid;
	struct list_head node;
	u64 exec_id;
	u32 data_type;		/* DEBUG: aipu_mm_data_type captured at alloc */
	/*
	 * Deferred-free: when set, the buffer has been logically freed by the
	 * owning fd (aipu_free_dma_iova_phy path) but its IOMMU mapping and IOVA
	 * are kept live until fd close. This defeats intra-fd use-after-unmap
	 * (DPTSW-23453 SMMU event 0x10): load-time intermediates freed by the SDK
	 * are still referenced by NPU-walked rodata/command-pool state, so tearing
	 * down their PTEs immediately lets a later job fault on the stale ref.
	 * The buffer stays on buffer_list so lookups (find_block_by_iova,
	 * iova_to_phys) keep resolving while the fd is alive; destroy happens in
	 * aipu_mm_v3_free_buffers_by_filp / aipu_mm_v3_destroy_buffer_now.
	 */
	bool deferred;
};

/**
 * struct aipu_tcb_buf - TCB buffer descriptor
 * @head: address of the list head
 * @tail: address of the list tail
 * @dep_job_id: ID of the job depends on this buffer list (if any)
 * @tail_tcb: tail of the TCB list
 * @node: list node
 * @pinned: is this buffer should be maintained after executions
 * @reg: pointer to the region contains this TCB
 */
struct aipu_tcb_buf {
	u64 head;
	u64 tail;
	int dep_job_id;
	struct aipu_tcb *tail_tcb;
	struct list_head node;
	bool pinned;
	struct aipu_mem_region *reg;
};

/**
 * struct aipu_mem_region - AIPU memory region
 * @type: region type: memory/sram/dtcm/gm
 * @base_iova: region base iova (bus address)
 * @base_pa: region base physical address
 * @base_va: region base virtual address
 * @bytes: total bytes of this region
 * @dev: device struct pointer
 * @attrs: attributes for DMA API
 * @tcb_buf_head: list head of tbuf
 * @filp: pointer to struct file requesting this region
 * @obj: pointer to the region object
 * @locked: is this region locked (therefore cannot be released) or not
 */
struct aipu_mem_region {
	enum aipu_mem_region_type type;
	dma_addr_t base_iova;
	dma_addr_t base_pa;
	void *base_va;
	u64 bytes;
	struct device *dev;
	unsigned long attrs;
	struct aipu_tcb_buf *tcb_buf_head;
	struct file *filp;
	struct aipu_mem_region_obj *obj;
	bool locked;
};

/**
 * struct aipu_mem_region_obj - object struct contains a region
 *     We link an object rather than the region directly because
 *     in some cases, a region might be linked in multiple lists.
 * @reg: pointer to a region
 * @list: list head
 */
struct aipu_mem_region_obj {
	struct aipu_mem_region *reg;
	struct list_head list;
};

/**
 * struct aipu_mem_region_list - list of allocated memory regions
 * @head: region objects
 */
struct aipu_mem_region_list {
	struct aipu_mem_region_obj *head;
};

/**
 * struct aipu_memory_manager - AIPU memory management struct (MM)
 * @version: AIPU ISA version number
 * @has_iommu: system has an IOMMU for AIPU to use or not
 * @dev: device struct pointer (AIPU core 0)
 * @lock: lock for the region list and importer bufs struct
 * @mem: list of all allocated memory regions
 * @gm_bytes: V3 GM size (in bytes)
 * @gm_policy: GM policy determined by customer (AIPU_GM_POLICY_SHARED/AIPU_GM_POLICY_HALF_DIVIDED)
 * @gm_policy_attr: GM policy sysfs attribute, for v3 only
 * @slock:   TCB buffer lock
 * @default_asid_base: ASID region 0/1 base address by default
 * @default_asid_size: ASID region 0/1 size by default
 * @obj_cache: slab cache of the region objects
 * @reg_cache: slab cache of the regions
 * @tbuf_cache: slab cache of the tcb descriptors
 * @importer_bufs: buffers from dma-buf importer(s)
 */
/**
 * struct aipu_asid_iova_info - Per-ASID IOVA management for V3
 */
struct aipu_asid_iova_info {
	u64 iova_base;		/* IOVA base for this ASID (e.g., 0, 3GB, 6GB...) */
	u64 iova_size;		/* IOVA size per ASID (3GB for V3) */
	u64 iova_used;		/* Currently used IOVA size */
	u64 asid_base;		/* NPU view base address (e.g., 0, 4GB, 8GB...) */
};

/* Zero-copy weights: host memory in 2 MiB chunks (outside the window) and window slots re-pointed at it. */
#define AIPU_WCHUNK		SZ_2M
#define AIPU_WBUF_MMAP_BASE	(1ULL << 40)	/* mmap offsets of weight buffers: base + id << 36 (above every NPU-view address) */
#define AIPU_WBUF_MMAP_SHIFT	36

struct aipu_wbuf {
	struct list_head node;
	struct file *filp;
	u64 id;
	u32 nchunks;		/* 2 MiB units */
	struct page **chunks;	/* per unit: an order-9 block, or NULL and its 512 pages in small[] */
	struct page ***small;	/* per unit: NULL, or 512 order-0 pages (no contiguous block was available) */
	u32 nbig;
	atomic_t mmaps;		/* live user mappings (vm_ops open/close) */
	u32 slot_refs;		/* slots currently mapping it (under wslot_lock) */
};

struct aipu_slot {
	struct list_head node;
	struct file *filp;
	u64 id;
	struct iova *iova;	/* the reserved range (size + one chunk of alignment slack) */
	u64 iova_start;		/* 2 MiB aligned */
	u64 bytes;		/* whole chunks */
	u64 reserved;		/* accounted in asid_iova[0].iova_used */
	u64 mapped;		/* bytes currently mapped from the slot's start */
	struct aipu_wbuf *wbuf;	/* what is mapped, or NULL */
};

struct aipu_memory_manager {
	int version;
	bool has_iommu;
	struct device *dev;
	struct mutex lock; /* Protect the region list and importer bufs struct */
	u32 valid_asid_cnt;
	struct aipu_mem_region_list mem;
	int gm_bytes;
	int gm_policy;
	struct device_attribute *gm_policy_attr;
	spinlock_t slock; /* Protect tcb_buf list */
	spinlock_t shlock; /* Protect hold tcb_buf list */
	u64 default_asid_base;
	u32 default_asid_size;
	struct kmem_cache *obj_cache;
	struct kmem_cache *reg_cache;
	struct kmem_cache *tbuf_cache;
	struct kmem_cache *hold_tbuf_cache;
	struct aipu_dma_buf_importer *importer_bufs;
	struct aipu_hold_tcb_buf *hold_tcb_head;
	u64 dma_mask;
	struct iommu_domain *iommu_domain;
	struct iova_domain iova_domain;
	struct mutex buffer_mutex;
	struct list_head buffer_list;
	u64 iova_base;
	u64 iova_size;
	u32 buffer_count;
	struct __cache_ops cache_ops;
	
	/* V3 custom IOVA support */
	bool use_v3_custom_iova;	/* Enable V3 custom IOVA mode */
	struct aipu_asid_iova_info asid_iova[ZHOUYI_ASID_COUNT];
	spinlock_t asid_lock;	/* Serialize asid_iova[].iova_used accounting */
	struct aipu_tcb_buf *v3_tcb_buf_head;	/* V3 custom IOVA TCB buffer list */
	struct list_head wbufs, slots;	/* zero-copy weights (aipu_wbuf, aipu_slot) */
	struct mutex wslot_lock;
	u64 wslot_next_id;
};

int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version);
int aipu_deinit_mm(struct aipu_memory_manager *mm);
int aipu_mm_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp);
int aipu_mm_free(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp,
		 bool unlock);
int aipu_mm_cache_flush(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf,
			struct file *filp);
int aipu_mm_cache_invalid(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf,
			  struct file *filp);
void aipu_mm_free_buffers(struct aipu_memory_manager *mm, struct file *filp);
char *aipu_mm_get_va(struct aipu_memory_manager *mm, u64 dev_pa);
int aipu_mm_mmap_buf(struct aipu_memory_manager *mm, struct vm_area_struct *vma,
		     struct file *filp);

/* V3 custom IOVA support */
int aipu_mm_v3_init_iova_domain(struct aipu_memory_manager *mm);
int aipu_wbuf_alloc(struct aipu_memory_manager *mm, struct aipu_wbuf_req *req, struct file *filp);
int aipu_wbuf_free(struct aipu_memory_manager *mm, u64 id, struct file *filp);
int aipu_wbuf_mmap(struct aipu_memory_manager *mm, struct vm_area_struct *vma, struct file *filp);
int aipu_slot_alloc(struct aipu_memory_manager *mm, struct aipu_slot_req *req, struct file *filp);
int aipu_slot_free(struct aipu_memory_manager *mm, u64 id, struct file *filp);
int aipu_slot_map(struct aipu_memory_manager *mm, struct aipu_slot_map *map, struct file *filp);
void aipu_wslot_release(struct aipu_memory_manager *mm, struct file *filp);
void aipu_mm_v3_deinit_iova_domain(struct aipu_memory_manager *mm);
struct aipu_iova_buffer *aipu_mm_v3_alloc_iova(struct aipu_memory_manager *mm,
					       struct aipu_buf_request *buf_req,
					       struct file *filp);
int aipu_mm_v3_alloc_phy_and_map(struct aipu_memory_manager *mm,
					struct aipu_iova_buffer *buffer,
					struct aipu_buf_request *buf_req);
void aipu_mm_v3_free_iova_buffer(struct aipu_memory_manager *mm,
					struct aipu_iova_buffer *buffer);
void aipu_mm_v3_destroy_buffer_now(struct aipu_memory_manager *mm,
					struct aipu_iova_buffer *buffer);
void aipu_mm_v3_free_buffers_by_filp(struct aipu_memory_manager *mm,
					struct file *filp);
struct aipu_phy_block *aipu_mm_v3_find_block_by_iova(struct aipu_memory_manager *mm,
						    u64 iova, u32 asid);
u64 aipu_mm_v3_pa_to_iova(struct aipu_memory_manager *mm, u64 dev_pa, u32 asid);
bool aipu_mm_v3_pa_owned_by(struct aipu_memory_manager *mm, u64 pa, struct file *filp);
u64 aipu_mm_v3_iova_to_pa(struct aipu_memory_manager *mm, u64 iova, u32 asid);
int aipu_mm_v3_map_imported_sgt(struct aipu_memory_manager *mm, struct sg_table *sgt,
				u32 asid, u64 *out_pa, u64 *out_size);
void aipu_mm_v3_unmap_imported_sgt(struct aipu_memory_manager *mm, u64 dev_pa,
				   u64 size, u32 asid);
u32 aipu_mm_v3_get_tcb_addr(struct aipu_memory_manager *mm, u64 dev_pa, u32 asid);
void aipu_mm_get_asid(struct aipu_memory_manager *mm, struct aipu_cap *cap);
u64 aipu_mm_get_asid_base(struct aipu_memory_manager *mm, u32 asid);
u64 aipu_mm_get_asid_size(struct aipu_memory_manager *mm, u32 asid);
u32 aipu_mm_get_asid_cnt(struct aipu_memory_manager *mm);
int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes);
int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next);
void aipu_mm_get_gm(struct aipu_memory_manager *mm, struct aipu_cap *cap);

bool is_grid_end(struct aipu_tcb *tcb);
int print_core_id(struct aipu_memory_manager *mm, u64 head, u64 tail);
struct aipu_tcb *aipu_mm_get_tcb(struct aipu_memory_manager *mm, u64 pa);
int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		     int next_job_id);
int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 prev_tail);
int aipu_mm_hold_tcb_buf_alloc(struct aipu_memory_manager *mm, struct aipu_job *kjob);
struct aipu_hold_tcb_buf *aipu_mm_get_hold_htbuf(struct aipu_memory_manager *mm, u64 hold_tcb_pa);
void aipu_mm_set_final_htbuf_index(struct aipu_memory_manager *mm, int index);
int aipu_alloc_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp);
int aipu_free_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp);
void aipu_cache_flush(struct aipu_memory_manager *mm, void *vaddr, size_t size);
void aipu_cache_clean(struct aipu_memory_manager *mm, void *vaddr, size_t size);
void aipu_cache_invalidate(struct aipu_memory_manager *mm, void *vaddr, size_t size);
struct aipu_iova_buffer *aipu_get_iova_buffer_by_job_id(struct aipu_memory_manager *mm, u64 exec_id, char* str);
int aipu_rebind_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_rebind_buf_desc *desc, struct file *filp);
#endif /* __AIPU_MM_H__ */
