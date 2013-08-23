/* linux/drivers/char/exynos_mem.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/errno.h>	/* error codes */
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/highmem.h>
#include <linux/dma-mapping.h>
#include <linux/exynos_mem.h>
#include <linux/cma.h>

#include <asm/cacheflush.h>

#include <plat/cpu.h>

#define L2_FLUSH_ALL	SZ_1M
#define L1_FLUSH_ALL	SZ_64K

/* limit the exynos-mem mapping region, to fix exynos4 memory security issue */
struct exynos_mem_region {
	const char * name;
	dma_addr_t start;
	dma_addr_t end;
	size_t  size;
};

static bool gIsExynosMemInit = false;

static struct exynos_mem_region available_regions[] = {
	{
		.name = "s3c-fimc.0",
		.size = 0,
	}, {
		.name = "s3c-fimc.1",
		.size = 0,
	}, {
		.name = "s3c-fimc.2",
		.size = 0,
	}, {
		.name = "s3c-fimc.3",
		.size = 0,
	}, {
		.name = NULL,
	},
};

static void init_exynos_mem_region(struct exynos_mem_region *reserved_regions)
{
	struct exynos_mem_region *mem_reg;
	struct device dev;
	struct cma_info mem_info;
	int err;

	if(gIsExynosMemInit)
		return;

	for(mem_reg = reserved_regions; mem_reg->name != NULL; mem_reg++) {
		dev.init_name = mem_reg->name;
		err = cma_info(&mem_info, &dev, 0);
		if (err) {
			printk(KERN_ERR "%s: get dev: %s cma info failed\n", __func__, dev.init_name);
			mem_reg->start = 0;
			mem_reg->end = 0;
			mem_reg->size = 0;
		} else {
			mem_reg->start = mem_info.lower_bound;
			mem_reg->end = mem_info.upper_bound;
			mem_reg->size = mem_info.total_size;
			printk(KERN_DEBUG "%s: get dev: %s cma, start_addr:0x%x, end_addr:0x%x, size:%d\n",
				 __func__, mem_reg->name, mem_reg->start, mem_reg->end, mem_reg->size);
		}
	}
	gIsExynosMemInit = true;
}

static int is_exynos_mem_available(struct vm_area_struct *vma)
{
	struct exynos_mem_region *mem_reg;
	dma_addr_t start = vma->vm_pgoff << PAGE_SHIFT;
	size_t size = vma->vm_end - vma->vm_start;
	dma_addr_t end = start + size;
	int available = -EINVAL;

	printk(KERN_DEBUG "%s, start 0x%x, end 0x%x, size:%d, vm_start:0x%lx, vm_end:0x%lx\n",
		__func__, start, end, size, vma->vm_start, vma->vm_end);

	if(!gIsExynosMemInit)
		init_exynos_mem_region(available_regions);

	for(mem_reg = available_regions; mem_reg->name != NULL; mem_reg++) {
		if(mem_reg->size == 0)
			continue;
		if (start >= mem_reg->start &&
		    end <= mem_reg->end) {
			available = 0;
			break;
		}
	}

	return available;
}
/* end */

struct exynos_mem {
	bool cacheable;
};

int exynos_mem_open(struct inode *inode, struct file *filp)
{
	struct exynos_mem *prv_data;

	prv_data = kzalloc(sizeof(struct exynos_mem), GFP_KERNEL);
	if (!prv_data) {
		pr_err("%s: not enough memory\n", __func__);
		return -ENOMEM;
	}

	prv_data->cacheable = true;	/* Default: cacheable */

	filp->private_data = prv_data;

	printk(KERN_DEBUG "[%s:%d] private_data(0x%08x)\n",
		__func__, __LINE__, (u32)prv_data);

	return 0;
}

int exynos_mem_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "[%s:%d] private_data(0x%08x)\n",
		__func__, __LINE__, (u32)filp->private_data);

	kfree(filp->private_data);

	return 0;
}

enum cacheop { EM_CLEAN, EM_INV, EM_FLUSH };

static void cache_maint_inner(void *vaddr, size_t size, enum cacheop op)
{
	switch (op) {
		case EM_CLEAN:
			dmac_map_area(vaddr, size, DMA_TO_DEVICE);
			break;
		case EM_INV:
			dmac_unmap_area(vaddr, size, DMA_TO_DEVICE);
			break;
		case EM_FLUSH:
			dmac_flush_range(vaddr, vaddr + size);
	}
}

static void cache_maint_phys(phys_addr_t start, size_t length, enum cacheop op)
{
	size_t left = length;
	phys_addr_t begin = start;

	if (!soc_is_exynos5250() && !soc_is_exynos5210()) {
		if (length > (size_t) L1_FLUSH_ALL) {
			flush_cache_all();
			smp_call_function(
					(smp_call_func_t)__cpuc_flush_kern_all,
					NULL, 1);

			goto outer_cache_ops;
		}
	}

#ifdef CONFIG_HIGHMEM
	do {
		size_t len;
		struct page *page;
		void *vaddr;
		off_t offset;

		page = phys_to_page(start);
		offset = offset_in_page(start);
		len = PAGE_SIZE - offset;

		if (left < len)
			len = left;

		if (PageHighMem(page)) {
			vaddr = kmap(page);
			cache_maint_inner(vaddr + offset, len, op);
			kunmap(page);
		} else {
			vaddr = page_address(page) + offset;
			cache_maint_inner(vaddr, len, op);
		}
		left -= len;
		start += len;
	} while (left);
#else
	cache_maint_inner(phys_to_virt(begin), left, op);
#endif

outer_cache_ops:
	switch (op) {
	case EM_CLEAN:
		outer_clean_range(begin, begin + length);
		break;
	case EM_INV:
		if (length <= L2_FLUSH_ALL) {
			outer_inv_range(begin, begin + length);
			break;
		}
		/* else FALL THROUGH */
	case EM_FLUSH:
		outer_flush_range(begin, begin + length);
		break;
	}
}

long exynos_mem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case EXYNOS_MEM_SET_CACHEABLE:
	{
		struct exynos_mem *mem = filp->private_data;
		int cacheable;
		if (get_user(cacheable, (u32 __user *)arg)) {
			pr_err("[%s:%d] err: EXYNOS_MEM_SET_CACHEABLE\n",
				__func__, __LINE__);
			return -EFAULT;
		}
		mem->cacheable = cacheable;
		break;
	}

	case EXYNOS_MEM_PADDR_CACHE_FLUSH:
	{
		struct exynos_mem_flush_range range;
		if (copy_from_user(&range,
				   (struct exynos_mem_flush_range __user *)arg,
				   sizeof(range))) {
			pr_err("[%s:%d] err: EXYNOS_MEM_PADDR_CACHE_FLUSH\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		cache_maint_phys(range.start, range.length, EM_FLUSH);
		break;
	}
	case EXYNOS_MEM_PADDR_CACHE_CLEAN:
	{
		struct exynos_mem_flush_range range;
		if (copy_from_user(&range,
				   (struct exynos_mem_flush_range __user *)arg,
				   sizeof(range))) {
			pr_err("[%s:%d] err: EXYNOS_MEM_PADDR_CACHE_FLUSH\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		cache_maint_phys(range.start, range.length, EM_CLEAN);
		break;
	}

	default:
		pr_err("[%s:%d] error command\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

static void exynos_mem_mmap_open(struct vm_area_struct *vma)
{
	pr_debug("[%s] addr(0x%08x)\n", __func__, (u32)vma->vm_start);
}

static void exynos_mem_mmap_close(struct vm_area_struct *vma)
{
	pr_debug("[%s] addr(0x%08x)\n", __func__, (u32)vma->vm_start);
}

static struct vm_operations_struct exynos_mem_ops = {
	.open	= exynos_mem_mmap_open,
	.close	= exynos_mem_mmap_close,
};

int exynos_mem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct exynos_mem *mem = (struct exynos_mem *)filp->private_data;
	bool cacheable = mem->cacheable;
	dma_addr_t start = vma->vm_pgoff << PAGE_SHIFT;
	u32 pfn = vma->vm_pgoff;
	u32 size = vma->vm_end - vma->vm_start;

	/* TODO: currently lowmem is only avaiable */
	if ((phys_to_virt(start) < (void *)PAGE_OFFSET) ||
	    (phys_to_virt(start) >= high_memory)) {
		pr_err("[%s] invalid paddr(0x%08x)\n", __func__, start);
		return -EINVAL;
	}

	/* limit the exynos-mem mapping region, to fix exynos4 memory security issue */
	if(is_exynos_mem_available(vma)) {
		pr_err("[%s] invalid paddr(0x%08x)\n", __func__, start);
		return -EINVAL;
	}
	/* end */

	if (!cacheable)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_RESERVED;
	vma->vm_ops = &exynos_mem_ops;

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		pr_err("writable mapping must be shared\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		pr_err("mmap fail\n");
		return -EINVAL;
	}

	vma->vm_ops->open(vma);

	return 0;
}
