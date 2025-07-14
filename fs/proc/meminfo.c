// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/percpu.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/atomic.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif
#include <asm/page.h>
#include <asm/pgtable.h>
#include "internal.h"

void __attribute__((weak)) arch_report_meminfo(struct seq_file *m)
{
}

static void show_val_kb(struct seq_file *m, const char *s, unsigned long num)
{
	seq_put_decimal_ull_width(m, s, num << (PAGE_SHIFT - 10), 8);
	seq_write(m, " kB\n", 4);
}

static int meminfo_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	unsigned long committed;
	long cached;
	long available;
	unsigned long pages[NR_LRU_LISTS];
	unsigned long sreclaimable, sunreclaim;
	int lru;
	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);

	cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_node_page_state(NR_LRU_BASE + lru);

	available = si_mem_available();
	sreclaimable = global_node_page_state(NR_SLAB_RECLAIMABLE);
	sunreclaim = global_node_page_state(NR_SLAB_UNRECLAIMABLE);

	show_val_kb(m, "MemTotal:       ", i.totalram);
	show_val_kb(m, "MemFree:        ", i.freeram);
	show_val_kb(m, "MemAvailable:   ", available);
	show_val_kb(m, "Buffers:        ", i.bufferram);
	show_val_kb(m, "Cached:         ", cached);
	show_val_kb(m, "SwapCached:     ", total_swapcache_pages());
	show_val_kb(m, "Active:         ", pages[LRU_ACTIVE_ANON] +
					   pages[LRU_ACTIVE_FILE]);
	show_val_kb(m, "Inactive:       ", pages[LRU_INACTIVE_ANON] +
					   pages[LRU_INACTIVE_FILE]);
	show_val_kb(m, "Active(anon):   ", pages[LRU_ACTIVE_ANON]);
	show_val_kb(m, "Inactive(anon): ", pages[LRU_INACTIVE_ANON]);
	show_val_kb(m, "Active(file):   ", pages[LRU_ACTIVE_FILE]);
	show_val_kb(m, "Inactive(file): ", pages[LRU_INACTIVE_FILE]);
	show_val_kb(m, "Unevictable:    ", pages[LRU_UNEVICTABLE]);
	show_val_kb(m, "Mlocked:        ", global_zone_page_state(NR_MLOCK));

#ifdef CONFIG_HIGHMEM
	show_val_kb(m, "HighTotal:      ", i.totalhigh);
	show_val_kb(m, "HighFree:       ", i.freehigh);
	show_val_kb(m, "LowTotal:       ", i.totalram - i.totalhigh);
	show_val_kb(m, "LowFree:        ", i.freeram - i.freehigh);
#endif

#ifndef CONFIG_MMU
	show_val_kb(m, "MmapCopy:       ",
		    (unsigned long)atomic_long_read(&mmap_pages_allocated));
#endif

	show_val_kb(m, "SwapTotal:      ", i.totalswap);
	show_val_kb(m, "SwapFree:       ", i.freeswap);
	show_val_kb(m, "Dirty:          ",
		    global_node_page_state(NR_FILE_DIRTY));
	show_val_kb(m, "Writeback:      ",
		    global_node_page_state(NR_WRITEBACK));
	show_val_kb(m, "AnonPages:      ",
		    global_node_page_state(NR_ANON_MAPPED));
	show_val_kb(m, "Mapped:         ",
		    global_node_page_state(NR_FILE_MAPPED));
	show_val_kb(m, "Shmem:          ", i.sharedram);
	show_val_kb(m, "KReclaimable:   ", sreclaimable +
		    global_node_page_state(NR_KERNEL_MISC_RECLAIMABLE));
	show_val_kb(m, "Slab:           ", sreclaimable + sunreclaim);
	show_val_kb(m, "SReclaimable:   ", sreclaimable);
	show_val_kb(m, "SUnreclaim:     ", sunreclaim);
	seq_printf(m, "KernelStack:    %8lu kB\n",
		   global_zone_page_state(NR_KERNEL_STACK_KB));
#ifdef CONFIG_SHADOW_CALL_STACK
	seq_printf(m, "ShadowCallStack:%8lu kB\n",
		   global_zone_page_state(NR_KERNEL_SCS_BYTES) / 1024);
#endif
	show_val_kb(m, "PageTables:     ",
		    global_zone_page_state(NR_PAGETABLE));
#ifdef CONFIG_QUICKLIST
	show_val_kb(m, "Quicklists:     ", quicklist_total_size());
#endif

	show_val_kb(m, "NFS_Unstable:   ",
		    global_node_page_state(NR_UNSTABLE_NFS));
	show_val_kb(m, "Bounce:         ",
		    global_zone_page_state(NR_BOUNCE));
	show_val_kb(m, "WritebackTmp:   ",
		    global_node_page_state(NR_WRITEBACK_TEMP));
	show_val_kb(m, "CommitLimit:    ", vm_commit_limit());
	show_val_kb(m, "Committed_AS:   ", committed);
	seq_printf(m, "VmallocTotal:   %8lu kB\n",
		   (unsigned long)VMALLOC_TOTAL >> 10);
	show_val_kb(m, "VmallocUsed:    ", vmalloc_nr_pages());
	show_val_kb(m, "VmallocChunk:   ", 0ul);
	show_val_kb(m, "Percpu:         ", pcpu_nr_pages());

#ifdef CONFIG_MEMORY_FAILURE
	seq_printf(m, "HardwareCorrupted: %5lu kB\n",
		   atomic_long_read(&num_poisoned_pages) << (PAGE_SHIFT - 10));
#endif

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	show_val_kb(m, "AnonHugePages:  ",
		    global_node_page_state(NR_ANON_THPS) * HPAGE_PMD_NR);
	show_val_kb(m, "ShmemHugePages: ",
		    global_node_page_state(NR_SHMEM_THPS) * HPAGE_PMD_NR);
	show_val_kb(m, "ShmemPmdMapped: ",
		    global_node_page_state(NR_SHMEM_PMDMAPPED) * HPAGE_PMD_NR);
#endif

#ifdef CONFIG_CMA
	show_val_kb(m, "CmaTotal:       ", totalcma_pages);
	show_val_kb(m, "CmaFree:        ",
		    global_zone_page_state(NR_FREE_CMA_PAGES));
#endif

	hugetlb_report_meminfo(m);

	arch_report_meminfo(m);

	return 0;
}

static int meminfo_quick_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	long cached;
	long available;
	unsigned long pages[NR_LRU_LISTS];
	unsigned long sreclaimable, sunreclaim;
	int lru;
	unsigned long reserved = 0;



#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);
	cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_node_page_state(NR_LRU_BASE + lru);

	available = si_mem_available();
	sreclaimable = global_node_page_state(NR_SLAB_RECLAIMABLE);
	sunreclaim = global_node_page_state(NR_SLAB_UNRECLAIMABLE);


	seq_printf(m,
		"%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu "
		"%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu "
		"%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu "
		"%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu "
		"%lu %lu %lu %lu %lu %lu %lu %lu %lu\n",
		K(i.totalram),
		K(i.freeram),
		K(i.bufferram),
		K(cached),
		K(i.sharedram),
		/* upper are compatitable for old version */
		/* 5: swap total */
		K(i.totalswap),
		/* 6: swap free */
		K(i.freeswap),
		/* 7: anon pages */
		K(global_node_page_state(NR_ANON_MAPPED)),
		/* 8: file mapped pages */
		K(global_node_page_state(NR_FILE_MAPPED)),
		/* 9: slab reclaimable */
		K(sreclaimable),
		/* 10: slab unreclaimable */
		K(sunreclaim),
		/* 11: kernel stack */
		global_zone_page_state(NR_KERNEL_STACK_KB),
		/* 12: page tables */
		K(global_zone_page_state(NR_PAGETABLE)),

		/* 13: ion free */
		reserved,
		/* 14: gpu free */
		reserved,
		/* 15: zcache total */
		reserved,

		/* 16: reserved */
		reserved,
		/* 17: reserved */
		reserved,
		/* 18: reserved */
		reserved,
		/* 19: reserved */
		reserved,
		/* 20: reserved */
		reserved,
		/* 21: reserved */
		reserved,
		/* 22: reserved */
		reserved,
		/* 23: reserved */
		reserved,
		/* 24: reserved */
		reserved,
		/* 25: MemAvailable */
		K(available),
		/* 26: SwapCached */
		K(total_swapcache_pages()),
		/* 27: Active */
		K(pages[LRU_ACTIVE_ANON] + pages[LRU_ACTIVE_FILE]),
		/* 28: Inactive */
		K(pages[LRU_INACTIVE_ANON] + pages[LRU_INACTIVE_FILE]),
		/* 29: Active(anon) */
		K(pages[LRU_ACTIVE_ANON]),
		/* 30: Inactive(anon) */
		K(pages[LRU_INACTIVE_ANON]),
		/* 31: Active(file) */
		K(pages[LRU_ACTIVE_FILE]),
		/* 32: Inactive(file) */
		K(pages[LRU_INACTIVE_FILE]),
		/* 33: Unevictable */
		K(pages[LRU_UNEVICTABLE]),
		/* 34: Mlocked */
		K(global_zone_page_state(NR_MLOCK)),
		/* 35: Dirty */
		K(global_node_page_state(NR_FILE_DIRTY)),
		/* 36: Writeback */
		K(global_node_page_state(NR_WRITEBACK)),
		/* 37: KReclaimable */
		K(sreclaimable + global_node_page_state(NR_KERNEL_MISC_RECLAIMABLE)),
		/* 38: Slab */
		K(sreclaimable + sunreclaim),
#ifdef CONFIG_SHADOW_CALL_STACK
		/* 39: ShadowCallStack */
		K(global_zone_page_state(NR_KERNEL_SCS_BYTES)),
#else
		/* 39: ShadowCallStack */
		reserved,
#endif

#ifdef CONFIG_FCACHE_CTRL
		/* 46: Vital */
		K(pages[LRU_VITAL]),
		/* 47: Staple */
		K(pages[LRU_STAPLE]),
#else
		/* 46: Vital */
		reserved,
		/* 47: Staple */
		reserved,
#endif
		/* 48: reserved */
		reserved
		);
	return 0;

#undef K
}

static int meminfo_quick_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_quick_proc_show, NULL);
}

static const struct file_operations meminfo_quick_proc_fops = {
	.open		= meminfo_quick_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if defined(CONFIG_RMS_ION_STAT)
static int rms_version_proc_show(struct seq_file *m, void *v)
{
	int version = 1001;
	int other1 = 0;
	int other2 = 0;
	seq_printf(m, "%d %d %d\n", version, other1, other2);
	return 0;
}

static int rms_version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rms_version_proc_show, NULL);
}

static const struct file_operations rms_version_proc_fops = {
	.open		= rms_version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static int __init proc_meminfo_init(void)
{
	proc_create_single("meminfo", 0, NULL, meminfo_proc_show);
	proc_create("meminfo_quick", S_IRUGO, NULL, &meminfo_quick_proc_fops);
#if defined(CONFIG_RMS_ION_STAT)
	proc_create("meminfo_version", 0444, NULL, &rms_version_proc_fops);
#endif
	return 0;
}
fs_initcall(proc_meminfo_init);
