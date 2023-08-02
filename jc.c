/*
 *  Johnny Cache
 *
 *  Logic of the code:
 *
 *
 *  The static policy is simple, it uses 3 functions:
 *   build_page_list(); 							// Builds the list of available pages
 *   set_dequeue_hook(minimize_conflicts);	// Called when allocating a page
 *   set_enqueue_hook(enqueue_freed_page);   // Called when freeing a page
 *
 *   minimize_conflict calls reserve_page that increases the heat of a bucket (atomic_add...)
 *   enqueue_freed_page calls decreases the heat of the bucket of the freed page
 *
 *   Both functions call move_pages_contention() to maintain the list of pages ordered by heat
 *
 *  None of these functions are efficient, but they do the job.
 *
 *
 *
 *  For the dynamic policy, we tried many implementations. None really worked (we have a paragraph
 *  discussing that in the paper). The current implementation is the following:
 *		ENABLE_SAMPLING and ENABLE_PERIODIC_MIGRATIONS need to be set to 1
 *
 *    In init_module, we set up the sampling. Whenever a memory access is found, pebs_sample is called.
 *    pebs_sample calls add_sample that increases the heat of pages & cache slots.
 *
 *    Currently the implementation relies on a ring buffer that stores the last N samples. When a page
 *    is sampled, its heat increases and its address is placed in the ring. While doing so, the sampled
 *    page may replace another page, whose heat will be decreased. So basically we maintain a "moving
 *    average" of the heat of pages, over the last N samples.
 *
 *  Periodically, a timer is triggered. It calls periodic_migrations() that migrates a page in case of conflict.
 *
 *
 *  AGAIN, keep in mind that the dynamic migrations DO NOT WORK WELL. As we describe in the paper, configuring
 *  dynamic migrations is very finicky. You'll need to adjust the sampling rate, etc. to get it to work.
 *
 *
 *
 * /!\ IMPORTANT /!\
 *	Don't forget to set CACHE_SIZE to the size of your DRAM cache.
 *	Experiments were done on a machine with 128GB of RAM (64GB per NUMA node), so our cache is 48GB (per NUMA node).
 *
 */
#include <linux/module.h>	/* Needed by all modules */
#include <linux/moduleparam.h>
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/mm.h>
#include <linux/rmap.h>
#include <linux/migrate.h>
#include <linux/hugetlb.h>
#include <linux/perf_event.h>
#include <asm-generic/io.h>
#include <linux/sort.h>
#include <linux/delay.h>

uint64_t total_samples = 0;
uint64_t total_samples_found = 0;
uint64_t *pfns = NULL;
size_t pfn_idx = 0;

#define ENABLE_SAMPLING 0 /* Activate dynamic conflict avoidance? */
#define ENABLE_PERIODIC_MIGRATIONS 0 /* Should also be 1 for the dynamic policy to work */


/*
 * Some helper functions to manipulate pages & bitmasks, copy/pasted from the kernel.
 */
/* Return a pointer to the bitmap storing bits affecting a block of pages */
static inline unsigned long *get_pageblock_bitmap(const struct page *page,
							unsigned long pfn)
{
#ifdef CONFIG_SPARSEMEM
	return section_to_usemap(__pfn_to_section(pfn));
#else
	return page_zone(page)->pageblock_flags;
#endif /* CONFIG_SPARSEMEM */
}

static inline int pfn_to_bitidx(const struct page *page, unsigned long pfn)
{
#ifdef CONFIG_SPARSEMEM
	pfn &= (PAGES_PER_SECTION-1);
#else
	pfn = pfn - round_down(page_zone(page)->zone_start_pfn, pageblock_nr_pages);
#endif /* CONFIG_SPARSEMEM */
	return (pfn >> pageblock_order) * NR_PAGEBLOCK_BITS;
}

static __always_inline
unsigned long __get_pfnblock_flags_mask(const struct page *page,
					unsigned long pfn,
					unsigned long mask)
{
	unsigned long *bitmap;
	unsigned long bitidx, word_bitidx;
	unsigned long word;

	bitmap = get_pageblock_bitmap(page, pfn);
	bitidx = pfn_to_bitidx(page, pfn);
	word_bitidx = bitidx / BITS_PER_LONG;
	bitidx &= (BITS_PER_LONG-1);

	word = bitmap[word_bitidx];
	return (word >> bitidx) & mask;
}

unsigned long get_pfnblock_flags_mask(const struct page *page,
					unsigned long pfn, unsigned long mask)
{
	return __get_pfnblock_flags_mask(page, pfn, mask);
}

struct migration_target_control {
   pid_t pid;
};


/*
 * Actual code of JC
 */
static int max_conflicts = 0; // module parameter, maximize conflicts or minize them?
#define CACHE_SIZE (48*512) // DRAM is 64GB -> cache is 48GB on our machine -> 48*512 2MB pages
#define RING_SIZE (1000000) // Compute stats on the last XX memory access samples
#define CONTENTION_LEVELS 20 // Number of contention levels (to be in level N, a page must have been accessed heat_to_contention_level(N) times)
#define PROBLEMATIC_CONTENTION 10 // If a page has been accessed more than that, it is hot
#define MAX_MIGRATED_PAGES 100 // Max number of migrations per period

#define LARGE_PAGE_SIZE (2LU*1024*1024) // 2MB
#define pfn_to_large_pfn(pfn) ((pfn)/(512)) // 512 4KB pages in a 2MB page
#define pfn_to_bucket(pfn) (pfn_to_large_pfn(pfn) % CACHE_SIZE)

#define HEAT_PRINT_SIZE 2048
static char *printbuf;
static int total_allocated_pages = 0;

/* How hot is a cache region? */
static atomic_t heatmap[CACHE_SIZE];

/* Ring buffer of samples */
static struct ring {
   atomic_t idx;
   struct sample {
      int weight;
      u64 pfn;
   } samples[RING_SIZE];
} ring;

/* Metadata to help page allocations */
struct slot_list {
   size_t bucket;
   struct page *page;
   int heat;
   size_t used_by; // tgid
   size_t virt_addr;
   struct list_head contention_list;
   struct list_head bin_list;
};
static struct list_head unused_pages[CONTENTION_LEVELS]; // heat -> pages
static struct list_head allocated_pages[CONTENTION_LEVELS]; // heat -> pages
static struct list_head heated_pages[CONTENTION_LEVELS]; // heat -> pages
static struct list_head cache_bins[CACHE_SIZE]; // cache slot -> pages
static spinlock_t lock; // list_heads are not thread safe...

/* Metadata to help page migration */
struct pid_pages {
   struct pid_pages *next;
   int nb_pages;
   int nb_max_pages;
   pid_t tgid;
   unsigned long *addresses;
   struct page **pages;
};
static struct pages_container {
   int nb_pid;
   struct pid_pages *pids;
} pages_to_migrate;


/* PEBS configuration */
#define PEBS_SAMPLE_TYPE PERF_SAMPLE_IP | PERF_SAMPLE_TID | PERF_SAMPLE_ADDR
#define PMEM_READ 0x80d1
#define DRAM_READ 0x20d1
#define STORE_ALL 0x82d0
static struct perf_event **events;
static size_t configs[] = { DRAM_READ, PMEM_READ, STORE_ALL };
//static size_t configs[] = { PMEM_READ };

/* Timer */
struct task_struct *kthread;





/*
 * Maintain heatmap
 * Maintain list of allocated and free pages at the correct contention level.
 * The current implementation maintains a ring buffer of samples, and updates the heatmap.
 *
 * E.g.,
 *  ring = [ pageX, pageY (current idx), pageZ ]
 *  When receiving a new sample, say on pageW, then
 *  ring = [ pageX, pageY, pageW (current idx) ]
 *  and the heat of pageZ is decreased and the heat of pageW is increased.
 *  The functions also update the heat of the cache slots.
 */
static size_t heat_to_contention_level(int heat) {
	/* Various definitions of contention */
   size_t idx = 0;
   //while(heat) {
   //heat /= 2;
   //idx++;
   //}
   idx = heat / 50;
   if(idx >= CONTENTION_LEVELS)
      idx = CONTENTION_LEVELS - 1;
   return idx;
}

/* Make sure all pages are in the correct contention list so that we can choose one from the lowest */
static void move_pages_contention(size_t bucket, int heat) {
   struct slot_list *s;
   size_t idx = heat_to_contention_level(heat);

   spin_lock(&lock);
   list_for_each_entry(s, &cache_bins[bucket], bin_list) {
      if(s->used_by && s->heat)
         list_move(&s->contention_list, &heated_pages[idx]);
      else if(s->used_by)
         list_move(&s->contention_list, &allocated_pages[idx]);
      else
         list_move(&s->contention_list, &unused_pages[idx]);
   }
   spin_unlock(&lock);
}

/* add_sample is a generic function to increaset the head of a page */
static void add_sample(u64 pfn, int weight) {
   size_t bucket = pfn_to_bucket(pfn);
   int idx = atomic_inc_return(&ring.idx) - 1;

   //return;

   if(idx >= RING_SIZE) {
      int old_weight = ring.samples[idx % RING_SIZE].weight;
      int old_pfn = ring.samples[idx % RING_SIZE].pfn;
      size_t old_bucket = pfn_to_bucket(old_pfn);
      atomic_sub(old_weight, &heatmap[old_bucket]);
      move_pages_contention(old_bucket, atomic_read(&heatmap[old_bucket]));
   }

   ring.samples[idx % RING_SIZE].pfn = pfn;
   ring.samples[idx % RING_SIZE].weight = weight;
   atomic_add(weight, &heatmap[bucket]);
   move_pages_contention(bucket, atomic_read(&heatmap[bucket]));

   {
      struct slot_list *s, *found = NULL;
      size_t bucket = pfn_to_bucket(pfn);
      spin_lock(&lock);
      list_for_each_entry(s, &cache_bins[bucket], bin_list) {
         if(pfn_to_large_pfn(page_to_pfn(s->page)) == pfn_to_large_pfn(pfn)) {
            found = s;
            found->heat += weight;
            list_move(&found->contention_list, &heated_pages[heat_to_contention_level(atomic_read(&heatmap[bucket]))]);
            break;
         }
      }
      spin_unlock(&lock);
      if(found) {
         total_samples_found++;
      } else if(total_samples % 10000 == 0) {
         printk("Didn't find pfn %lu at GB %lu\n", (long unsigned)pfn, (long unsigned)pfn*4096/1024/1024/1024/100);
      }

      total_samples++;
   }
}

/*
 * At initialization time, add a new discovered 2MB page
 */
static int add_new_page(struct page *page) {
   size_t bucket = pfn_to_bucket(page_to_pfn(page));
   size_t idx = heat_to_contention_level(atomic_read(&heatmap[bucket]));
   struct slot_list *s = kmalloc(sizeof(*s), GFP_KERNEL);
   if(!s) {
      printk(KERN_INFO "Fail to allocate all the pages!\n");
      return -1;
   }

   s->bucket = bucket;
   s->page = page;
   s->used_by = 0;
   s->heat = 0;
   INIT_LIST_HEAD(&s->bin_list);
   INIT_LIST_HEAD(&s->contention_list);
   list_add(&s->bin_list, &cache_bins[bucket]);
   list_add(&s->contention_list, &unused_pages[idx]);
   return 0;
}

/*
 * After munmap, re-add the page in the free list
 */
static int add_freed_page(struct page *page) {
   struct slot_list *s;
   size_t bucket = pfn_to_bucket(page_to_pfn(page));

   atomic_sub(50, &heatmap[bucket]);
   move_pages_contention(bucket, atomic_read(&heatmap[bucket]));

   {
      int heat = atomic_read(&heatmap[bucket]);
      int found = 0;
      size_t idx = heat_to_contention_level(heat);

      spin_lock(&lock);
      list_for_each_entry(s, &cache_bins[bucket], bin_list) {
         if(s->page == page) {
            s->used_by = 0;
            list_move(&s->contention_list, &unused_pages[idx]);
            found = 1;
            //printk("Freeing page %lu %lx\n", pfn_to_large_pfn(page_to_pfn(page)), s->virt_addr);
            break;
         }
      }
      spin_unlock(&lock);

      if(!found)
         printk("Didn't find page %lu\n", pfn_to_large_pfn(page_to_pfn(page)));
   }
   return 0;
}

/*
 * Mark a 2MB page as used (done just after mmap)
 */
static void reserve_page(struct hstate *h, int nid, pid_t pid, struct slot_list *s, int contention_idx, struct vm_area_struct *vma, unsigned long addr)
{
   size_t bucket = pfn_to_bucket(page_to_pfn(s->page));

   spin_lock(&lock);
   s->used_by = pid;
   s->heat = 0;
   s->virt_addr = addr;
   list_move(&s->contention_list, &allocated_pages[contention_idx]);
   spin_unlock(&lock);

   atomic_add(50, &heatmap[bucket]);
   move_pages_contention(bucket, atomic_read(&heatmap[bucket]));

   list_move(&s->page->lru, &h->hugepage_activelist);
   set_page_count(s->page, 1);
   ClearHPageFreed(s->page);
   h->free_huge_pages--;
   h->free_huge_pages_node[nid]--;

   total_allocated_pages++;
   //printk("Allocating page %lu heat %d here %lu %lx\n", pfn_to_large_pfn(page_to_pfn(s->page)), atomic_read(&heatmap[bucket]), addr, addr);
}

struct hstate *size_to_hstate(unsigned long size)
{
	struct hstate *h;
   struct hstate *hstates = get_hstates();
   int hugetlb_max_hstate = get_max_hstates();

	for_each_hstate(h) {
		if (huge_page_size(h) == size)
			return h;
	}
	return NULL;
}

/*
 * Function called at initialization time to build the list of 2MB pages
 */
static void build_page_list(void) {
   size_t i, nb_pages = 0;
   int nid = 0; // TODO!
   struct hstate *h;
   struct page *page;
   bool pin = !!(current->flags & PF_MEMALLOC_PIN);
   struct hstate *hstates = get_hstates();
   int hugetlb_max_hstate = get_max_hstates();

   for(i = 0; i < CONTENTION_LEVELS; i++)
      INIT_LIST_HEAD(&unused_pages[i]);
   for(i = 0; i < CONTENTION_LEVELS; i++)
      INIT_LIST_HEAD(&allocated_pages[i]);
   for(i = 0; i < CONTENTION_LEVELS; i++)
      INIT_LIST_HEAD(&heated_pages[i]);
   for(i = 0; i < CACHE_SIZE; i++)
      INIT_LIST_HEAD(&cache_bins[i]);

   // Pages have 0 refcount here -- but let's not increase it to not interfere with the kernel
   // They are not going to disappear anyway, except if the user reduces the number of
   // hugetlbfs pages...
   for_each_hstate(h) {
      list_for_each_entry(page, &h->hugepage_freelists[nid], lru) {
         if (pin && !is_pinnable_page(page))
            continue;

         if (PageHWPoison(page))
            continue;

         nb_pages++;
         if(add_new_page(page) != 0)
            break;
      }
   }
   printk("Successfully created a list of %lu pages\n", nb_pages);
   /*{
      int i;
      struct slot_list *s;
      for(i = 0; i < CACHE_SIZE; i++) {
         int nb_entries = 0, nb_allocated = 0;
         list_for_each_entry(s, &cache_bins[i], bin_list) {
            nb_entries++;
            if(s->used_by)
               nb_allocated++;
         }
         printk("CHECK: cache_bins[%d] = %d entries %d allocated %d heat\n", i, nb_entries, nb_allocated, atomic_read(&heatmap[i]));
      }
   }*/
}

static __attribute__((unused)) void check_list(void) {
   size_t i;
   struct slot_list *s;
   for(i = 0; i < CONTENTION_LEVELS; i++) {
      int nb_entries = 0;
      list_for_each_entry(s, &unused_pages[i], contention_list) {
         nb_entries++;
      }
      printk("CHECK: contention[%lu] = %d entries\n", i, nb_entries);
   }
   return;
   for(i = 0; i < CACHE_SIZE; i++) {
      int nb_entries = 0, nb_allocated = 0;
      list_for_each_entry(s, &cache_bins[i], bin_list) {
         nb_entries++;
         if(s->used_by)
            nb_allocated++;
      }
      printk("CHECK: cache_bins[%lu] = %d entries %d allocated %d heat\n", i, nb_entries, nb_allocated, atomic_read(&heatmap[i]));
   }
}







/*
 * Page choice algorithm
 */
static struct page *_minimize_conflicts(struct hstate *h, int nid, pid_t pid, struct vm_area_struct *vma, unsigned long addr)
{
   size_t i;
   struct page *ret = NULL;
   struct slot_list *s = NULL;
   lockdep_assert_held(&hugetlb_lock);

   for(i = 0; i < CONTENTION_LEVELS; i++) {
      //for(i = CONTENTION_LEVELS - 1; i >= 0; i--) {
      s = list_first_entry_or_null(&unused_pages[i], struct slot_list, contention_list);
      /*size_t lowest_pfn = 0;
      list_for_each_entry(sl, &unused_pages[i], contention_list) {
         if(lowest_pfn == 0 || page_to_pfn(sl->page) < lowest_pfn) {
            s = sl;
            lowest_pfn = page_to_pfn(sl->page);
         }
      }*/
      if(s)
         break;
   }

   if(s) {
      ret = s->page;
      reserve_page(h, nid, pid, s, i, vma, addr);
   }
   return ret;
}

static struct page *minimize_conflicts(struct hstate *h, int nid, struct vm_area_struct *vma, unsigned long addr) {
   return _minimize_conflicts(h, nid, current->tgid, vma, addr);
}

static struct page *enqueue_freed_page(struct hstate *h, struct page *page) {
   int nid = page_to_nid(page);

	lockdep_assert_held(&hugetlb_lock);
	VM_BUG_ON_PAGE(page_count(page), page);

	list_move(&page->lru, &h->hugepage_freelists[nid]);
	h->free_huge_pages++;
	h->free_huge_pages_node[nid]++;
	SetHPageFreed(page);

   add_freed_page(page);
   return page;
}









/*
 * PEBS
 */
static u64 perf_virt_to_phys(u64 virt)
{
	u64 phys_addr = 0;

	if (!virt)
		return 0;

	if (virt >= TASK_SIZE) {
		if (virt_addr_valid((void *)(uintptr_t)virt) &&
		    !(virt >= VMALLOC_START && virt < VMALLOC_END))
			phys_addr = (u64)virt_to_phys((void *)(uintptr_t)virt);
	} else {
		if (current->mm != NULL) {
			struct page *p;
			pagefault_disable();
			if (get_user_page_fast_only(virt, 0, &p)) {
				phys_addr = (page_to_pfn(p) << PAGE_SHIFT) + virt % PAGE_SIZE;
				put_page(p);
			}
			pagefault_enable();
		}
	}
	return phys_addr;
}

static void pebs_sample(struct perf_event *event, struct perf_sample_data *data, struct pt_regs *regs)
{
   size_t phys = perf_virt_to_phys(data->addr);
   size_t pfn = phys / 4096;
   if(phys && pfn) {}
   add_sample(pfn, (event->attr.config == STORE_ALL)?10:1); // TODO
   //printk("Event %p (config %llx) CPU %u vs %u Tid %u Virt addr: %llx Phys %llx\n", event, event->attr.config, data->cpu_entry.cpu, smp_processor_id(), current->pid, data->addr, perf_virt_to_phys(data->addr));
}





/*
 * Periodic migrations
 * 1/ Find pages with high contention
 * 2/ Place these pages in a per pid container
 * 3/ Migrate pages per pid using the migrate_page kernel function
 */
struct pid_pages *insert_pid_in_container(struct pages_container *c, pid_t tgid) {
   struct pid_pages *p = kmalloc(sizeof(*p), GFP_KERNEL);
   memset(p, 0, sizeof(*p));
   p->tgid = tgid;
   p->next = c->pids;
   c->pids = p;
   c->nb_pid++;
   return p;
}

void insert_page_in_container(struct pages_container *c, pid_t tgid, unsigned long addr, struct page *page) {
   struct pid_pages *p = NULL;
   struct pid_pages *l = c->pids;
   while(l) {
      if(l->tgid == tgid) {
         p = l;
         break;
      }
      l = l->next;
   }

   if(!p)
      p = insert_pid_in_container(c, tgid);

   if(p->nb_pages >= p->nb_max_pages) {
      if(p->nb_max_pages) {
         p->nb_max_pages *= 2;
      } else {
         p->nb_max_pages = 256;
      }
      p->addresses = krealloc(p->addresses, sizeof(*p->addresses)*p->nb_max_pages, GFP_KERNEL);
      p->pages = krealloc(p->pages, sizeof(*p->pages)*p->nb_max_pages, GFP_KERNEL);
   }
   p->addresses[p->nb_pages] = addr;
   p->pages[p->nb_pages] = page;
   p->nb_pages++;
}


static void clean_container(struct pages_container *c) {
   struct pid_pages *p, *tmp;
   for(p = c->pids; p;) {
      if(p->pages)
         kfree(p->pages);
      if(p->addresses)
         kfree(p->addresses);
      tmp = p;
      p = p->next;
      kfree(tmp);
   }
   c->pids = 0;
   c->nb_pid = 0;
}


struct task_struct *_find_task_by_vpid(pid_t vnr)
{
   return pid_task(find_pid_ns(vnr, task_active_pid_ns(current)), PIDTYPE_PID);
}

struct page *alloc_migration_target(struct page *old, unsigned long private) {
   struct hstate *h = page_hstate(old);
   struct migration_target_control *mtc = (void*)private;
   return _minimize_conflicts(h, 0, mtc->pid, NULL, (unsigned long)page_address(old));
}

/* Equivalent of the move_pages syscall, except it doesn't migrate pages between NUMA nodes but within the same node               */
/* TODO: optimize. We might not need this whole VMA and MM overhead since we already have the struct page...                       */
/*       ... but at least we know that the following code is safe. pages are refcounted so they are not going to be freed under us */
int s_migrate_pages(pid_t pid, unsigned long nr_pages, unsigned long *addresses, struct page **pages) {
   int nb_migrations = 0, i;
   struct task_struct *task;
   struct mm_struct *mm;
   LIST_HEAD(pagelist);
   struct migration_target_control mtc = {
      .pid = pid
   };

   rcu_read_lock();
	task = _find_task_by_vpid(pid);
	if (!task) {
		rcu_read_unlock();
		return -1;
	}
   rcu_read_unlock();

	get_task_struct(task);
   mm = get_task_mm(task);
   put_task_struct(task);

   if (!mm)
      return -2;

   mmap_read_lock(mm);
   for(i = 0; i < nr_pages; i++) {
      //struct page *page = pages[i]; // unsafe?
      struct page *page;
      struct vm_area_struct *vma = find_vma(mm, addresses[i]);
      if(!vma) {
         printk("Couldn't find VMA for page %lx\n", addresses[i]);
         continue;
      }

      page = follow_page(vma, addresses[i], FOLL_GET | FOLL_DUMP); // increment page_ref_count
      if(!page) // refcount should be == 2 here, 1 because the page is mapped, 1 because follow_page increments it, unless page got freed
         continue;

      isolate_huge_page(page, &pagelist); // also increment page_ref_count, refcount == 3
      put_page(page); // decrement the refcount, should be == 2 now

      nb_migrations++;
   }
   mmap_read_unlock(mm);

   migrate_pages(&pagelist, alloc_migration_target, NULL, (unsigned long)&mtc, MIGRATE_SYNC, MR_NUMA_MISPLACED, NULL); // will decrement all refcounts

   mmput(mm);
   return nb_migrations;
}

static int periodic_migrations(void* data)
{
   if(!ENABLE_PERIODIC_MIGRATIONS)
      return 0;

   memset(&pages_to_migrate, 0, sizeof(pages_to_migrate));
   while(true) {
      struct pid_pages *p;
      struct slot_list *s;
      int idx, nb_attempts = 0, nb_done = 0;

      if(kthread_should_stop()) {
         printk("Exiting periodic migrations");
         return 0;
      }

      spin_lock(&lock);
      for(idx = CONTENTION_LEVELS - 1; idx > PROBLEMATIC_CONTENTION; idx--) {
         list_for_each_entry(s, &heated_pages[idx], contention_list) {
            int nb_heated_pages = 0;
            {
               struct slot_list *_s;
               size_t bucket = pfn_to_bucket(page_to_pfn(s->page));
               //printk("\tWill migrate pfn %lu bucket %lu heat %d pfn heat %d\n", page_to_pfn(s->page), bucket, atomic_read(&heatmap[bucket]), s->heat);
               list_for_each_entry(_s, &cache_bins[bucket], bin_list) {
                  if(_s->heat)
                     nb_heated_pages++;
                  //printk("\t\tpfn %lu heat %d\n", page_to_pfn(_s->page), _s->heat);
               }
            }
            if(nb_heated_pages > 1) {
               struct slot_list *_s;
               size_t bucket = pfn_to_bucket(page_to_pfn(s->page));
               printk("\tWill migrate %lx pfn %lu bucket %lu heat %d pfn heat %d\n", s->virt_addr, page_to_pfn(s->page), bucket, atomic_read(&heatmap[bucket]), s->heat);
               list_for_each_entry(_s, &cache_bins[bucket], bin_list) {
                  if(_s->heat)
                     nb_heated_pages++;
                  printk("\t\tpfn %lu heat %d\n", page_to_pfn(_s->page), _s->heat);
               }
               insert_page_in_container(&pages_to_migrate, s->used_by, s->virt_addr, s->page);
               //list_del(&s->contention_list); // page will be freed
               nb_attempts++;
               if(nb_attempts >= MAX_MIGRATED_PAGES)
                  goto end;
            }
         }
      }
end:
      spin_unlock(&lock);

      for(p = pages_to_migrate.pids; p; p = p->next) {
         nb_done = s_migrate_pages(p->tgid, p->nb_pages, p->addresses, p->pages);
      }


      printk("[Periodic Migrations] Looked at %d pages, %d succeeded\n", nb_attempts, nb_done);

      clean_container(&pages_to_migrate);
      msleep_interruptible(1000);
   }
   return 0;
}



/*
 * Init and cleanup
 */
int init_module(void)
{
   size_t config, cpu, ncpus = num_online_cpus();
   static struct perf_event_attr wd_hw_attr = {
      .type = PERF_TYPE_RAW,
      .size		= sizeof(struct perf_event_attr),
      .pinned		= 0,
      .disabled	= 1,
      .precise_ip = 2,
      .sample_id_all = 1,
      .exclude_kernel = 1,
      .exclude_guest = 1,
      .exclude_hv = 0,
      .exclude_user =0,
      .sample_type = PERF_SAMPLE_IP | PERF_SAMPLE_TID | PERF_SAMPLE_WEIGHT | PERF_SAMPLE_ADDR | PERF_SAMPLE_PHYS_ADDR,
   };

   /* Clear structures */
   printbuf = vmalloc(HEAT_PRINT_SIZE);
   memset(heatmap, 0, sizeof(heatmap));
   memset(&ring, 0, sizeof(ring));

   /* Create all the metadata for the hugepages */
   build_page_list();

   /* Launch perf events */
   events = vmalloc(ncpus * ARRAY_SIZE(configs) * sizeof(*events));
   printk(KERN_INFO "Creating %lu events - %lu configs %lu cpus\n", ARRAY_SIZE(configs) * ncpus, ARRAY_SIZE(configs), ncpus);
   for(config = 0; ENABLE_SAMPLING && (config < ARRAY_SIZE(configs)); config++) {
      for(cpu = 0; cpu < ncpus; cpu++) {
         size_t idx = config * ncpus + cpu;
         wd_hw_attr.config = configs[config];
         if(configs[config] == STORE_ALL)
            wd_hw_attr.sample_period = 5003;
         else
            wd_hw_attr.sample_period = 5003;
         events[idx] = perf_event_create_kernel_counter(&wd_hw_attr, cpu, NULL, pebs_sample, NULL);
         if(IS_ERR(events[idx])) {
            printk(KERN_INFO "Could not create event %lu on cpu %lu\n", configs[config], cpu);
            return -1;
         }
         perf_event_enable(events[idx]);
         printk("Enab\n");
      }
   }

   /* Tell the kernel to use our dequeue/enqueue functions for smart alloc/freeing of huge pages */
   set_dequeue_hook(minimize_conflicts);
   set_enqueue_hook(enqueue_freed_page);

   /* Run periodic migrations */
   kthread = kthread_run(periodic_migrations, NULL, "kthread-periodic-migrations");
   get_task_struct(kthread);

	return 0;
}

void cleanup_module(void)
{
   size_t cpu, config, ncpus = num_online_cpus();

   /* Tell the kernel to stop using our dequeue/enqueue functions */
   set_dequeue_hook(NULL);
   set_enqueue_hook(NULL);

   /* Stop periodic migrations */
   kthread_stop(kthread);
   put_task_struct(kthread);

   /* Print some debug info */
	printk(KERN_INFO "Goodbye world. Total allocated pages %d\n", total_allocated_pages);
	printk(KERN_INFO "Goodbye world. Total samples %llu found %llu\n", total_samples, total_samples_found);
   for(config = 0; ENABLE_SAMPLING && (config < ARRAY_SIZE(configs)); config++) {
      for(cpu = 0; cpu < ncpus; cpu++) {
         size_t idx = config * ncpus + cpu;
         perf_event_disable(events[idx]);
         perf_event_release_kernel(events[idx]);
      }
   }
   check_list();
   vfree(printbuf);
}

module_param(max_conflicts, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(max_conflicts, "Set to 1 to maximize conflicts");
MODULE_LICENSE("GPL");
