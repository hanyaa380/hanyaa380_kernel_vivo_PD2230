/*
 * kernel/lockdep.c
 *
 * Runtime locking correctness validator
 *
 * Started by Ingo Molnar:
 *
 *  Copyright (C) 2006,2007 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *  Copyright (C) 2007 Red Hat, Inc., Peter Zijlstra
 *
 * this code maps all the lock dependencies as they occur in a live kernel
 * and will warn about the following classes of locking bugs:
 *
 * - lock inversion scenarios
 * - circular lock dependencies
 * - hardirq/softirq safe/unsafe locking bugs
 *
 * Bugs are reported even if the current locking scenario does not cause
 * any deadlock at this point.
 *
 * I.e. if anytime in the past two locks were taken in a different order,
 * even if it happened for another task, even if those were different
 * locks (but of the same class as this lock), this code will detect it.
 *
 * Thanks to Arjan van de Ven for coming up with the initial idea of
 * mapping lock dependencies runtime.
 */
#define DISABLE_BRANCH_PROFILING
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/task.h>
#include <linux/sched/mm.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/kallsyms.h>
#include <linux/interrupt.h>
#include <linux/stacktrace.h>
#include <linux/debug_locks.h>
#include <linux/irqflags.h>
#include <linux/utsname.h>
#include <linux/hash.h>
#include <linux/ftrace.h>
#include <linux/stringify.h>
#include <linux/bitops.h>
#include <linux/gfp.h>
#include <linux/random.h>
#include <linux/jhash.h>
#include <linux/nmi.h>

#include <asm/sections.h>

#include "lockdep_internals.h"
#define MTK_INTERNAL_LCOKDEP
#include "lockdep_internals_mtk.h"

#define CREATE_TRACE_POINTS
#include <trace/events/lock.h>

#ifdef CONFIG_PROVE_LOCKING
int prove_locking = 1;
module_param(prove_locking, int, 0644);
#else
#define prove_locking 0
#endif

#ifdef CONFIG_LOCK_STAT
int lock_stat = 1;
module_param(lock_stat, int, 0644);
#else
#define lock_stat 0
#endif

/*
 * lockdep_lock: protects the lockdep graph, the hashes and the
 *               class/list/hash allocators.
 *
 * This is one of the rare exceptions where it's justified
 * to use a raw spinlock - we really dont want the spinlock
 * code to recurse back into the lockdep code...
 */
static arch_spinlock_t lockdep_lock = (arch_spinlock_t)__ARCH_SPIN_LOCK_UNLOCKED;

static int graph_lock(void)
{
	arch_spin_lock(&lockdep_lock);
	/*
	 * Make sure that if another CPU detected a bug while
	 * walking the graph we dont change it (while the other
	 * CPU is busy printing out stuff with the graph lock
	 * dropped already)
	 */
	if (!debug_locks) {
		arch_spin_unlock(&lockdep_lock);
		return 0;
	}
	/* prevent any recursions within lockdep from causing deadlocks */
	current->lockdep_recursion++;
	return 1;
}

static inline int graph_unlock(void)
{
	if (debug_locks && !arch_spin_is_locked(&lockdep_lock)) {
		/*
		 * The lockdep graph lock isn't locked while we expect it to
		 * be, we're confused now, bye!
		 */
		return DEBUG_LOCKS_WARN_ON(1);
	}

	current->lockdep_recursion--;
	arch_spin_unlock(&lockdep_lock);
	return 0;
}

/*
 * Turn lock debugging off and return with 0 if it was off already,
 * and also release the graph lock:
 */
static inline int debug_locks_off_graph_unlock(void)
{
	int ret = debug_locks_off();

	arch_spin_unlock(&lockdep_lock);

	return ret;
}

unsigned long nr_list_entries;
static struct lock_list list_entries[MAX_LOCKDEP_ENTRIES];

/*
 * All data structures here are protected by the global debug_lock.
 *
 * Mutex key structs only get allocated, once during bootup, and never
 * get freed - this significantly simplifies the debugging code.
 */
unsigned long nr_lock_classes;
static struct lock_class lock_classes[MAX_LOCKDEP_KEYS];

static inline struct lock_class *hlock_class(struct held_lock *hlock)
{
	if (!hlock->class_idx) {
		/*
		 * Someone passed in garbage, we give up.
		 */
		DEBUG_LOCKS_WARN_ON(1);
		return NULL;
	}
	return lock_classes + hlock->class_idx - 1;
}

#ifdef CONFIG_LOCK_STAT
static DEFINE_PER_CPU(struct lock_class_stats[MAX_LOCKDEP_KEYS], cpu_lock_stats);

static inline u64 lockstat_clock(void)
{
	return local_clock();
}

static int lock_point(unsigned long points[], unsigned long ip)
{
	int i;

	for (i = 0; i < LOCKSTAT_POINTS; i++) {
		if (points[i] == 0) {
			points[i] = ip;
			break;
		}
		if (points[i] == ip)
			break;
	}

	return i;
}

static void lock_time_inc(struct lock_time *lt, u64 time)
{
	if (time > lt->max)
		lt->max = time;

	if (time < lt->min || !lt->nr)
		lt->min = time;

	lt->total += time;
	lt->nr++;
}

static inline void lock_time_add(struct lock_time *src, struct lock_time *dst)
{
	if (!src->nr)
		return;

	if (src->max > dst->max)
		dst->max = src->max;

	if (src->min < dst->min || !dst->nr)
		dst->min = src->min;

	dst->total += src->total;
	dst->nr += src->nr;
}

struct lock_class_stats lock_stats(struct lock_class *class)
{
	struct lock_class_stats stats;
	int cpu, i;

	memset(&stats, 0, sizeof(struct lock_class_stats));
	for_each_possible_cpu(cpu) {
		struct lock_class_stats *pcs =
			&per_cpu(cpu_lock_stats, cpu)[class - lock_classes];

		for (i = 0; i < ARRAY_SIZE(stats.contention_point); i++)
			stats.contention_point[i] += pcs->contention_point[i];

		for (i = 0; i < ARRAY_SIZE(stats.contending_point); i++)
			stats.contending_point[i] += pcs->contending_point[i];

		lock_time_add(&pcs->read_waittime, &stats.read_waittime);
		lock_time_add(&pcs->write_waittime, &stats.write_waittime);

		lock_time_add(&pcs->read_holdtime, &stats.read_holdtime);
		lock_time_add(&pcs->write_holdtime, &stats.write_holdtime);

		for (i = 0; i < ARRAY_SIZE(stats.bounces); i++)
			stats.bounces[i] += pcs->bounces[i];
	}

	return stats;
}

void clear_lock_stats(struct lock_class *class)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct lock_class_stats *cpu_stats =
			&per_cpu(cpu_lock_stats, cpu)[class - lock_classes];

		memset(cpu_stats, 0, sizeof(struct lock_class_stats));
	}
	memset(class->contention_point, 0, sizeof(class->contention_point));
	memset(class->contending_point, 0, sizeof(class->contending_point));
}

static struct lock_class_stats *get_lock_stats(struct lock_class *class)
{
	return &this_cpu_ptr(cpu_lock_stats)[class - lock_classes];
}

static void lock_release_holdtime(struct held_lock *hlock)
{
	struct lock_class_stats *stats;
	u64 holdtime;

	if (!lock_stat)
		return;

	holdtime = lockstat_clock() - hlock->holdtime_stamp;

	stats = get_lock_stats(hlock_class(hlock));
	if (hlock->read)
		lock_time_inc(&stats->read_holdtime, holdtime);
	else
		lock_time_inc(&stats->write_holdtime, holdtime);
}
#else
static inline void lock_release_holdtime(struct held_lock *hlock)
{
}
#endif

/*
 * We keep a global list of all lock classes. The list only grows,
 * never shrinks. The list is only accessed with the lockdep
 * spinlock lock held.
 */
LIST_HEAD(all_lock_classes);

/*
 * The lockdep classes are in a hash-table as well, for fast lookup:
 */
#define CLASSHASH_BITS		(MAX_LOCKDEP_KEYS_BITS - 1)
#define CLASSHASH_SIZE		(1UL << CLASSHASH_BITS)
#define __classhashfn(key)	hash_long((unsigned long)key, CLASSHASH_BITS)
#define classhashentry(key)	(classhash_table + __classhashfn((key)))

static struct hlist_head classhash_table[CLASSHASH_SIZE];

/*
 * We put the lock dependency chains into a hash-table as well, to cache
 * their existence:
 */
#define CHAINHASH_BITS		(MAX_LOCKDEP_CHAINS_BITS-1)
#define CHAINHASH_SIZE		(1UL << CHAINHASH_BITS)
#define __chainhashfn(chain)	hash_long(chain, CHAINHASH_BITS)
#define chainhashentry(chain)	(chainhash_table + __chainhashfn((chain)))

static struct hlist_head chainhash_table[CHAINHASH_SIZE];

/*
 * The hash key of the lock dependency chains is a hash itself too:
 * it's a hash of all locks taken up to that lock, including that lock.
 * It's a 64-bit hash, because it's important for the keys to be
 * unique.
 */
static inline u64 iterate_chain_key(u64 key, u32 idx)
{
	u32 k0 = key, k1 = key >> 32;

	__jhash_mix(idx, k0, k1); /* Macro that modifies arguments! */

	return k0 | (u64)k1 << 32;
}

void lockdep_off(void)
{
	current->lockdep_recursion++;
}
EXPORT_SYMBOL(lockdep_off);

void lockdep_on(void)
{
	current->lockdep_recursion--;
}
EXPORT_SYMBOL(lockdep_on);

/*
 * Debugging switches:
 */

#define VERBOSE			0
#define VERY_VERBOSE		0

#if VERBOSE
# define HARDIRQ_VERBOSE	1
# define SOFTIRQ_VERBOSE	1
#else
# define HARDIRQ_VERBOSE	0
# define SOFTIRQ_VERBOSE	0
#endif

#if VERBOSE || HARDIRQ_VERBOSE || SOFTIRQ_VERBOSE
/*
 * Quick filtering for interesting events:
 */
static int class_filter(struct lock_class *class)
{
#if 0
	/* Example */
	if (class->name_version == 1 &&
			!strcmp(class->name, "lockname"))
		return 1;
	if (class->name_version == 1 &&
			!strcmp(class->name, "&struct->lockfield"))
		return 1;
#endif
	/* Filter everything else. 1 would be to allow everything else */
	return 0;
}
#endif

static int verbose(struct lock_class *class)
{
#if VERBOSE
	return class_filter(class);
#endif
	return 0;
}

/*
 * Stack-trace: tightly packed array of stack backtrace
 * addresses. Protected by the graph_lock.
 */
unsigned long nr_stack_trace_entries;
static unsigned long stack_trace[MAX_STACK_TRACE_ENTRIES];

static void print_lockdep_off(const char *bug_msg)
{
	printk(KERN_DEBUG "%s\n", bug_msg);
	printk(KERN_DEBUG "turning off the locking correctness validator.\n");
#ifdef CONFIG_LOCK_STAT
	printk(KERN_DEBUG "Please attach the output of /proc/lock_stat to the bug report\n");
#endif
}

static int save_trace(struct stack_trace *trace)
{
	trace->nr_entries = 0;
	trace->max_entries = MAX_STACK_TRACE_ENTRIES - nr_stack_trace_entries;
	trace->entries = stack_trace + nr_stack_trace_entries;

	trace->skip = 3;

	save_stack_trace(trace);

	/*
	 * Some daft arches put -1 at the end to indicate its a full trace.
	 *
	 * <rant> this is buggy anyway, since it takes a whole extra entry so a
	 * complete trace that maxes out the entries provided will be reported
	 * as incomplete, friggin useless </rant>
	 */
	if (trace->nr_entries != 0 &&
	    trace->entries[trace->nr_entries-1] == ULONG_MAX)
		trace->nr_entries--;

	trace->max_entries = trace->nr_entries;

	nr_stack_trace_entries += trace->nr_entries;

	if (nr_stack_trace_entries >= MAX_STACK_TRACE_ENTRIES-1) {
		if (!debug_locks_off_graph_unlock())
			return 0;

		print_lockdep_off("BUG: MAX_STACK_TRACE_ENTRIES too low!");
		dump_stack();
		lockdep_aee();

		return 0;
	}

	return 1;
}

unsigned int nr_hardirq_chains;
unsigned int nr_softirq_chains;
unsigned int nr_process_chains;
unsigned int max_lockdep_depth;

#ifdef CONFIG_DEBUG_LOCKDEP
/*
 * Various lockdep statistics:
 */
DEFINE_PER_CPU(struct lockdep_stats, lockdep_stats);
#endif

/*
 * Locking printouts:
 */

#define __USAGE(__STATE)						\
	[LOCK_USED_IN_##__STATE] = "IN-"__stringify(__STATE)"-W",	\
	[LOCK_ENABLED_##__STATE] = __stringify(__STATE)"-ON-W",		\
	[LOCK_USED_IN_##__STATE##_READ] = "IN-"__stringify(__STATE)"-R",\
	[LOCK_ENABLED_##__STATE##_READ] = __stringify(__STATE)"-ON-R",

static const char *usage_str[] =
{
#define LOCKDEP_STATE(__STATE) __USAGE(__STATE)
#include "lockdep_states.h"
#undef LOCKDEP_STATE
	[LOCK_USED] = "INITIAL USE",
};

const char * __get_key_name(struct lockdep_subclass_key *key, char *str)
{
	return kallsyms_lookup((unsigned long)key, NULL, NULL, NULL, str);
}

static inline unsigned long lock_flag(enum lock_usage_bit bit)
{
	return 1UL << bit;
}

static char get_usage_char(struct lock_class *class, enum lock_usage_bit bit)
{
	char c = '.';

	if (class->usage_mask & lock_flag(bit + 2))
		c = '+';
	if (class->usage_mask & lock_flag(bit)) {
		c = '-';
		if (class->usage_mask & lock_flag(bit + 2))
			c = '?';
	}

	return c;
}

void get_usage_chars(struct lock_class *class, char usage[LOCK_USAGE_CHARS])
{
	int i = 0;

#define LOCKDEP_STATE(__STATE) 						\
	usage[i++] = get_usage_char(class, LOCK_USED_IN_##__STATE);	\
	usage[i++] = get_usage_char(class, LOCK_USED_IN_##__STATE##_READ);
#include "lockdep_states.h"
#undef LOCKDEP_STATE

	usage[i] = '\0';
}

static void __print_lock_name(struct lock_class *class)
{
	char str[KSYM_NAME_LEN];
	const char *name;

	name = class->name;
	if (!name) {
		name = __get_key_name(class->key, str);
		printk(KERN_CONT "%s", name);
	} else {
		printk(KERN_CONT "%s", name);
		if (class->name_version > 1)
			printk(KERN_CONT "#%d", class->name_version);
		if (class->subclass)
			printk(KERN_CONT "/%d", class->subclass);
	}
}

static void print_lock_name(struct lock_class *class)
{
	char usage[LOCK_USAGE_CHARS];

	get_usage_chars(class, usage);

	printk(KERN_CONT " (");
	__print_lock_name(class);
	printk(KERN_CONT "){%s}", usage);
}

static void print_lockdep_cache(struct lockdep_map *lock)
{
	const char *name;
	char str[KSYM_NAME_LEN];

	name = lock->name;
	if (!name)
		name = __get_key_name(lock->key->subkeys, str);

	printk(KERN_CONT "%s", name);
}

static void print_lock(struct held_lock *hlock)
{
	/*
	 * We can be called locklessly through debug_show_all_locks() so be
	 * extra careful, the hlock might have been released and cleared.
	 */
	unsigned int class_idx = hlock->class_idx;

	/* Don't re-read hlock->class_idx, can't use READ_ONCE() on bitfields: */
	barrier();

	if (!class_idx || (class_idx - 1) >= MAX_LOCKDEP_KEYS) {
		printk(KERN_CONT "<RELEASED>\n");
		return;
	}

	printk(KERN_CONT "%p", hlock->instance);
	print_lock_name(lock_classes + class_idx - 1);
	printk(KERN_CONT ", at: %pS\n", (void *)hlock->acquire_ip);
}

#ifndef MTK_LOCK_DEBUG_HELD_LOCK
static void lockdep_print_held_locks(struct task_struct *p)
{
	int i, depth = READ_ONCE(p->lockdep_depth);

	if (!depth)
		printk("no locks held by %s/%d.\n", p->comm, task_pid_nr(p));
	else
		printk("%d lock%s held by %s/%d:\n", depth,
		       depth > 1 ? "s" : "", p->comm, task_pid_nr(p));
	/*
	 * It's not reliable to print a task's held locks if it's not sleeping
	 * and it's not the current task.
	 */
	if (p->state == TASK_RUNNING && p != current)
		return;
	for (i = 0; i < depth; i++) {
		printk(" #%d: ", i);
		print_lock(p->held_locks + i);
	}
}
#endif

static void print_kernel_ident(void)
{
	printk("%s %.*s %s\n", init_utsname()->release,
		(int)strcspn(init_utsname()->version, " "),
		init_utsname()->version,
		print_tainted());
}

static int very_verbose(struct lock_class *class)
{
#if VERY_VERBOSE
	return class_filter(class);
#endif
	return 0;
}

/*
 * Is this the address of a static object:
 */
#ifdef __KERNEL__
static int static_obj(void *obj)
{
	unsigned long start = (unsigned long) &_stext,
		      end   = (unsigned long) &_end,
		      addr  = (unsigned long) obj;

	/*
	 * static variable?
	 */
	if ((addr >= start) && (addr < end))
		return 1;

	if (arch_is_kernel_data(addr))
		return 1;

	/*
	 * in-kernel percpu var?
	 */
	if (is_kernel_percpu_address(addr))
		return 1;

	/*
	 * module static or percpu var?
	 */
	return is_module_address(addr) || is_module_percpu_address(addr);
}
#endif

/*
 * To make lock name printouts unique, we calculate a unique
 * class->name_version generation counter:
 */
static int count_matching_names(struct lock_class *new_class)
{
	struct lock_class *class;
	int count = 0;

	if (!new_class->name)
		return 0;

	list_for_each_entry_rcu(class, &all_lock_classes, lock_entry) {
		if (new_class->key - new_class->subclass == class->key)
			return class->name_version;
		if (class->name && !strcmp(class->name, new_class->name))
			count = max(count, class->name_version);
	}

	return count + 1;
}

static inline struct lock_class *
look_up_lock_class(const struct lockdep_map *lock, unsigned int subclass)
{
	struct lockdep_subclass_key *key;
	struct hlist_head *hash_head;
	struct lock_class *class;

	if (unlikely(subclass >= MAX_LOCKDEP_SUBCLASSES)) {
		debug_locks_off();
		printk(KERN_ERR
			"BUG: looking up invalid subclass: %u\n", subclass);
		printk(KERN_ERR
			"turning off the locking correctness validator.\n");
		dump_stack();
		lockdep_aee();
		return NULL;
	}

	/*
	 * If it is not initialised then it has never been locked,
	 * so it won't be present in the hash table.
	 */
	if (unlikely(!lock->key))
		return NULL;

	/*
	 * NOTE: the class-key must be unique. For dynamic locks, a static
	 * lock_class_key variable is passed in through the mutex_init()
	 * (or spin_lock_init()) call - which acts as the key. For static
	 * locks we use the lock object itself as the key.
	 */
	BUILD_BUG_ON(sizeof(struct lock_class_key) >
			sizeof(struct lockdep_map));

	key = lock->key->subkeys + subclass;

	hash_head = classhashentry(key);

	/*
	 * We do an RCU walk of the hash, see lockdep_free_key_range().
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return NULL;

	hlist_for_each_entry_rcu(class, hash_head, hash_entry) {
		if (class->key == key) {
			/*
			 * Huh! same key, different name? Did someone trample
			 * on some memory? We're most confused.
			 */
			WARN_ON_ONCE(class->name != lock->name);
			return class;
		}
	}

	return NULL;
}

/*
 * Static locks do not have their class-keys yet - for them the key is
 * the lock object itself. If the lock is in the per cpu area, the
 * canonical address of the lock (per cpu offset removed) is used.
 */
static bool assign_lock_key(struct lockdep_map *lock)
{
	unsigned long can_addr, addr = (unsigned long)lock;

	if (__is_kernel_percpu_address(addr, &can_addr))
		lock->key = (void *)can_addr;
	else if (__is_module_percpu_address(addr, &can_addr))
		lock->key = (void *)can_addr;
	else if (static_obj(lock))
		lock->key = (void *)lock;
	else {
		/* Debug-check: all keys must be persistent! */
		debug_locks_off();
		pr_err("INFO: trying to register non-static key.\n");
		pr_err("The code is fine but needs lockdep annotation, or maybe\n");
		pr_err("you didn't initialize this object before use?\n");
		pr_err("turning off the locking correctness validator.\n");
		dump_stack();
		lockdep_aee();
		return false;
	}

	return true;
}

/*
 * Register a lock's class in the hash-table, if the class is not present
 * yet. Otherwise we look it up. We cache the result in the lock object
 * itself, so actual lookup of the hash should be once per lock object.
 */
static struct lock_class *
register_lock_class(struct lockdep_map *lock, unsigned int subclass, int force)
{
	struct lockdep_subclass_key *key;
	struct hlist_head *hash_head;
	struct lock_class *class;

	DEBUG_LOCKS_WARN_ON(!irqs_disabled());

	class = look_up_lock_class(lock, subclass);
	if (likely(class))
		goto out_set_class_cache;

	if (!lock->key) {
		if (!assign_lock_key(lock))
			return NULL;
	} else if (!static_obj(lock->key)) {
		return NULL;
	}

	key = lock->key->subkeys + subclass;
	hash_head = classhashentry(key);

	if (!graph_lock()) {
		return NULL;
	}
	/*
	 * We have to do the hash-walk again, to avoid races
	 * with another CPU:
	 */
	hlist_for_each_entry_rcu(class, hash_head, hash_entry) {
		if (class->key == key)
			goto out_unlock_set;
	}

	/*
	 * Allocate a new key from the static array, and add it to
	 * the hash:
	 */
	if (nr_lock_classes >= MAX_LOCKDEP_KEYS) {
		if (!debug_locks_off_graph_unlock()) {
			return NULL;
		}

		print_lockdep_off("BUG: MAX_LOCKDEP_KEYS too low!");
		dump_stack();
		return NULL;
	}
	class = lock_classes + nr_lock_classes++;
	debug_atomic_inc(nr_unused_locks);
	class->key = key;
	class->name = lock->name;
	class->subclass = subclass;
	INIT_LIST_HEAD(&class->lock_entry);
	INIT_LIST_HEAD(&class->locks_before);
	INIT_LIST_HEAD(&class->locks_after);
	class->name_version = count_matching_names(class);
	/*
	 * We use RCU's safe list-add method to make
	 * parallel walking of the hash-list safe:
	 */
	hlist_add_head_rcu(&class->hash_entry, hash_head);
	/*
	 * Add it to the global list of classes:
	 */
	list_add_tail_rcu(&class->lock_entry, &all_lock_classes);

	if (verbose(class)) {
		graph_unlock();

		printk("\nnew class %px: %s", class->key, class->name);
		if (class->name_version > 1)
			printk(KERN_CONT "#%d", class->name_version);
		printk(KERN_CONT "\n");
		dump_stack();

		if (!graph_lock()) {
			return NULL;
		}
	}
out_unlock_set:
	graph_unlock();

out_set_class_cache:
	if (!subclass || force)
		lock->class_cache[0] = class;
	else if (subclass < NR_LOCKDEP_CACHING_CLASSES)
		lock->class_cache[subclass] = class;

	/*
	 * Hash collision, did we smoke some? We found a class with a matching
	 * hash but the subclass -- which is hashed in -- didn't match.
	 */
	if (DEBUG_LOCKS_WARN_ON(class->subclass != subclass))
		return NULL;

	return class;
}

#ifdef CONFIG_PROVE_LOCKING
/*
 * Allocate a lockdep entry. (assumes the graph_lock held, returns
 * with NULL on failure)
 */
static struct lock_list *alloc_list_entry(void)
{
	if (nr_list_entries >= MAX_LOCKDEP_ENTRIES) {
		if (!debug_locks_off_graph_unlock())
			return NULL;

		print_lockdep_off("BUG: MAX_LOCKDEP_ENTRIES too low!");
		dump_stack();
		lockdep_aee();
		return NULL;
	}
	return list_entries + nr_list_entries++;
}

/*
 * Add a new dependency to the head of the list:
 */
static int add_lock_to_list(struct lock_class *this, struct list_head *head,
			    unsigned long ip, int distance,
			    struct stack_trace *trace)
{
	struct lock_list *entry;
	/*
	 * Lock not present yet - get a new dependency struct and
	 * add it to the list:
	 */
	entry = alloc_list_entry();
	if (!entry)
		return 0;

	entry->class = this;
	entry->distance = distance;
	entry->trace = *trace;
	/*
	 * Both allocation and removal are done under the graph lock; but
	 * iteration is under RCU-sched; see look_up_lock_class() and
	 * lockdep_free_key_range().
	 */
	list_add_tail_rcu(&entry->entry, head);

	return 1;
}

/*
 * For good efficiency of modular, we use power of 2
 */
#define MAX_CIRCULAR_QUEUE_SIZE		4096UL
#define CQ_MASK				(MAX_CIRCULAR_QUEUE_SIZE-1)

/*
 * The circular_queue and helpers is used to implement the
 * breadth-first search(BFS)algorithem, by which we can build
 * the shortest path from the next lock to be acquired to the
 * previous held lock if there is a circular between them.
 */
struct circular_queue {
	unsigned long element[MAX_CIRCULAR_QUEUE_SIZE];
	unsigned int  front, rear;
};

static struct circular_queue lock_cq;

unsigned int max_bfs_queue_depth;

static unsigned int lockdep_dependency_gen_id;

static inline void __cq_init(struct circular_queue *cq)
{
	cq->front = cq->rear = 0;
	lockdep_dependency_gen_id++;
}

static inline int __cq_empty(struct circular_queue *cq)
{
	return (cq->front == cq->rear);
}

static inline int __cq_full(struct circular_queue *cq)
{
	return ((cq->rear + 1) & CQ_MASK) == cq->front;
}

static inline int __cq_enqueue(struct circular_queue *cq, unsigned long elem)
{
	if (__cq_full(cq))
		return -1;

	cq->element[cq->rear] = elem;
	cq->rear = (cq->rear + 1) & CQ_MASK;
	return 0;
}

static inline int __cq_dequeue(struct circular_queue *cq, unsigned long *elem)
{
	if (__cq_empty(cq))
		return -1;

	*elem = cq->element[cq->front];
	cq->front = (cq->front + 1) & CQ_MASK;
	return 0;
}

static inline unsigned int  __cq_get_elem_count(struct circular_queue *cq)
{
	return (cq->rear - cq->front) & CQ_MASK;
}

static inline void mark_lock_accessed(struct lock_list *lock,
					struct lock_list *parent)
{
	unsigned long nr;

	nr = lock - list_entries;
	WARN_ON(nr >= nr_list_entries); /* Out-of-bounds, input fail */
	lock->parent = parent;
	lock->class->dep_gen_id = lockdep_dependency_gen_id;
}

static inline unsigned long lock_accessed(struct lock_list *lock)
{
	unsigned long nr;

	nr = lock - list_entries;
	WARN_ON(nr >= nr_list_entries); /* Out-of-bounds, input fail */
	return lock->class->dep_gen_id == lockdep_dependency_gen_id;
}

static inline struct lock_list *get_lock_parent(struct lock_list *child)
{
	return child->parent;
}

static inline int get_lock_depth(struct lock_list *child)
{
	int depth = 0;
	struct lock_list *parent;

	while ((parent = get_lock_parent(child))) {
		child = parent;
		depth++;
	}
	return depth;
}

static int __bfs(struct lock_list *source_entry,
		 void *data,
		 int (*match)(struct lock_list *entry, void *data),
		 struct lock_list **target_entry,
		 int forward)
{
	struct lock_list *entry;
	struct list_head *head;
	struct circular_queue *cq = &lock_cq;
	int ret = 1;

	if (match(source_entry, data)) {
		*target_entry = source_entry;
		ret = 0;
		goto exit;
	}

	if (forward)
		head = &source_entry->class->locks_after;
	else
		head = &source_entry->class->locks_before;

	if (list_empty(head))
		goto exit;

	__cq_init(cq);
	__cq_enqueue(cq, (unsigned long)source_entry);

	while (!__cq_empty(cq)) {
		struct lock_list *lock;

		__cq_dequeue(cq, (unsigned long *)&lock);

		if (!lock->class) {
			ret = -2;
			goto exit;
		}

		if (forward)
			head = &lock->class->locks_after;
		else
			head = &lock->class->locks_before;

		DEBUG_LOCKS_WARN_ON(!irqs_disabled());

		list_for_each_entry_rcu(entry, head, entry) {
			if (!lock_accessed(entry)) {
				unsigned int cq_depth;
				mark_lock_accessed(entry, lock);
				if (match(entry, data)) {
					*target_entry = entry;
					ret = 0;
					goto exit;
				}

				if (__cq_enqueue(cq, (unsigned long)entry)) {
					ret = -1;
					goto exit;
				}
				cq_depth = __cq_get_elem_count(cq);
				if (max_bfs_queue_depth < cq_depth)
					max_bfs_queue_depth = cq_depth;
			}
		}
	}
exit:
	return ret;
}

static inline int __bfs_forwards(struct lock_list *src_entry,
			void *data,
			int (*match)(struct lock_list *entry, void *data),
			struct lock_list **target_entry)
{
	return __bfs(src_entry, data, match, target_entry, 1);

}

static inline int __bfs_backwards(struct lock_list *src_entry,
			void *data,
			int (*match)(struct lock_list *entry, void *data),
			struct lock_list **target_entry)
{
	return __bfs(src_entry, data, match, target_entry, 0);

}

/*
 * Recursive, forwards-direction lock-dependency checking, used for
 * both noncyclic checking and for hardirq-unsafe/softirq-unsafe
 * checking.
 */

/*
 * Print a dependency chain entry (this is only done when a deadlock
 * has been detected):
 */
static noinline int
print_circular_bug_entry(struct lock_list *target, int depth)
{
	if (debug_locks_silent)
		return 0;
	printk("\n-> #%u", depth);
	print_lock_name(target->class);
	printk(KERN_CONT ":\n");
	print_stack_trace(&target->trace, 6);

	return 0;
}

static void
print_circular_lock_scenario(struct held_lock *src,
			     struct held_lock *tgt,
			     struct lock_list *prt)
{
	struct lock_class *source = hlock_class(src);
	struct lock_class *target = hlock_class(tgt);
	struct lock_class *parent = prt->class;

	/*
	 * A direct locking problem where unsafe_class lock is taken
	 * directly by safe_class lock, then all we need to show
	 * is the deadlock scenario, as it is obvious that the
	 * unsafe lock is taken under the safe lock.
	 *
	 * But if there is a chain instead, where the safe lock takes
	 * an intermediate lock (middle_class) where this lock is
	 * not the same as the safe lock, then the lock chain is
	 * used to describe the problem. Otherwise we would need
	 * to show a different CPU case for each link in the chain
	 * from the safe_class lock to the unsafe_class lock.
	 */
	if (parent != source) {
		printk("Chain exists of:\n  ");
		__print_lock_name(source);
		printk(KERN_CONT " --> ");
		__print_lock_name(parent);
		printk(KERN_CONT " --> ");
		__print_lock_name(target);
		printk(KERN_CONT "\n\n");
	}

	printk(" Possible unsafe locking scenario:\n\n");
	printk("       CPU0                    CPU1\n");
	printk("       ----                    ----\n");
	printk("  lock(");
	__print_lock_name(target);
	printk(KERN_CONT ");\n");
	printk("                               lock(");
	__print_lock_name(parent);
	printk(KERN_CONT ");\n");
	printk("                               lock(");
	__print_lock_name(target);
	printk(KERN_CONT ");\n");
	printk("  lock(");
	__print_lock_name(source);
	printk(KERN_CONT ");\n");
	printk("\n *** DEADLOCK ***\n\n");
}

/*
 * When a circular dependency is detected, print the
 * header first:
 */
static noinline int
print_circular_bug_header(struct lock_list *entry, unsigned int depth,
			struct held_lock *check_src,
			struct held_lock *check_tgt)
{
	struct task_struct *curr = current;

	if (debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("======================================================\n");
	pr_warn("WARNING: possible circular locking dependency detected\n");
	print_kernel_ident();
	pr_warn("------------------------------------------------------\n");
	pr_warn("%s/%d is trying to acquire lock:\n",
		curr->comm, task_pid_nr(curr));
	print_lock(check_src);

	pr_warn("\nbut task is already holding lock:\n");

	print_lock(check_tgt);
	pr_warn("\nwhich lock already depends on the new lock.\n\n");
	pr_warn("\nthe existing dependency chain (in reverse order) is:\n");

	print_circular_bug_entry(entry, depth);
	lockdep_aee();

	return 0;
}

static inline int class_equal(struct lock_list *entry, void *data)
{
	return entry->class == data;
}

static noinline int print_circular_bug(struct lock_list *this,
				struct lock_list *target,
				struct held_lock *check_src,
				struct held_lock *check_tgt,
				struct stack_trace *trace)
{
	struct task_struct *curr = current;
	struct lock_list *parent;
	struct lock_list *first_parent;
	int depth;

	/* MTK_LOCK_LOG_TO_FTRACE */
	if (is_log_lock_held(current)) {
		tracing_on();
		trace_circular_bug(this, target, check_src, check_tgt, trace);
		return 0;
	}

	if (!debug_locks_off_graph_unlock() || debug_locks_silent)
		return 0;

	if (!save_trace(&this->trace))
		return 0;

	depth = get_lock_depth(target);

	print_circular_bug_header(target, depth, check_src, check_tgt);

	parent = get_lock_parent(target);
	first_parent = parent;

	while (parent) {
		print_circular_bug_entry(parent, --depth);
		parent = get_lock_parent(parent);
	}

	printk("\nother info that might help us debug this:\n\n");
	print_circular_lock_scenario(check_src, check_tgt,
				     first_parent);

	lockdep_print_held_locks(curr);

	printk("\nstack backtrace:\n");
	dump_stack();

	return 0;
}

static noinline int print_bfs_bug(int ret)
{
	if (!debug_locks_off_graph_unlock())
		return 0;

	/*
	 * Breadth-first-search failed, graph got corrupted?
	 */
	WARN(1, "lockdep bfs error:%d\n", ret);

	return 0;
}

static int noop_count(struct lock_list *entry, void *data)
{
	(*(unsigned long *)data)++;
	return 0;
}

static unsigned long __lockdep_count_forward_deps(struct lock_list *this)
{
	unsigned long  count = 0;
	struct lock_list *uninitialized_var(target_entry);

	__bfs_forwards(this, (void *)&count, noop_count, &target_entry);

	return count;
}
unsigned long lockdep_count_forward_deps(struct lock_class *class)
{
	unsigned long ret, flags;
	struct lock_list this;

	this.parent = NULL;
	this.class = class;

	raw_local_irq_save(flags);
	current->lockdep_recursion = 1;
	arch_spin_lock(&lockdep_lock);
	ret = __lockdep_count_forward_deps(&this);
	arch_spin_unlock(&lockdep_lock);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);

	return ret;
}

static unsigned long __lockdep_count_backward_deps(struct lock_list *this)
{
	unsigned long  count = 0;
	struct lock_list *uninitialized_var(target_entry);

	__bfs_backwards(this, (void *)&count, noop_count, &target_entry);

	return count;
}

unsigned long lockdep_count_backward_deps(struct lock_class *class)
{
	unsigned long ret, flags;
	struct lock_list this;

	this.parent = NULL;
	this.class = class;

	raw_local_irq_save(flags);
	current->lockdep_recursion = 1;
	arch_spin_lock(&lockdep_lock);
	ret = __lockdep_count_backward_deps(&this);
	arch_spin_unlock(&lockdep_lock);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);

	return ret;
}

/*
 * Prove that the dependency graph starting at <entry> can not
 * lead to <target>. Print an error and return 0 if it does.
 */
static noinline int
check_noncircular(struct lock_list *root, struct lock_class *target,
		struct lock_list **target_entry)
{
	int result;

	debug_atomic_inc(nr_cyclic_checks);

	result = __bfs_forwards(root, target, class_equal, target_entry);

	return result;
}

static noinline int
check_redundant(struct lock_list *root, struct lock_class *target,
		struct lock_list **target_entry)
{
	int result;

	debug_atomic_inc(nr_redundant_checks);

	result = __bfs_forwards(root, target, class_equal, target_entry);

	return result;
}

#if defined(CONFIG_TRACE_IRQFLAGS) && defined(CONFIG_PROVE_LOCKING)
/*
 * Forwards and backwards subgraph searching, for the purposes of
 * proving that two subgraphs can be connected by a new dependency
 * without creating any illegal irq-safe -> irq-unsafe lock dependency.
 */

static inline int usage_match(struct lock_list *entry, void *bit)
{
	return entry->class->usage_mask & (1 << (enum lock_usage_bit)bit);
}



/*
 * Find a node in the forwards-direction dependency sub-graph starting
 * at @root->class that matches @bit.
 *
 * Return 0 if such a node exists in the subgraph, and put that node
 * into *@target_entry.
 *
 * Return 1 otherwise and keep *@target_entry unchanged.
 * Return <0 on error.
 */
static int
find_usage_forwards(struct lock_list *root, enum lock_usage_bit bit,
			struct lock_list **target_entry)
{
	int result;

	debug_atomic_inc(nr_find_usage_forwards_checks);

	result = __bfs_forwards(root, (void *)bit, usage_match, target_entry);

	return result;
}

/*
 * Find a node in the backwards-direction dependency sub-graph starting
 * at @root->class that matches @bit.
 *
 * Return 0 if such a node exists in the subgraph, and put that node
 * into *@target_entry.
 *
 * Return 1 otherwise and keep *@target_entry unchanged.
 * Return <0 on error.
 */
static int
find_usage_backwards(struct lock_list *root, enum lock_usage_bit bit,
			struct lock_list **target_entry)
{
	int result;

	debug_atomic_inc(nr_find_usage_backwards_checks);

	result = __bfs_backwards(root, (void *)bit, usage_match, target_entry);

	return result;
}

static void print_lock_class_header(struct lock_class *class, int depth)
{
	int bit;

	printk("%*s->", depth, "");
	print_lock_name(class);
	printk(KERN_CONT " ops: %lu", class->ops);
	printk(KERN_CONT " {\n");

	for (bit = 0; bit < LOCK_USAGE_STATES; bit++) {
		if (class->usage_mask & (1 << bit)) {
			int len = depth;

			len += printk("%*s   %s", depth, "", usage_str[bit]);
			len += printk(KERN_CONT " at:\n");
			print_stack_trace(class->usage_traces + bit, len);
		}
	}
	printk("%*s }\n", depth, "");

	printk("%*s ... key      at: [<%px>] %pS\n",
		depth, "", class->key, class->key);
}

/*
 * printk the shortest lock dependencies from @start to @end in reverse order:
 */
static void __used
print_shortest_lock_dependencies(struct lock_list *leaf,
				struct lock_list *root)
{
	struct lock_list *entry = leaf;
	int depth;

	/*compute depth from generated tree by BFS*/
	depth = get_lock_depth(leaf);

	do {
		print_lock_class_header(entry->class, depth);
		printk("%*s ... acquired at:\n", depth, "");
		print_stack_trace(&entry->trace, 2);
		printk("\n");

		if (depth == 0 && (entry != root)) {
			printk("lockdep:%s bad path found in chain graph\n", __func__);
			break;
		}

		entry = get_lock_parent(entry);
		depth--;
	} while (entry && (depth >= 0));

	return;
}

static void
print_irq_lock_scenario(struct lock_list *safe_entry,
			struct lock_list *unsafe_entry,
			struct lock_class *prev_class,
			struct lock_class *next_class)
{
	struct lock_class *safe_class = safe_entry->class;
	struct lock_class *unsafe_class = unsafe_entry->class;
	struct lock_class *middle_class = prev_class;

	if (middle_class == safe_class)
		middle_class = next_class;

	/*
	 * A direct locking problem where unsafe_class lock is taken
	 * directly by safe_class lock, then all we need to show
	 * is the deadlock scenario, as it is obvious that the
	 * unsafe lock is taken under the safe lock.
	 *
	 * But if there is a chain instead, where the safe lock takes
	 * an intermediate lock (middle_class) where this lock is
	 * not the same as the safe lock, then the lock chain is
	 * used to describe the problem. Otherwise we would need
	 * to show a different CPU case for each link in the chain
	 * from the safe_class lock to the unsafe_class lock.
	 */
	if (middle_class != unsafe_class) {
		printk("Chain exists of:\n  ");
		__print_lock_name(safe_class);
		printk(KERN_CONT " --> ");
		__print_lock_name(middle_class);
		printk(KERN_CONT " --> ");
		__print_lock_name(unsafe_class);
		printk(KERN_CONT "\n\n");
	}

	printk(" Possible interrupt unsafe locking scenario:\n\n");
	printk("       CPU0                    CPU1\n");
	printk("       ----                    ----\n");
	printk("  lock(");
	__print_lock_name(unsafe_class);
	printk(KERN_CONT ");\n");
	printk("                               local_irq_disable();\n");
	printk("                               lock(");
	__print_lock_name(safe_class);
	printk(KERN_CONT ");\n");
	printk("                               lock(");
	__print_lock_name(middle_class);
	printk(KERN_CONT ");\n");
	printk("  <Interrupt>\n");
	printk("    lock(");
	__print_lock_name(safe_class);
	printk(KERN_CONT ");\n");
	printk("\n *** DEADLOCK ***\n\n");
}

static int
print_bad_irq_dependency(struct task_struct *curr,
			 struct lock_list *prev_root,
			 struct lock_list *next_root,
			 struct lock_list *backwards_entry,
			 struct lock_list *forwards_entry,
			 struct held_lock *prev,
			 struct held_lock *next,
			 enum lock_usage_bit bit1,
			 enum lock_usage_bit bit2,
			 const char *irqclass)
{
	if (!debug_locks_off_graph_unlock() || debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("=====================================================\n");
	pr_warn("WARNING: %s-safe -> %s-unsafe lock order detected\n",
		irqclass, irqclass);
	print_kernel_ident();
	pr_warn("-----------------------------------------------------\n");
	pr_warn("%s/%d [HC%u[%lu]:SC%u[%lu]:HE%u:SE%u] is trying to acquire:\n",
		curr->comm, task_pid_nr(curr),
		curr->hardirq_context, hardirq_count() >> HARDIRQ_SHIFT,
		curr->softirq_context, softirq_count() >> SOFTIRQ_SHIFT,
		curr->hardirqs_enabled,
		curr->softirqs_enabled);
	print_lock(next);

	pr_warn("\nand this task is already holding:\n");
	print_lock(prev);
	pr_warn("which would create a new lock dependency:\n");
	print_lock_name(hlock_class(prev));
	pr_cont(" ->");
	print_lock_name(hlock_class(next));
	pr_cont("\n");

	pr_warn("\nbut this new dependency connects a %s-irq-safe lock:\n",
		irqclass);
	print_lock_name(backwards_entry->class);
	pr_warn("\n... which became %s-irq-safe at:\n", irqclass);

	print_stack_trace(backwards_entry->class->usage_traces + bit1, 1);

	pr_warn("\nto a %s-irq-unsafe lock:\n", irqclass);
	print_lock_name(forwards_entry->class);
	pr_warn("\n... which became %s-irq-unsafe at:\n", irqclass);
	pr_warn("...");

	print_stack_trace(forwards_entry->class->usage_traces + bit2, 1);

	pr_warn("\nother info that might help us debug this:\n\n");
	print_irq_lock_scenario(backwards_entry, forwards_entry,
				hlock_class(prev), hlock_class(next));

	lockdep_print_held_locks(curr);

	pr_warn("\nthe dependencies between %s-irq-safe lock and the holding lock:\n", irqclass);
	if (!save_trace(&prev_root->trace))
		return 0;
	print_shortest_lock_dependencies(backwards_entry, prev_root);

	pr_warn("\nthe dependencies between the lock to be acquired");
	pr_warn(" and %s-irq-unsafe lock:\n", irqclass);
	if (!save_trace(&next_root->trace))
		return 0;
	print_shortest_lock_dependencies(forwards_entry, next_root);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();

	return 0;
}

static int
check_usage(struct task_struct *curr, struct held_lock *prev,
	    struct held_lock *next, enum lock_usage_bit bit_backwards,
	    enum lock_usage_bit bit_forwards, const char *irqclass)
{
	int ret;
	struct lock_list this, that;
	struct lock_list *uninitialized_var(target_entry);
	struct lock_list *uninitialized_var(target_entry1);

	this.parent = NULL;

	this.class = hlock_class(prev);
	ret = find_usage_backwards(&this, bit_backwards, &target_entry);
	if (ret < 0)
		return print_bfs_bug(ret);
	if (ret == 1)
		return ret;

	that.parent = NULL;
	that.class = hlock_class(next);
	ret = find_usage_forwards(&that, bit_forwards, &target_entry1);
	if (ret < 0)
		return print_bfs_bug(ret);
	if (ret == 1)
		return ret;

	return print_bad_irq_dependency(curr, &this, &that,
			target_entry, target_entry1,
			prev, next,
			bit_backwards, bit_forwards, irqclass);
}

static const char *state_names[] = {
#define LOCKDEP_STATE(__STATE) \
	__stringify(__STATE),
#include "lockdep_states.h"
#undef LOCKDEP_STATE
};

static const char *state_rnames[] = {
#define LOCKDEP_STATE(__STATE) \
	__stringify(__STATE)"-READ",
#include "lockdep_states.h"
#undef LOCKDEP_STATE
};

static inline const char *state_name(enum lock_usage_bit bit)
{
	return (bit & 1) ? state_rnames[bit >> 2] : state_names[bit >> 2];
}

static int exclusive_bit(int new_bit)
{
	/*
	 * USED_IN
	 * USED_IN_READ
	 * ENABLED
	 * ENABLED_READ
	 *
	 * bit 0 - write/read
	 * bit 1 - used_in/enabled
	 * bit 2+  state
	 */

	int state = new_bit & ~3;
	int dir = new_bit & 2;

	/*
	 * keep state, bit flip the direction and strip read.
	 */
	return state | (dir ^ 2);
}

static int check_irq_usage(struct task_struct *curr, struct held_lock *prev,
			   struct held_lock *next, enum lock_usage_bit bit)
{
	/*
	 * Prove that the new dependency does not connect a hardirq-safe
	 * lock with a hardirq-unsafe lock - to achieve this we search
	 * the backwards-subgraph starting at <prev>, and the
	 * forwards-subgraph starting at <next>:
	 */
	if (!check_usage(curr, prev, next, bit,
			   exclusive_bit(bit), state_name(bit)))
		return 0;

	bit++; /* _READ */

	/*
	 * Prove that the new dependency does not connect a hardirq-safe-read
	 * lock with a hardirq-unsafe lock - to achieve this we search
	 * the backwards-subgraph starting at <prev>, and the
	 * forwards-subgraph starting at <next>:
	 */
	if (!check_usage(curr, prev, next, bit,
			   exclusive_bit(bit), state_name(bit)))
		return 0;

	return 1;
}

static int
check_prev_add_irq(struct task_struct *curr, struct held_lock *prev,
		struct held_lock *next)
{
#define LOCKDEP_STATE(__STATE)						\
	if (!check_irq_usage(curr, prev, next, LOCK_USED_IN_##__STATE))	\
		return 0;
#include "lockdep_states.h"
#undef LOCKDEP_STATE

	return 1;
}

static void inc_chains(void)
{
	if (current->hardirq_context)
		nr_hardirq_chains++;
	else {
		if (current->softirq_context)
			nr_softirq_chains++;
		else
			nr_process_chains++;
	}
}

#else

static inline int
check_prev_add_irq(struct task_struct *curr, struct held_lock *prev,
		struct held_lock *next)
{
	return 1;
}

static inline void inc_chains(void)
{
	nr_process_chains++;
}

#endif

static void
print_deadlock_scenario(struct held_lock *nxt,
			     struct held_lock *prv)
{
	struct lock_class *next = hlock_class(nxt);
	struct lock_class *prev = hlock_class(prv);

	printk(" Possible unsafe locking scenario:\n\n");
	printk("       CPU0\n");
	printk("       ----\n");
	printk("  lock(");
	__print_lock_name(prev);
	printk(KERN_CONT ");\n");
	printk("  lock(");
	__print_lock_name(next);
	printk(KERN_CONT ");\n");
	printk("\n *** DEADLOCK ***\n\n");
	printk(" May be due to missing lock nesting notation\n\n");
}

static int
print_deadlock_bug(struct task_struct *curr, struct held_lock *prev,
		   struct held_lock *next)
{
	if (!debug_locks_off_graph_unlock() || debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("============================================\n");
	pr_warn("WARNING: possible recursive locking detected\n");
	print_kernel_ident();
	pr_warn("--------------------------------------------\n");
	pr_warn("%s/%d is trying to acquire lock:\n",
		curr->comm, task_pid_nr(curr));
	print_lock(next);
	pr_warn("\nbut task is already holding lock:\n");
	print_lock(prev);

	pr_warn("\nother info that might help us debug this:\n");
	print_deadlock_scenario(next, prev);
	lockdep_print_held_locks(curr);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();

	return 0;
}

/*
 * Check whether we are holding such a class already.
 *
 * (Note that this has to be done separately, because the graph cannot
 * detect such classes of deadlocks.)
 *
 * Returns: 0 on deadlock detected, 1 on OK, 2 on recursive read
 */
static int
check_deadlock(struct task_struct *curr, struct held_lock *next,
	       struct lockdep_map *next_instance, int read)
{
	struct held_lock *prev;
	struct held_lock *nest = NULL;
	int i;

	for (i = 0; i < curr->lockdep_depth; i++) {
		prev = curr->held_locks + i;

		if (prev->instance == next->nest_lock)
			nest = prev;

		if (hlock_class(prev) != hlock_class(next))
			continue;

		/*
		 * Allow read-after-read recursion of the same
		 * lock class (i.e. read_lock(lock)+read_lock(lock)):
		 */
		if ((read == 2) && prev->read)
			return 2;

		/*
		 * We're holding the nest_lock, which serializes this lock's
		 * nesting behaviour.
		 */
		if (nest)
			return 2;

		return print_deadlock_bug(curr, prev, next);
	}
	return 1;
}

/*
 * There was a chain-cache miss, and we are about to add a new dependency
 * to a previous lock. We recursively validate the following rules:
 *
 *  - would the adding of the <prev> -> <next> dependency create a
 *    circular dependency in the graph? [== circular deadlock]
 *
 *  - does the new prev->next dependency connect any hardirq-safe lock
 *    (in the full backwards-subgraph starting at <prev>) with any
 *    hardirq-unsafe lock (in the full forwards-subgraph starting at
 *    <next>)? [== illegal lock inversion with hardirq contexts]
 *
 *  - does the new prev->next dependency connect any softirq-safe lock
 *    (in the full backwards-subgraph starting at <prev>) with any
 *    softirq-unsafe lock (in the full forwards-subgraph starting at
 *    <next>)? [== illegal lock inversion with softirq contexts]
 *
 * any of these scenarios could lead to a deadlock.
 *
 * Then if all the validations pass, we add the forwards and backwards
 * dependency.
 */
static int
check_prev_add(struct task_struct *curr, struct held_lock *prev,
	       struct held_lock *next, int distance, struct stack_trace *trace,
	       int (*save)(struct stack_trace *trace))
{
	struct lock_list *uninitialized_var(target_entry);
	struct lock_list *entry;
	struct lock_list this;
	int ret;

	/*
	 * Prove that the new <prev> -> <next> dependency would not
	 * create a circular dependency in the graph. (We do this by
	 * forward-recursing into the graph starting at <next>, and
	 * checking whether we can reach <prev>.)
	 *
	 * We are using global variables to control the recursion, to
	 * keep the stackframe size of the recursive functions low:
	 */
	this.class = hlock_class(next);
	this.parent = NULL;
	ret = check_noncircular(&this, hlock_class(prev), &target_entry);
	if (unlikely(!ret)) {
		if (!trace->entries) {
			/*
			 * If @save fails here, the printing might trigger
			 * a WARN but because of the !nr_entries it should
			 * not do bad things.
			 */
			save(trace);
		}
		return print_circular_bug(&this, target_entry, next, prev, trace);
	}
	else if (unlikely(ret < 0))
		return print_bfs_bug(ret);

	if (!check_prev_add_irq(curr, prev, next))
		return 0;

	/*
	 * For recursive read-locks we do all the dependency checks,
	 * but we dont store read-triggered dependencies (only
	 * write-triggered dependencies). This ensures that only the
	 * write-side dependencies matter, and that if for example a
	 * write-lock never takes any other locks, then the reads are
	 * equivalent to a NOP.
	 */
	if (next->read == 2 || prev->read == 2)
		return 1;
	/*
	 * Is the <prev> -> <next> dependency already present?
	 *
	 * (this may occur even though this is a new chain: consider
	 *  e.g. the L1 -> L2 -> L3 -> L4 and the L5 -> L1 -> L2 -> L3
	 *  chains - the second one will be new, but L1 already has
	 *  L2 added to its dependency list, due to the first chain.)
	 */
	list_for_each_entry(entry, &hlock_class(prev)->locks_after, entry) {
		if (entry->class == hlock_class(next)) {
			if (distance == 1)
				entry->distance = 1;
			return 1;
		}
	}

	/*
	 * Is the <prev> -> <next> link redundant?
	 */
	this.class = hlock_class(prev);
	this.parent = NULL;
	ret = check_redundant(&this, hlock_class(next), &target_entry);
	if (!ret) {
		debug_atomic_inc(nr_redundant);
		return 2;
	}
	if (ret < 0)
		return print_bfs_bug(ret);


	if (!trace->entries && !save(trace))
		return 0;

	/*
	 * Ok, all validations passed, add the new lock
	 * to the previous lock's dependency list:
	 */
	ret = add_lock_to_list(hlock_class(next),
			       &hlock_class(prev)->locks_after,
			       next->acquire_ip, distance, trace);

	if (!ret)
		return 0;

	ret = add_lock_to_list(hlock_class(prev),
			       &hlock_class(next)->locks_before,
			       next->acquire_ip, distance, trace);
	if (!ret)
		return 0;

#ifdef MTK_LOCK_DEBUG_NEW_DEPENDENCY
	check_new_dependency(prev, next);
#endif

	return 2;
}

/*
 * Add the dependency to all directly-previous locks that are 'relevant'.
 * The ones that are relevant are (in increasing distance from curr):
 * all consecutive trylock entries and the final non-trylock entry - or
 * the end of this context's lock-chain - whichever comes first.
 */
static int
check_prevs_add(struct task_struct *curr, struct held_lock *next)
{
	int depth = curr->lockdep_depth;
	struct held_lock *hlock;
	struct stack_trace trace = {
		.nr_entries = 0,
		.max_entries = 0,
		.entries = NULL,
		.skip = 0,
	};

	/*
	 * Debugging checks.
	 *
	 * Depth must not be zero for a non-head lock:
	 */
	if (!depth)
		goto out_bug;
	/*
	 * At least two relevant locks must exist for this
	 * to be a head:
	 */
	if (curr->held_locks[depth].irq_context !=
			curr->held_locks[depth-1].irq_context)
		goto out_bug;

	for (;;) {
		int distance = curr->lockdep_depth - depth + 1;
		hlock = curr->held_locks + depth - 1;

		/*
		 * Only non-recursive-read entries get new dependencies
		 * added:
		 */
		if (hlock->read != 2 && hlock->check) {
			int ret = check_prev_add(curr, hlock, next, distance, &trace, save_trace);
			if (!ret)
				return 0;

			/*
			 * Stop after the first non-trylock entry,
			 * as non-trylock entries have added their
			 * own direct dependencies already, so this
			 * lock is connected to them indirectly:
			 */
			if (!hlock->trylock)
				break;
		}

		depth--;
		/*
		 * End of lock-stack?
		 */
		if (!depth)
			break;
		/*
		 * Stop the search if we cross into another context:
		 */
		if (curr->held_locks[depth].irq_context !=
				curr->held_locks[depth-1].irq_context)
			break;
	}
	return 1;
out_bug:
	if (!debug_locks_off_graph_unlock())
		return 0;

	/*
	 * Clearly we all shouldn't be here, but since we made it we
	 * can reliable say we messed up our state. See the above two
	 * gotos for reasons why we could possibly end up here.
	 */
	WARN_ON(1);

	return 0;
}

unsigned long nr_lock_chains;
struct lock_chain lock_chains[MAX_LOCKDEP_CHAINS];
int nr_chain_hlocks;
static u16 chain_hlocks[MAX_LOCKDEP_CHAIN_HLOCKS];

struct lock_class *lock_chain_get_class(struct lock_chain *chain, int i)
{
	return lock_classes + chain_hlocks[chain->base + i];
}

/*
 * Returns the index of the first held_lock of the current chain
 */
static inline int get_first_held_lock(struct task_struct *curr,
					struct held_lock *hlock)
{
	int i;
	struct held_lock *hlock_curr;

	for (i = curr->lockdep_depth - 1; i >= 0; i--) {
		hlock_curr = curr->held_locks + i;
		if (hlock_curr->irq_context != hlock->irq_context)
			break;

	}

	return ++i;
}

#ifdef CONFIG_DEBUG_LOCKDEP
/*
 * Returns the next chain_key iteration
 */
static u64 print_chain_key_iteration(int class_idx, u64 chain_key)
{
	u64 new_chain_key = iterate_chain_key(chain_key, class_idx);

	printk(" class_idx:%d -> chain_key:%016Lx",
		class_idx,
		(unsigned long long)new_chain_key);
	return new_chain_key;
}

static void
print_chain_keys_held_locks(struct task_struct *curr, struct held_lock *hlock_next)
{
	struct held_lock *hlock;
	u64 chain_key = 0;
	int depth = curr->lockdep_depth;
	int i;

	printk("depth: %u\n", depth + 1);
	for (i = get_first_held_lock(curr, hlock_next); i < depth; i++) {
		hlock = curr->held_locks + i;
		chain_key = print_chain_key_iteration(hlock->class_idx, chain_key);

		print_lock(hlock);
	}

	print_chain_key_iteration(hlock_next->class_idx, chain_key);
	print_lock(hlock_next);
}

static void print_chain_keys_chain(struct lock_chain *chain)
{
	int i;
	u64 chain_key = 0;
	int class_id;

	printk("depth: %u\n", chain->depth);
	for (i = 0; i < chain->depth; i++) {
		class_id = chain_hlocks[chain->base + i];
		chain_key = print_chain_key_iteration(class_id + 1, chain_key);

		print_lock_name(lock_classes + class_id);
		printk("\n");
	}
}

static void print_collision(struct task_struct *curr,
			struct held_lock *hlock_next,
			struct lock_chain *chain)
{
	pr_warn("\n");
	pr_warn("============================\n");
	pr_warn("WARNING: chain_key collision\n");
	print_kernel_ident();
	pr_warn("----------------------------\n");
	pr_warn("%s/%d: ", current->comm, task_pid_nr(current));
	pr_warn("Hash chain already cached but the contents don't match!\n");

	pr_warn("Held locks:");
	print_chain_keys_held_locks(curr, hlock_next);

	pr_warn("Locks in cached chain:");
	print_chain_keys_chain(chain);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
}
#endif

/*
 * Checks whether the chain and the current held locks are consistent
 * in depth and also in content. If they are not it most likely means
 * that there was a collision during the calculation of the chain_key.
 * Returns: 0 not passed, 1 passed
 */
static int check_no_collision(struct task_struct *curr,
			struct held_lock *hlock,
			struct lock_chain *chain)
{
#ifdef CONFIG_DEBUG_LOCKDEP
	int i, j, id;

	i = get_first_held_lock(curr, hlock);

	if (DEBUG_LOCKS_WARN_ON(chain->depth != curr->lockdep_depth - (i - 1))) {
		print_collision(curr, hlock, chain);
		return 0;
	}

	for (j = 0; j < chain->depth - 1; j++, i++) {
		id = curr->held_locks[i].class_idx - 1;

		if (DEBUG_LOCKS_WARN_ON(chain_hlocks[chain->base + j] != id)) {
			print_collision(curr, hlock, chain);
			return 0;
		}
	}
#endif
	return 1;
}

/*
 * This is for building a chain between just two different classes,
 * instead of adding a new hlock upon current, which is done by
 * add_chain_cache().
 *
 * This can be called in any context with two classes, while
 * add_chain_cache() must be done within the lock owener's context
 * since it uses hlock which might be racy in another context.
 */
static inline int add_chain_cache_classes(unsigned int prev,
					  unsigned int next,
					  unsigned int irq_context,
					  u64 chain_key)
{
	struct hlist_head *hash_head = chainhashentry(chain_key);
	struct lock_chain *chain;

	/*
	 * Allocate a new chain entry from the static array, and add
	 * it to the hash:
	 */

	/*
	 * We might need to take the graph lock, ensure we've got IRQs
	 * disabled to make this an IRQ-safe lock.. for recursion reasons
	 * lockdep won't complain about its own locking errors.
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return 0;

	if (unlikely(nr_lock_chains >= MAX_LOCKDEP_CHAINS)) {
		if (!debug_locks_off_graph_unlock())
			return 0;

		print_lockdep_off("BUG: MAX_LOCKDEP_CHAINS too low!");
		dump_stack();
		return 0;
	}

	chain = lock_chains + nr_lock_chains++;
	chain->chain_key = chain_key;
	chain->irq_context = irq_context;
	chain->depth = 2;
	if (likely(nr_chain_hlocks + chain->depth <= MAX_LOCKDEP_CHAIN_HLOCKS)) {
		chain->base = nr_chain_hlocks;
		nr_chain_hlocks += chain->depth;
		chain_hlocks[chain->base] = prev - 1;
		chain_hlocks[chain->base + 1] = next -1;
	}
#ifdef CONFIG_DEBUG_LOCKDEP
	/*
	 * Important for check_no_collision().
	 */
	else {
		if (!debug_locks_off_graph_unlock())
			return 0;

		print_lockdep_off("BUG: MAX_LOCKDEP_CHAIN_HLOCKS too low!");
		dump_stack();
		return 0;
	}
#endif

	hlist_add_head_rcu(&chain->entry, hash_head);
	debug_atomic_inc(chain_lookup_misses);
	inc_chains();

	return 1;
}

/*
 * Adds a dependency chain into chain hashtable. And must be called with
 * graph_lock held.
 *
 * Return 0 if fail, and graph_lock is released.
 * Return 1 if succeed, with graph_lock held.
 */
static inline int add_chain_cache(struct task_struct *curr,
				  struct held_lock *hlock,
				  u64 chain_key)
{
	struct lock_class *class = hlock_class(hlock);
	struct hlist_head *hash_head = chainhashentry(chain_key);
	struct lock_chain *chain;
	int i, j;

	/*
	 * Allocate a new chain entry from the static array, and add
	 * it to the hash:
	 */

	/*
	 * We might need to take the graph lock, ensure we've got IRQs
	 * disabled to make this an IRQ-safe lock.. for recursion reasons
	 * lockdep won't complain about its own locking errors.
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return 0;

	if (unlikely(nr_lock_chains >= MAX_LOCKDEP_CHAINS)) {
		if (!debug_locks_off_graph_unlock())
			return 0;

		print_lockdep_off("BUG: MAX_LOCKDEP_CHAINS too low!");
		dump_stack();
		return 0;
	}
	chain = lock_chains + nr_lock_chains++;
	chain->chain_key = chain_key;
	chain->irq_context = hlock->irq_context;
	i = get_first_held_lock(curr, hlock);
	chain->depth = curr->lockdep_depth + 1 - i;

	BUILD_BUG_ON((1UL << 24) <= ARRAY_SIZE(chain_hlocks));
	BUILD_BUG_ON((1UL << 6)  <= ARRAY_SIZE(curr->held_locks));
	BUILD_BUG_ON((1UL << 8*sizeof(chain_hlocks[0])) <= ARRAY_SIZE(lock_classes));

	if (likely(nr_chain_hlocks + chain->depth <= MAX_LOCKDEP_CHAIN_HLOCKS)) {
		chain->base = nr_chain_hlocks;
		for (j = 0; j < chain->depth - 1; j++, i++) {
			int lock_id = curr->held_locks[i].class_idx - 1;
			chain_hlocks[chain->base + j] = lock_id;
		}
		chain_hlocks[chain->base + j] = class - lock_classes;
	}

	if (nr_chain_hlocks < MAX_LOCKDEP_CHAIN_HLOCKS)
		nr_chain_hlocks += chain->depth;

#ifdef CONFIG_DEBUG_LOCKDEP
	/*
	 * Important for check_no_collision().
	 */
	if (unlikely(nr_chain_hlocks > MAX_LOCKDEP_CHAIN_HLOCKS)) {
		if (!debug_locks_off_graph_unlock())
			return 0;

		print_lockdep_off("BUG: MAX_LOCKDEP_CHAIN_HLOCKS too low!");
		dump_stack();
		return 0;
	}
#endif

	hlist_add_head_rcu(&chain->entry, hash_head);
	debug_atomic_inc(chain_lookup_misses);
	inc_chains();

	return 1;
}

/*
 * Look up a dependency chain.
 */
static inline struct lock_chain *lookup_chain_cache(u64 chain_key)
{
	struct hlist_head *hash_head = chainhashentry(chain_key);
	struct lock_chain *chain;

	/*
	 * We can walk it lock-free, because entries only get added
	 * to the hash:
	 */
	hlist_for_each_entry_rcu(chain, hash_head, entry) {
		if (chain->chain_key == chain_key) {
			debug_atomic_inc(chain_lookup_hits);
			return chain;
		}
	}
	return NULL;
}

/*
 * If the key is not present yet in dependency chain cache then
 * add it and return 1 - in this case the new dependency chain is
 * validated. If the key is already hashed, return 0.
 * (On return with 1 graph_lock is held.)
 */
static inline int lookup_chain_cache_add(struct task_struct *curr,
					 struct held_lock *hlock,
					 u64 chain_key)
{
	struct lock_class *class = hlock_class(hlock);
	struct lock_chain *chain = lookup_chain_cache(chain_key);

	if (chain) {
cache_hit:
		if (!check_no_collision(curr, hlock, chain))
			return 0;

		if (very_verbose(class)) {
			printk("\nhash chain already cached, key: "
					"%016Lx tail class: [%px] %s\n",
					(unsigned long long)chain_key,
					class->key, class->name);
		}

		return 0;
	}

	if (very_verbose(class)) {
		printk("\nnew hash chain, key: %016Lx tail class: [%px] %s\n",
			(unsigned long long)chain_key, class->key, class->name);
	}

	if (!graph_lock())
		return 0;

	/*
	 * We have to walk the chain again locked - to avoid duplicates:
	 */
	chain = lookup_chain_cache(chain_key);
	if (chain) {
		graph_unlock();
		goto cache_hit;
	}

	if (!add_chain_cache(curr, hlock, chain_key))
		return 0;

	return 1;
}

static int validate_chain(struct task_struct *curr, struct lockdep_map *lock,
		struct held_lock *hlock, int chain_head, u64 chain_key)
{
	/*
	 * Trylock needs to maintain the stack of held locks, but it
	 * does not add new dependencies, because trylock can be done
	 * in any order.
	 *
	 * We look up the chain_key and do the O(N^2) check and update of
	 * the dependencies only if this is a new dependency chain.
	 * (If lookup_chain_cache_add() return with 1 it acquires
	 * graph_lock for us)
	 */
	if (!hlock->trylock && hlock->check &&
	    lookup_chain_cache_add(curr, hlock, chain_key)) {
		/*
		 * Check whether last held lock:
		 *
		 * - is irq-safe, if this lock is irq-unsafe
		 * - is softirq-safe, if this lock is hardirq-unsafe
		 *
		 * And check whether the new lock's dependency graph
		 * could lead back to the previous lock.
		 *
		 * any of these scenarios could lead to a deadlock. If
		 * All validations
		 */
		int ret = check_deadlock(curr, hlock, lock, hlock->read);

		if (!ret)
			return 0;
		/*
		 * Mark recursive read, as we jump over it when
		 * building dependencies (just like we jump over
		 * trylock entries):
		 */
		if (ret == 2)
			hlock->read = 2;
		/*
		 * Add dependency only if this lock is not the head
		 * of the chain, and if it's not a secondary read-lock:
		 */
		if (!chain_head && ret != 2) {
			if (!check_prevs_add(curr, hlock))
				return 0;
		}

		graph_unlock();
	} else {
		/* after lookup_chain_cache_add(): */
		if (unlikely(!debug_locks))
			return 0;
	}

	return 1;
}
#else
static inline int validate_chain(struct task_struct *curr,
	       	struct lockdep_map *lock, struct held_lock *hlock,
		int chain_head, u64 chain_key)
{
	return 1;
}
#endif

/*
 * We are building curr_chain_key incrementally, so double-check
 * it from scratch, to make sure that it's done correctly:
 */
static void check_chain_key(struct task_struct *curr)
{
#ifdef CONFIG_DEBUG_LOCKDEP
	struct held_lock *hlock, *prev_hlock = NULL;
	unsigned int i;
	u64 chain_key = 0;

	for (i = 0; i < curr->lockdep_depth; i++) {
		hlock = curr->held_locks + i;
		if (chain_key != hlock->prev_chain_key) {
			debug_locks_off();
			/*
			 * We got mighty confused, our chain keys don't match
			 * with what we expect, someone trample on our task state?
			 */
			WARN(1, "hm#1, depth: %u [%u], %016Lx != %016Lx\n",
				curr->lockdep_depth, i,
				(unsigned long long)chain_key,
				(unsigned long long)hlock->prev_chain_key);
			return;
		}
		/*
		 * Whoops ran out of static storage again?
		 */
		if (DEBUG_LOCKS_WARN_ON(hlock->class_idx > MAX_LOCKDEP_KEYS))
			return;

		if (prev_hlock && (prev_hlock->irq_context !=
							hlock->irq_context))
			chain_key = 0;
		chain_key = iterate_chain_key(chain_key, hlock->class_idx);
		prev_hlock = hlock;
	}
	if (chain_key != curr->curr_chain_key) {
		debug_locks_off();
		/*
		 * More smoking hash instead of calculating it, damn see these
		 * numbers float.. I bet that a pink elephant stepped on my memory.
		 */
		WARN(1, "hm#2, depth: %u [%u], %016Lx != %016Lx\n",
			curr->lockdep_depth, i,
			(unsigned long long)chain_key,
			(unsigned long long)curr->curr_chain_key);
	}
#endif
}

static void
print_usage_bug_scenario(struct held_lock *lock)
{
	struct lock_class *class = hlock_class(lock);

	printk(" Possible unsafe locking scenario:\n\n");
	printk("       CPU0\n");
	printk("       ----\n");
	printk("  lock(");
	__print_lock_name(class);
	printk(KERN_CONT ");\n");
	printk("  <Interrupt>\n");
	printk("    lock(");
	__print_lock_name(class);
	printk(KERN_CONT ");\n");
	printk("\n *** DEADLOCK ***\n\n");
}

static int
print_usage_bug(struct task_struct *curr, struct held_lock *this,
		enum lock_usage_bit prev_bit, enum lock_usage_bit new_bit)
{
	if (!debug_locks_off_graph_unlock() || debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("================================\n");
	pr_warn("WARNING: inconsistent lock state\n");
	print_kernel_ident();
	pr_warn("--------------------------------\n");

	pr_warn("inconsistent {%s} -> {%s} usage.\n",
		usage_str[prev_bit], usage_str[new_bit]);

	pr_warn("%s/%d [HC%u[%lu]:SC%u[%lu]:HE%u:SE%u] takes:\n",
		curr->comm, task_pid_nr(curr),
		trace_hardirq_context(curr), hardirq_count() >> HARDIRQ_SHIFT,
		trace_softirq_context(curr), softirq_count() >> SOFTIRQ_SHIFT,
		trace_hardirqs_enabled(curr),
		trace_softirqs_enabled(curr));
	print_lock(this);

	pr_warn("{%s} state was registered at:\n", usage_str[prev_bit]);
	print_stack_trace(hlock_class(this)->usage_traces + prev_bit, 1);

	print_irqtrace_events(curr);
	pr_warn("\nother info that might help us debug this:\n");
	print_usage_bug_scenario(this);

	lockdep_print_held_locks(curr);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();

	return 0;
}

/*
 * Print out an error if an invalid bit is set:
 */
static inline int
valid_state(struct task_struct *curr, struct held_lock *this,
	    enum lock_usage_bit new_bit, enum lock_usage_bit bad_bit)
{
	if (unlikely(hlock_class(this)->usage_mask & (1 << bad_bit)))
		return print_usage_bug(curr, this, bad_bit, new_bit);
	return 1;
}

static int mark_lock(struct task_struct *curr, struct held_lock *this,
		     enum lock_usage_bit new_bit);

#if defined(CONFIG_TRACE_IRQFLAGS) && defined(CONFIG_PROVE_LOCKING)

/*
 * print irq inversion bug:
 */
static int
print_irq_inversion_bug(struct task_struct *curr,
			struct lock_list *root, struct lock_list *other,
			struct held_lock *this, int forwards,
			const char *irqclass)
{
	struct lock_list *entry = other;
	struct lock_list *middle = NULL;
	int depth;

	if (!debug_locks_off_graph_unlock() || debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("========================================================\n");
	pr_warn("WARNING: possible irq lock inversion dependency detected\n");
	print_kernel_ident();
	pr_warn("--------------------------------------------------------\n");
	pr_warn("%s/%d just changed the state of lock:\n",
		curr->comm, task_pid_nr(curr));
	print_lock(this);
	if (forwards)
		pr_warn("but this lock took another, %s-unsafe lock in the past:\n", irqclass);
	else
		pr_warn("but this lock was taken by another, %s-safe lock in the past:\n", irqclass);
	print_lock_name(other->class);
	pr_warn("\n\nand interrupts could create inverse lock ordering between them.\n\n");

	pr_warn("\nother info that might help us debug this:\n");

	/* Find a middle lock (if one exists) */
	depth = get_lock_depth(other);
	do {
		if (depth == 0 && (entry != root)) {
			pr_warn("lockdep:%s bad path found in chain graph\n", __func__);
			break;
		}
		middle = entry;
		entry = get_lock_parent(entry);
		depth--;
	} while (entry && entry != root && (depth >= 0));
	if (forwards)
		print_irq_lock_scenario(root, other,
			middle ? middle->class : root->class, other->class);
	else
		print_irq_lock_scenario(other, root,
			middle ? middle->class : other->class, root->class);

	lockdep_print_held_locks(curr);

	pr_warn("\nthe shortest dependencies between 2nd lock and 1st lock:\n");
	if (!save_trace(&root->trace))
		return 0;
	print_shortest_lock_dependencies(other, root);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();

	return 0;
}

/*
 * Prove that in the forwards-direction subgraph starting at <this>
 * there is no lock matching <mask>:
 */
static int
check_usage_forwards(struct task_struct *curr, struct held_lock *this,
		     enum lock_usage_bit bit, const char *irqclass)
{
	int ret;
	struct lock_list root;
	struct lock_list *uninitialized_var(target_entry);

	root.parent = NULL;
	root.class = hlock_class(this);
	ret = find_usage_forwards(&root, bit, &target_entry);
	if (ret < 0)
		return print_bfs_bug(ret);
	if (ret == 1)
		return ret;

	return print_irq_inversion_bug(curr, &root, target_entry,
					this, 1, irqclass);
}

/*
 * Prove that in the backwards-direction subgraph starting at <this>
 * there is no lock matching <mask>:
 */
static int
check_usage_backwards(struct task_struct *curr, struct held_lock *this,
		      enum lock_usage_bit bit, const char *irqclass)
{
	int ret;
	struct lock_list root;
	struct lock_list *uninitialized_var(target_entry);

	root.parent = NULL;
	root.class = hlock_class(this);
	ret = find_usage_backwards(&root, bit, &target_entry);
	if (ret < 0)
		return print_bfs_bug(ret);
	if (ret == 1)
		return ret;

	return print_irq_inversion_bug(curr, &root, target_entry,
					this, 0, irqclass);
}

void print_irqtrace_events(struct task_struct *curr)
{
	printk("irq event stamp: %u\n", curr->irq_events);
	printk("hardirqs last  enabled at (%u): [<%px>] %pS\n",
		curr->hardirq_enable_event, (void *)curr->hardirq_enable_ip,
		(void *)curr->hardirq_enable_ip);
	printk("hardirqs last disabled at (%u): [<%px>] %pS\n",
		curr->hardirq_disable_event, (void *)curr->hardirq_disable_ip,
		(void *)curr->hardirq_disable_ip);
	printk("softirqs last  enabled at (%u): [<%px>] %pS\n",
		curr->softirq_enable_event, (void *)curr->softirq_enable_ip,
		(void *)curr->softirq_enable_ip);
	printk("softirqs last disabled at (%u): [<%px>] %pS\n",
		curr->softirq_disable_event, (void *)curr->softirq_disable_ip,
		(void *)curr->softirq_disable_ip);
}

static int HARDIRQ_verbose(struct lock_class *class)
{
#if HARDIRQ_VERBOSE
	return class_filter(class);
#endif
	return 0;
}

static int SOFTIRQ_verbose(struct lock_class *class)
{
#if SOFTIRQ_VERBOSE
	return class_filter(class);
#endif
	return 0;
}

#define STRICT_READ_CHECKS	1

static int (*state_verbose_f[])(struct lock_class *class) = {
#define LOCKDEP_STATE(__STATE) \
	__STATE##_verbose,
#include "lockdep_states.h"
#undef LOCKDEP_STATE
};

static inline int state_verbose(enum lock_usage_bit bit,
				struct lock_class *class)
{
	return state_verbose_f[bit >> 2](class);
}

typedef int (*check_usage_f)(struct task_struct *, struct held_lock *,
			     enum lock_usage_bit bit, const char *name);

static int
mark_lock_irq(struct task_struct *curr, struct held_lock *this,
		enum lock_usage_bit new_bit)
{
	int excl_bit = exclusive_bit(new_bit);
	int read = new_bit & 1;
	int dir = new_bit & 2;

	/*
	 * mark USED_IN has to look forwards -- to ensure no dependency
	 * has ENABLED state, which would allow recursion deadlocks.
	 *
	 * mark ENABLED has to look backwards -- to ensure no dependee
	 * has USED_IN state, which, again, would allow  recursion deadlocks.
	 */
	check_usage_f usage = dir ?
		check_usage_backwards : check_usage_forwards;

	/*
	 * Validate that this particular lock does not have conflicting
	 * usage states.
	 */
	if (!valid_state(curr, this, new_bit, excl_bit))
		return 0;

	/*
	 * Validate that the lock dependencies don't have conflicting usage
	 * states.
	 */
	if ((!read || !dir || STRICT_READ_CHECKS) &&
			!usage(curr, this, excl_bit, state_name(new_bit & ~1)))
		return 0;

	/*
	 * Check for read in write conflicts
	 */
	if (!read) {
		if (!valid_state(curr, this, new_bit, excl_bit + 1))
			return 0;

		if (STRICT_READ_CHECKS &&
			!usage(curr, this, excl_bit + 1,
				state_name(new_bit + 1)))
			return 0;
	}

	if (state_verbose(new_bit, hlock_class(this)))
		return 2;

	return 1;
}

enum mark_type {
#define LOCKDEP_STATE(__STATE)	__STATE,
#include "lockdep_states.h"
#undef LOCKDEP_STATE
};

/*
 * Mark all held locks with a usage bit:
 */
static int
mark_held_locks(struct task_struct *curr, enum mark_type mark)
{
	enum lock_usage_bit usage_bit;
	struct held_lock *hlock;
	int i;

	for (i = 0; i < curr->lockdep_depth; i++) {
		hlock = curr->held_locks + i;

		usage_bit = 2 + (mark << 2); /* ENABLED */
		if (hlock->read)
			usage_bit += 1; /* READ */

		BUG_ON(usage_bit >= LOCK_USAGE_STATES);

		if (!hlock->check)
			continue;

		if (!mark_lock(curr, hlock, usage_bit))
			return 0;
	}

	return 1;
}

/*
 * Hardirqs will be enabled:
 */
static void __trace_hardirqs_on_caller(unsigned long ip)
{
	struct task_struct *curr = current;

	/* we'll do an OFF -> ON transition: */
	curr->hardirqs_enabled = 1;

	/*
	 * We are going to turn hardirqs on, so set the
	 * usage bit for all held locks:
	 */
	if (!mark_held_locks(curr, HARDIRQ))
		return;
	/*
	 * If we have softirqs enabled, then set the usage
	 * bit for all held locks. (disabled hardirqs prevented
	 * this bit from being set before)
	 */
	if (curr->softirqs_enabled)
		if (!mark_held_locks(curr, SOFTIRQ))
			return;

	curr->hardirq_enable_ip = ip;
	curr->hardirq_enable_event = ++curr->irq_events;
	debug_atomic_inc(hardirqs_on_events);
}

void lockdep_hardirqs_on(unsigned long ip)
{
	if (unlikely(!debug_locks || current->lockdep_recursion))
		return;

	if (unlikely(current->hardirqs_enabled)) {
		/*
		 * Neither irq nor preemption are disabled here
		 * so this is racy by nature but losing one hit
		 * in a stat is not a big deal.
		 */
		__debug_atomic_inc(redundant_hardirqs_on);
		return;
	}

	/*
	 * We're enabling irqs and according to our state above irqs weren't
	 * already enabled, yet we find the hardware thinks they are in fact
	 * enabled.. someone messed up their IRQ state tracing.
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return;

	/*
	 * See the fine text that goes along with this variable definition.
	 */
	if (DEBUG_LOCKS_WARN_ON(unlikely(early_boot_irqs_disabled)))
		return;

	/*
	 * Can't allow enabling interrupts while in an interrupt handler,
	 * that's general bad form and such. Recursion, limited stack etc..
	 */
	if (DEBUG_LOCKS_WARN_ON(current->hardirq_context))
		return;

	current->lockdep_recursion = 1;
	__trace_hardirqs_on_caller(ip);
	current->lockdep_recursion = 0;
}

/*
 * Hardirqs were disabled:
 */
void lockdep_hardirqs_off(unsigned long ip)
{
	struct task_struct *curr = current;

	if (unlikely(!debug_locks || current->lockdep_recursion))
		return;

	/*
	 * So we're supposed to get called after you mask local IRQs, but for
	 * some reason the hardware doesn't quite think you did a proper job.
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return;

	if (curr->hardirqs_enabled) {
		/*
		 * We have done an ON -> OFF transition:
		 */
		curr->hardirqs_enabled = 0;
		curr->hardirq_disable_ip = ip;
		curr->hardirq_disable_event = ++curr->irq_events;
		debug_atomic_inc(hardirqs_off_events);
	} else
		debug_atomic_inc(redundant_hardirqs_off);
}

/*
 * Softirqs will be enabled:
 */
void trace_softirqs_on(unsigned long ip)
{
	struct task_struct *curr = current;

	if (unlikely(!debug_locks || current->lockdep_recursion))
		return;

	/*
	 * We fancy IRQs being disabled here, see softirq.c, avoids
	 * funny state and nesting things.
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return;

	if (curr->softirqs_enabled) {
		debug_atomic_inc(redundant_softirqs_on);
		return;
	}

	current->lockdep_recursion = 1;
	/*
	 * We'll do an OFF -> ON transition:
	 */
	curr->softirqs_enabled = 1;
	curr->softirq_enable_ip = ip;
	curr->softirq_enable_event = ++curr->irq_events;
	debug_atomic_inc(softirqs_on_events);
	/*
	 * We are going to turn softirqs on, so set the
	 * usage bit for all held locks, if hardirqs are
	 * enabled too:
	 */
	if (curr->hardirqs_enabled)
		mark_held_locks(curr, SOFTIRQ);
	current->lockdep_recursion = 0;
}

/*
 * Softirqs were disabled:
 */
void trace_softirqs_off(unsigned long ip)
{
	struct task_struct *curr = current;

	if (unlikely(!debug_locks || current->lockdep_recursion))
		return;

	/*
	 * We fancy IRQs being disabled here, see softirq.c
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return;

	if (curr->softirqs_enabled) {
		/*
		 * We have done an ON -> OFF transition:
		 */
		curr->softirqs_enabled = 0;
		curr->softirq_disable_ip = ip;
		curr->softirq_disable_event = ++curr->irq_events;
		debug_atomic_inc(softirqs_off_events);
		/*
		 * Whoops, we wanted softirqs off, so why aren't they?
		 */
		DEBUG_LOCKS_WARN_ON(!softirq_count());
	} else
		debug_atomic_inc(redundant_softirqs_off);
}

static int mark_irqflags(struct task_struct *curr, struct held_lock *hlock)
{
	/*
	 * If non-trylock use in a hardirq or softirq context, then
	 * mark the lock as used in these contexts:
	 */
	if (!hlock->trylock) {
		if (hlock->read) {
			if (curr->hardirq_context)
				if (!mark_lock(curr, hlock,
						LOCK_USED_IN_HARDIRQ_READ))
					return 0;
			if (curr->softirq_context)
				if (!mark_lock(curr, hlock,
						LOCK_USED_IN_SOFTIRQ_READ))
					return 0;
		} else {
			if (curr->hardirq_context)
				if (!mark_lock(curr, hlock, LOCK_USED_IN_HARDIRQ))
					return 0;
			if (curr->softirq_context)
				if (!mark_lock(curr, hlock, LOCK_USED_IN_SOFTIRQ))
					return 0;
		}
	}
	if (!hlock->hardirqs_off) {
		if (hlock->read) {
			if (!mark_lock(curr, hlock,
					LOCK_ENABLED_HARDIRQ_READ))
				return 0;
			if (curr->softirqs_enabled)
				if (!mark_lock(curr, hlock,
						LOCK_ENABLED_SOFTIRQ_READ))
					return 0;
		} else {
			if (!mark_lock(curr, hlock,
					LOCK_ENABLED_HARDIRQ))
				return 0;
			if (curr->softirqs_enabled)
				if (!mark_lock(curr, hlock,
						LOCK_ENABLED_SOFTIRQ))
					return 0;
		}
	}

	return 1;
}

static inline unsigned int task_irq_context(struct task_struct *task)
{
	return 2 * !!task->hardirq_context + !!task->softirq_context;
}

static int separate_irq_context(struct task_struct *curr,
		struct held_lock *hlock)
{
	unsigned int depth = curr->lockdep_depth;

	/*
	 * Keep track of points where we cross into an interrupt context:
	 */
	if (depth) {
		struct held_lock *prev_hlock;

		prev_hlock = curr->held_locks + depth-1;
		/*
		 * If we cross into another context, reset the
		 * hash key (this also prevents the checking and the
		 * adding of the dependency to 'prev'):
		 */
		if (prev_hlock->irq_context != hlock->irq_context)
			return 1;
	}
	return 0;
}

#else /* defined(CONFIG_TRACE_IRQFLAGS) && defined(CONFIG_PROVE_LOCKING) */

static inline
int mark_lock_irq(struct task_struct *curr, struct held_lock *this,
		enum lock_usage_bit new_bit)
{
	WARN_ON(1); /* Impossible innit? when we don't have TRACE_IRQFLAG */
	return 1;
}

static inline int mark_irqflags(struct task_struct *curr,
		struct held_lock *hlock)
{
	return 1;
}

static inline unsigned int task_irq_context(struct task_struct *task)
{
	return 0;
}

static inline int separate_irq_context(struct task_struct *curr,
		struct held_lock *hlock)
{
	return 0;
}

#endif /* defined(CONFIG_TRACE_IRQFLAGS) && defined(CONFIG_PROVE_LOCKING) */

/*
 * Mark a lock with a usage bit, and validate the state transition:
 */
static int mark_lock(struct task_struct *curr, struct held_lock *this,
			     enum lock_usage_bit new_bit)
{
	unsigned int new_mask = 1 << new_bit, ret = 1;

	/*
	 * If already set then do not dirty the cacheline,
	 * nor do any checks:
	 */
	if (likely(hlock_class(this)->usage_mask & new_mask))
		return 1;

	if (!graph_lock())
		return 0;
	/*
	 * Make sure we didn't race:
	 */
	if (unlikely(hlock_class(this)->usage_mask & new_mask)) {
		graph_unlock();
		return 1;
	}

	hlock_class(this)->usage_mask |= new_mask;

	if (!save_trace(hlock_class(this)->usage_traces + new_bit))
		return 0;

	switch (new_bit) {
#define LOCKDEP_STATE(__STATE)			\
	case LOCK_USED_IN_##__STATE:		\
	case LOCK_USED_IN_##__STATE##_READ:	\
	case LOCK_ENABLED_##__STATE:		\
	case LOCK_ENABLED_##__STATE##_READ:
#include "lockdep_states.h"
#undef LOCKDEP_STATE
		ret = mark_lock_irq(curr, this, new_bit);
		if (!ret)
			return 0;
		break;
	case LOCK_USED:
		debug_atomic_dec(nr_unused_locks);
		break;
	default:
		if (!debug_locks_off_graph_unlock())
			return 0;
		WARN_ON(1);
		return 0;
	}

	graph_unlock();

	/*
	 * We must printk outside of the graph_lock:
	 */
	if (ret == 2) {
		printk("\nmarked lock as {%s}:\n", usage_str[new_bit]);
		print_lock(this);
		print_irqtrace_events(curr);
		dump_stack();
	}

	return ret;
}

/*
 * Initialize a lock instance's lock-class mapping info:
 */
static void __lockdep_init_map(struct lockdep_map *lock, const char *name,
		      struct lock_class_key *key, int subclass)
{
	int i;

	for (i = 0; i < NR_LOCKDEP_CACHING_CLASSES; i++)
		lock->class_cache[i] = NULL;

#ifdef CONFIG_LOCK_STAT
	lock->cpu = raw_smp_processor_id();
#endif

	/*
	 * Can't be having no nameless bastards around this place!
	 */
	if (DEBUG_LOCKS_WARN_ON(!name)) {
		lock->name = "NULL";
		return;
	}

	lock->name = name;

	/*
	 * No key, no joy, we need to hash something.
	 */
	if (DEBUG_LOCKS_WARN_ON(!key))
		return;
	/*
	 * Sanity check, the lock-class key must be persistent:
	 */
	if (!static_obj(key)) {
		printk("BUG: key %px not in .data!\n", key);
		/*
		 * What it says above ^^^^^, I suggest you read it.
		 */
		DEBUG_LOCKS_WARN_ON(1);
		lockdep_aee();
		return;
	}
	lock->key = key;

	if (unlikely(!debug_locks))
		return;

	if (subclass) {
		unsigned long flags;

		if (DEBUG_LOCKS_WARN_ON(current->lockdep_recursion))
			return;

		raw_local_irq_save(flags);
		current->lockdep_recursion = 1;
		register_lock_class(lock, subclass, 1);
		current->lockdep_recursion = 0;
		raw_local_irq_restore(flags);
	}
}

void lockdep_init_map(struct lockdep_map *lock, const char *name,
		      struct lock_class_key *key, int subclass)
{
	__lockdep_init_map(lock, name, key, subclass);
}
EXPORT_SYMBOL_GPL(lockdep_init_map);

struct lock_class_key __lockdep_no_validate__;
EXPORT_SYMBOL_GPL(__lockdep_no_validate__);

static int
print_lock_nested_lock_not_held(struct task_struct *curr,
				struct held_lock *hlock,
				unsigned long ip)
{
	if (!debug_locks_off())
		return 0;
	if (debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("==================================\n");
	pr_warn("WARNING: Nested lock was not taken\n");
	print_kernel_ident();
	pr_warn("----------------------------------\n");

	pr_warn("%s/%d is trying to lock:\n", curr->comm, task_pid_nr(curr));
	print_lock(hlock);

	pr_warn("\nbut this task is not holding:\n");
	pr_warn("%s\n", hlock->nest_lock->name);

	pr_warn("\nstack backtrace:\n");
	dump_stack();

	pr_warn("\nother info that might help us debug this:\n");
	lockdep_print_held_locks(curr);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();

	return 0;
}

static int __lock_is_held(const struct lockdep_map *lock, int read);

/*
 * This gets called for every mutex_lock*()/spin_lock*() operation.
 * We maintain the dependency maps and validate the locking attempt:
 */
static int __lock_acquire(struct lockdep_map *lock, unsigned int subclass,
			  int trylock, int read, int check, int hardirqs_off,
			  struct lockdep_map *nest_lock, unsigned long ip,
			  int references, int pin_count)
{
	struct task_struct *curr = current;
	struct lock_class *class = NULL;
	struct held_lock *hlock;
	unsigned int depth;
	int chain_head = 0;
	int class_idx;
	u64 chain_key;

#ifdef MTK_LOCK_MONITOR
	if (!debug_locks)
		__add_held_lock(lock, subclass, trylock, read, check,
				hardirqs_off, nest_lock, ip,
				references, pin_count);
#endif

	if (unlikely(!debug_locks))
		return 0;

	/*
	 * Lockdep should run with IRQs disabled, otherwise we could
	 * get an interrupt which would want to take locks, which would
	 * end up in lockdep and have you got a head-ache already?
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return 0;

	if (!prove_locking || lock->key == &__lockdep_no_validate__)
		check = 0;

	if (subclass < NR_LOCKDEP_CACHING_CLASSES)
		class = lock->class_cache[subclass];
	/*
	 * Not cached?
	 */
	if (unlikely(!class)) {
		class = register_lock_class(lock, subclass, 0);
		if (!class)
			return 0;
	}
	atomic_inc((atomic_t *)&class->ops);
	if (very_verbose(class)) {
		printk("\nacquire class [%px] %s", class->key, class->name);
		if (class->name_version > 1)
			printk(KERN_CONT "#%d", class->name_version);
		printk(KERN_CONT "\n");
		dump_stack();
	}

	/*
	 * Add the lock to the list of currently held locks.
	 * (we dont increase the depth just yet, up until the
	 * dependency checks are done)
	 */
	depth = curr->lockdep_depth;
	/*
	 * Ran out of static storage for our per-task lock stack again have we?
	 */
	if (DEBUG_LOCKS_WARN_ON(depth >= MAX_LOCK_DEPTH))
		return 0;

	class_idx = class - lock_classes + 1;

	if (depth) {
		hlock = curr->held_locks + depth - 1;
		if (hlock->class_idx == class_idx && nest_lock) {
			if (!references)
				references++;

			if (!hlock->references)
				hlock->references++;

			hlock->references += references;

			/* Overflow */
			if (DEBUG_LOCKS_WARN_ON(hlock->references < references))
				return 0;

			return 1;
		}
	}

	hlock = curr->held_locks + depth;
	/*
	 * Plain impossible, we just registered it and checked it weren't no
	 * NULL like.. I bet this mushroom I ate was good!
	 */
	if (DEBUG_LOCKS_WARN_ON(!class))
		return 0;
	hlock->class_idx = class_idx;
	hlock->acquire_ip = ip;
	hlock->instance = lock;
	hlock->nest_lock = nest_lock;
	hlock->irq_context = task_irq_context(curr);
	hlock->trylock = trylock;
	hlock->read = read;
	hlock->check = check;
	hlock->hardirqs_off = !!hardirqs_off;
	hlock->references = references;
#ifdef CONFIG_LOCK_STAT
	hlock->waittime_stamp = 0;
	hlock->holdtime_stamp = lockstat_clock();
#endif
	hlock->pin_count = pin_count;

#ifdef MTK_LOCK_DEBUG_HELD_LOCK
	held_lock_save_trace(&hlock->trace, hlock->entries);
#endif
#ifdef MTK_LOCK_MONITOR
	hlock->timestamp = sched_clock();
	hlock->acquired = true;
#endif

	if (check && !mark_irqflags(curr, hlock))
		return 0;

	/* mark it as used: */
	if (!mark_lock(curr, hlock, LOCK_USED))
		return 0;

	/*
	 * Calculate the chain hash: it's the combined hash of all the
	 * lock keys along the dependency chain. We save the hash value
	 * at every step so that we can get the current hash easily
	 * after unlock. The chain hash is then used to cache dependency
	 * results.
	 *
	 * The 'key ID' is what is the most compact key value to drive
	 * the hash, not class->key.
	 */
	/*
	 * Whoops, we did it again.. ran straight out of our static allocation.
	 */
	if (DEBUG_LOCKS_WARN_ON(class_idx > MAX_LOCKDEP_KEYS))
		return 0;

	chain_key = curr->curr_chain_key;
	if (!depth) {
		/*
		 * How can we have a chain hash when we ain't got no keys?!
		 */
		if (DEBUG_LOCKS_WARN_ON(chain_key != 0))
			return 0;
		chain_head = 1;
	}

	hlock->prev_chain_key = chain_key;
	if (separate_irq_context(curr, hlock)) {
		chain_key = 0;
		chain_head = 1;
	}
	chain_key = iterate_chain_key(chain_key, class_idx);

	if (nest_lock && !__lock_is_held(nest_lock, -1))
		return print_lock_nested_lock_not_held(curr, hlock, ip);

	if (!validate_chain(curr, lock, hlock, chain_head, chain_key))
		return 0;

	curr->curr_chain_key = chain_key;
	curr->lockdep_depth++;
	check_chain_key(curr);
#ifdef CONFIG_DEBUG_LOCKDEP
	if (unlikely(!debug_locks))
		return 0;
#endif
	if (unlikely(curr->lockdep_depth >= MAX_LOCK_DEPTH)) {
		debug_locks_off();
		print_lockdep_off("BUG: MAX_LOCK_DEPTH too low!");
		printk(KERN_DEBUG "depth: %i  max: %lu!\n",
		       curr->lockdep_depth, MAX_LOCK_DEPTH);

		lockdep_print_held_locks(current);
		debug_show_all_locks();
		dump_stack();

		return 0;
	}

	if (unlikely(curr->lockdep_depth > max_lockdep_depth))
		max_lockdep_depth = curr->lockdep_depth;

	return 1;
}

static int
print_unlock_imbalance_bug(struct task_struct *curr, struct lockdep_map *lock,
			   unsigned long ip)
{
	if (!debug_locks_off())
		return 0;
	if (debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("=====================================\n");
	pr_warn("WARNING: bad unlock balance detected!\n");
	print_kernel_ident();
	pr_warn("-------------------------------------\n");
	pr_warn("%s/%d is trying to release lock (",
		curr->comm, task_pid_nr(curr));
	print_lockdep_cache(lock);
	pr_cont(") at:\n");
	print_ip_sym(ip);
	pr_warn("but there are no more locks to release!\n");
	pr_warn("\nother info that might help us debug this:\n");
	lockdep_print_held_locks(curr);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();

	return 0;
}

static int match_held_lock(const struct held_lock *hlock,
					const struct lockdep_map *lock)
{
	if (hlock->instance == lock)
		return 1;

	if (hlock->references) {
		const struct lock_class *class = lock->class_cache[0];

		if (!class)
			class = look_up_lock_class(lock, 0);

		/*
		 * If look_up_lock_class() failed to find a class, we're trying
		 * to test if we hold a lock that has never yet been acquired.
		 * Clearly if the lock hasn't been acquired _ever_, we're not
		 * holding it either, so report failure.
		 */
		if (!class)
			return 0;

		/*
		 * References, but not a lock we're actually ref-counting?
		 * State got messed up, follow the sites that change ->references
		 * and try to make sense of it.
		 */
		if (DEBUG_LOCKS_WARN_ON(!hlock->nest_lock))
			return 0;

		if (hlock->class_idx == class - lock_classes + 1)
			return 1;
	}

	return 0;
}

/* @depth must not be zero */
static struct held_lock *find_held_lock(struct task_struct *curr,
					struct lockdep_map *lock,
					unsigned int depth, int *idx)
{
	struct held_lock *ret, *hlock, *prev_hlock;
	int i;

	i = depth - 1;
	hlock = curr->held_locks + i;
	ret = hlock;
	if (match_held_lock(hlock, lock))
		goto out;

	ret = NULL;
	for (i--, prev_hlock = hlock--;
	     i >= 0;
	     i--, prev_hlock = hlock--) {
		/*
		 * We must not cross into another context:
		 */
		if (prev_hlock->irq_context != hlock->irq_context) {
			ret = NULL;
			break;
		}
		if (match_held_lock(hlock, lock)) {
			ret = hlock;
			break;
		}
	}

out:
	*idx = i;
	return ret;
}

static int reacquire_held_locks(struct task_struct *curr, unsigned int depth,
			      int idx)
{
	struct held_lock *hlock;

	for (hlock = curr->held_locks + idx; idx < depth; idx++, hlock++) {
		if (!__lock_acquire(hlock->instance,
				    hlock_class(hlock)->subclass,
				    hlock->trylock,
				    hlock->read, hlock->check,
				    hlock->hardirqs_off,
				    hlock->nest_lock, hlock->acquire_ip,
				    hlock->references, hlock->pin_count))
			return 1;
	}
	return 0;
}

static int
__lock_set_class(struct lockdep_map *lock, const char *name,
		 struct lock_class_key *key, unsigned int subclass,
		 unsigned long ip)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	struct lock_class *class;
	unsigned int depth;
	int i;

	depth = curr->lockdep_depth;
	/*
	 * This function is about (re)setting the class of a held lock,
	 * yet we're not actually holding any locks. Naughty user!
	 */
	if (DEBUG_LOCKS_WARN_ON(!depth))
		return 0;

	hlock = find_held_lock(curr, lock, depth, &i);
	if (!hlock)
		return print_unlock_imbalance_bug(curr, lock, ip);

	lockdep_init_map(lock, name, key, 0);
	class = register_lock_class(lock, subclass, 0);
	hlock->class_idx = class - lock_classes + 1;

	curr->lockdep_depth = i;
	curr->curr_chain_key = hlock->prev_chain_key;

	if (reacquire_held_locks(curr, depth, i))
		return 0;

	/*
	 * I took it apart and put it back together again, except now I have
	 * these 'spare' parts.. where shall I put them.
	 */
	if (DEBUG_LOCKS_WARN_ON(curr->lockdep_depth != depth))
		return 0;
	return 1;
}

static int __lock_downgrade(struct lockdep_map *lock, unsigned long ip)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	unsigned int depth;
	int i;

	if (unlikely(!debug_locks))
		return 0;

	depth = curr->lockdep_depth;
	/*
	 * This function is about (re)setting the class of a held lock,
	 * yet we're not actually holding any locks. Naughty user!
	 */
	if (DEBUG_LOCKS_WARN_ON(!depth))
		return 0;

	hlock = find_held_lock(curr, lock, depth, &i);
	if (!hlock)
		return print_unlock_imbalance_bug(curr, lock, ip);

	curr->lockdep_depth = i;
	curr->curr_chain_key = hlock->prev_chain_key;

	WARN(hlock->read, "downgrading a read lock");
	hlock->read = 1;
	hlock->acquire_ip = ip;

	if (reacquire_held_locks(curr, depth, i))
		return 0;

	/*
	 * I took it apart and put it back together again, except now I have
	 * these 'spare' parts.. where shall I put them.
	 */
	if (DEBUG_LOCKS_WARN_ON(curr->lockdep_depth != depth))
		return 0;
	return 1;
}

/*
 * Remove the lock to the list of currently held locks - this gets
 * called on mutex_unlock()/spin_unlock*() (or on a failed
 * mutex_lock_interruptible()).
 *
 * @nested is an hysterical artifact, needs a tree wide cleanup.
 */
static int
__lock_release(struct lockdep_map *lock, int nested, unsigned long ip)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	unsigned int depth;
	int i;

#ifdef MTK_LOCK_MONITOR
	if (!debug_locks)
		__del_held_lock(lock);
#endif

	if (unlikely(!debug_locks))
		return 0;

	depth = curr->lockdep_depth;
	/*
	 * So we're all set to release this lock.. wait what lock? We don't
	 * own any locks, you've been drinking again?
	 */
	if (depth <= 0)
		return print_unlock_imbalance_bug(curr, lock, ip);

	/*
	 * Check whether the lock exists in the current stack
	 * of held locks:
	 */
	hlock = find_held_lock(curr, lock, depth, &i);
	if (!hlock)
		return print_unlock_imbalance_bug(curr, lock, ip);

	if (hlock->instance == lock)
		lock_release_holdtime(hlock);

#ifdef MTK_LOCK_MONITOR
	hlock->timestamp = 0;
#endif

	WARN(hlock->pin_count, "releasing a pinned lock\n");

	if (hlock->references) {
		hlock->references--;
		if (hlock->references) {
			/*
			 * We had, and after removing one, still have
			 * references, the current lock stack is still
			 * valid. We're done!
			 */
			return 1;
		}
	}

	/*
	 * We have the right lock to unlock, 'hlock' points to it.
	 * Now we remove it from the stack, and add back the other
	 * entries (if any), recalculating the hash along the way:
	 */

	curr->lockdep_depth = i;
	curr->curr_chain_key = hlock->prev_chain_key;

	if (reacquire_held_locks(curr, depth, i + 1))
		return 0;

	/*
	 * We had N bottles of beer on the wall, we drank one, but now
	 * there's not N-1 bottles of beer left on the wall...
	 */
	if (DEBUG_LOCKS_WARN_ON(curr->lockdep_depth != depth - 1))
		return 0;

	return 1;
}

static int __lock_is_held(const struct lockdep_map *lock, int read)
{
	struct task_struct *curr = current;
	int i;

	for (i = 0; i < curr->lockdep_depth; i++) {
		struct held_lock *hlock = curr->held_locks + i;

		if (match_held_lock(hlock, lock)) {
			if (read == -1 || hlock->read == read)
				return 1;

			return 0;
		}
	}

	return 0;
}

static struct pin_cookie __lock_pin_lock(struct lockdep_map *lock)
{
	struct pin_cookie cookie = NIL_COOKIE;
	struct task_struct *curr = current;
	int i;

	if (unlikely(!debug_locks))
		return cookie;

	for (i = 0; i < curr->lockdep_depth; i++) {
		struct held_lock *hlock = curr->held_locks + i;

		if (match_held_lock(hlock, lock)) {
			/*
			 * Grab 16bits of randomness; this is sufficient to not
			 * be guessable and still allows some pin nesting in
			 * our u32 pin_count.
			 */
			cookie.val = 1 + (prandom_u32() >> 16);
			hlock->pin_count += cookie.val;
			return cookie;
		}
	}

	WARN(1, "pinning an unheld lock\n");
	return cookie;
}

static void __lock_repin_lock(struct lockdep_map *lock, struct pin_cookie cookie)
{
	struct task_struct *curr = current;
	int i;

	if (unlikely(!debug_locks))
		return;

	for (i = 0; i < curr->lockdep_depth; i++) {
		struct held_lock *hlock = curr->held_locks + i;

		if (match_held_lock(hlock, lock)) {
			hlock->pin_count += cookie.val;
			return;
		}
	}

	WARN(1, "pinning an unheld lock\n");
}

static void __lock_unpin_lock(struct lockdep_map *lock, struct pin_cookie cookie)
{
	struct task_struct *curr = current;
	int i;

	if (unlikely(!debug_locks))
		return;

	for (i = 0; i < curr->lockdep_depth; i++) {
		struct held_lock *hlock = curr->held_locks + i;

		if (match_held_lock(hlock, lock)) {
			if (WARN(!hlock->pin_count, "unpinning an unpinned lock\n"))
				return;

			hlock->pin_count -= cookie.val;

			if (WARN((int)hlock->pin_count < 0, "pin count corrupted\n"))
				hlock->pin_count = 0;

			return;
		}
	}

	WARN(1, "unpinning an unheld lock\n");
}

/*
 * Check whether we follow the irq-flags state precisely:
 */
static void check_flags(unsigned long flags)
{
#if defined(CONFIG_PROVE_LOCKING) && defined(CONFIG_DEBUG_LOCKDEP) && \
    defined(CONFIG_TRACE_IRQFLAGS)
	if (!debug_locks)
		return;

	if (irqs_disabled_flags(flags)) {
		if (DEBUG_LOCKS_WARN_ON(current->hardirqs_enabled)) {
			printk("possible reason: unannotated irqs-off.\n");
		}
	} else {
		if (DEBUG_LOCKS_WARN_ON(!current->hardirqs_enabled)) {
			printk("possible reason: unannotated irqs-on.\n");
		}
	}

	/*
	 * We dont accurately track softirq state in e.g.
	 * hardirq contexts (such as on 4KSTACKS), so only
	 * check if not in hardirq contexts:
	 */
	if (!hardirq_count()) {
		if (softirq_count()) {
			/* like the above, but with softirqs */
			DEBUG_LOCKS_WARN_ON(current->softirqs_enabled);
		} else {
			/* lick the above, does it taste good? */
			DEBUG_LOCKS_WARN_ON(!current->softirqs_enabled);
		}
	}

	if (!debug_locks)
		print_irqtrace_events(current);
#endif
}

void lock_set_class(struct lockdep_map *lock, const char *name,
		    struct lock_class_key *key, unsigned int subclass,
		    unsigned long ip)
{
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	current->lockdep_recursion = 1;
	check_flags(flags);
	if (__lock_set_class(lock, name, key, subclass, ip))
		check_chain_key(current);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_set_class);

void lock_downgrade(struct lockdep_map *lock, unsigned long ip)
{
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	current->lockdep_recursion = 1;
	check_flags(flags);
	if (__lock_downgrade(lock, ip))
		check_chain_key(current);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_downgrade);

/*
 * We are not always called with irqs disabled - do that here,
 * and also avoid lockdep recursion:
 */
void lock_acquire(struct lockdep_map *lock, unsigned int subclass,
			  int trylock, int read, int check,
			  struct lockdep_map *nest_lock, unsigned long ip)
{
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	check_flags(flags);

	current->lockdep_recursion = 1;
	trace_lock_acquire(lock, subclass, trylock, read, check, nest_lock, ip);
	__lock_acquire(lock, subclass, trylock, read, check,
		       irqs_disabled_flags(flags), nest_lock, ip, 0, 0);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_acquire);

void lock_release(struct lockdep_map *lock, int nested,
			  unsigned long ip)
{
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	check_flags(flags);
	current->lockdep_recursion = 1;
	trace_lock_release(lock, ip);
	if (__lock_release(lock, nested, ip))
		check_chain_key(current);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_release);

int lock_is_held_type(const struct lockdep_map *lock, int read)
{
	unsigned long flags;
	int ret = 0;

	if (unlikely(current->lockdep_recursion))
		return 1; /* avoid false negative lockdep_assert_held() */

	raw_local_irq_save(flags);
	check_flags(flags);

	current->lockdep_recursion = 1;
	ret = __lock_is_held(lock, read);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL_GPL(lock_is_held_type);

struct pin_cookie lock_pin_lock(struct lockdep_map *lock)
{
	struct pin_cookie cookie = NIL_COOKIE;
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return cookie;

	raw_local_irq_save(flags);
	check_flags(flags);

	current->lockdep_recursion = 1;
	cookie = __lock_pin_lock(lock);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);

	return cookie;
}
EXPORT_SYMBOL_GPL(lock_pin_lock);

void lock_repin_lock(struct lockdep_map *lock, struct pin_cookie cookie)
{
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	check_flags(flags);

	current->lockdep_recursion = 1;
	__lock_repin_lock(lock, cookie);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_repin_lock);

void lock_unpin_lock(struct lockdep_map *lock, struct pin_cookie cookie)
{
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	check_flags(flags);

	current->lockdep_recursion = 1;
	__lock_unpin_lock(lock, cookie);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_unpin_lock);

#ifdef CONFIG_LOCK_STAT
static int
print_lock_contention_bug(struct task_struct *curr, struct lockdep_map *lock,
			   unsigned long ip)
{
	if (!debug_locks_off())
		return 0;
	if (debug_locks_silent)
		return 0;

	pr_warn("\n");
	pr_warn("=================================\n");
	pr_warn("WARNING: bad contention detected!\n");
	print_kernel_ident();
	pr_warn("---------------------------------\n");
	pr_warn("%s/%d is trying to contend lock (",
		curr->comm, task_pid_nr(curr));
	print_lockdep_cache(lock);
	pr_cont(") at:\n");
	print_ip_sym(ip);
	pr_warn("but there are no locks held!\n");
	pr_warn("\nother info that might help us debug this:\n");
	lockdep_print_held_locks(curr);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();

	return 0;
}

static void
__lock_contended(struct lockdep_map *lock, unsigned long ip)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	struct lock_class_stats *stats;
	unsigned int depth;
	int i, contention_point, contending_point;

	depth = curr->lockdep_depth;
	/*
	 * Whee, we contended on this lock, except it seems we're not
	 * actually trying to acquire anything much at all..
	 */
	if (DEBUG_LOCKS_WARN_ON(!depth))
		return;

	hlock = find_held_lock(curr, lock, depth, &i);
	if (!hlock) {
		print_lock_contention_bug(curr, lock, ip);
		return;
	}

	if (hlock->instance != lock)
		return;

	hlock->waittime_stamp = lockstat_clock();

	contention_point = lock_point(hlock_class(hlock)->contention_point, ip);
	contending_point = lock_point(hlock_class(hlock)->contending_point,
				      lock->ip);

	stats = get_lock_stats(hlock_class(hlock));
	if (contention_point < LOCKSTAT_POINTS)
		stats->contention_point[contention_point]++;
	if (contending_point < LOCKSTAT_POINTS)
		stats->contending_point[contending_point]++;
	if (lock->cpu != smp_processor_id())
		stats->bounces[bounce_contended + !!hlock->read]++;
#ifdef MTK_LOCK_MONITOR
	hlock->acquired = false;
#endif
}

static void
__lock_acquired(struct lockdep_map *lock, unsigned long ip)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	struct lock_class_stats *stats;
	unsigned int depth;
	u64 now, waittime = 0;
	int i, cpu;

	depth = curr->lockdep_depth;
	/*
	 * Yay, we acquired ownership of this lock we didn't try to
	 * acquire, how the heck did that happen?
	 */
	if (DEBUG_LOCKS_WARN_ON(!depth))
		return;

	hlock = find_held_lock(curr, lock, depth, &i);
	if (!hlock) {
		print_lock_contention_bug(curr, lock, _RET_IP_);
		return;
	}

	if (hlock->instance != lock)
		return;

	cpu = smp_processor_id();
	if (hlock->waittime_stamp) {
		now = lockstat_clock();
		waittime = now - hlock->waittime_stamp;
		hlock->holdtime_stamp = now;
	}

	trace_lock_acquired(lock, ip);

	stats = get_lock_stats(hlock_class(hlock));
	if (waittime) {
		if (hlock->read)
			lock_time_inc(&stats->read_waittime, waittime);
		else
			lock_time_inc(&stats->write_waittime, waittime);
	}
	if (lock->cpu != cpu)
		stats->bounces[bounce_acquired + !!hlock->read]++;

	lock->cpu = cpu;
	lock->ip = ip;
#ifdef MTK_LOCK_MONITOR
	hlock->acquired = true;
#endif
}

void lock_contended(struct lockdep_map *lock, unsigned long ip)
{
	unsigned long flags;

	if (unlikely(!lock_stat || !debug_locks) && !lock_mon_enable)
		return;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	check_flags(flags);
	current->lockdep_recursion = 1;
	trace_lock_contended(lock, ip);
	__lock_contended(lock, ip);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_contended);

void lock_acquired(struct lockdep_map *lock, unsigned long ip)
{
	unsigned long flags;

	if (unlikely(!lock_stat || !debug_locks) && !lock_mon_enable)
		return;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	check_flags(flags);
	current->lockdep_recursion = 1;
	__lock_acquired(lock, ip);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(lock_acquired);
#endif

/*
 * Used by the testsuite, sanitize the validator state
 * after a simulated failure:
 */

void lockdep_reset(void)
{
	unsigned long flags;
	int i;

	raw_local_irq_save(flags);
	current->curr_chain_key = 0;
	current->lockdep_depth = 0;
	current->lockdep_recursion = 0;
	memset(current->held_locks, 0, MAX_LOCK_DEPTH*sizeof(struct held_lock));
	nr_hardirq_chains = 0;
	nr_softirq_chains = 0;
	nr_process_chains = 0;
	debug_locks = 1;
	for (i = 0; i < CHAINHASH_SIZE; i++)
		INIT_HLIST_HEAD(chainhash_table + i);
	raw_local_irq_restore(flags);
}

static void zap_class(struct lock_class *class)
{
	int i;

	/*
	 * Remove all dependencies this lock is
	 * involved in:
	 */
	for (i = 0; i < nr_list_entries; i++) {
		if (list_entries[i].class == class)
			list_del_rcu(&list_entries[i].entry);
	}
	/*
	 * Unhash the class and remove it from the all_lock_classes list:
	 */
	hlist_del_rcu(&class->hash_entry);
	list_del_rcu(&class->lock_entry);

	RCU_INIT_POINTER(class->key, NULL);
	RCU_INIT_POINTER(class->name, NULL);
}

static inline int within(const void *addr, void *start, unsigned long size)
{
	return addr >= start && addr < start + size;
}

/*
 * Used in module.c to remove lock classes from memory that is going to be
 * freed; and possibly re-used by other modules.
 *
 * We will have had one sync_sched() before getting here, so we're guaranteed
 * nobody will look up these exact classes -- they're properly dead but still
 * allocated.
 */
void lockdep_free_key_range(void *start, unsigned long size)
{
	struct lock_class *class;
	struct hlist_head *head;
	unsigned long flags;
	int i;
	int locked;

	raw_local_irq_save(flags);
	locked = graph_lock();

	/*
	 * Unhash all classes that were created by this module:
	 */
	for (i = 0; i < CLASSHASH_SIZE; i++) {
		head = classhash_table + i;
		hlist_for_each_entry_rcu(class, head, hash_entry) {
			if (within(class->key, start, size))
				zap_class(class);
			else if (within(class->name, start, size))
				zap_class(class);
		}
	}

	if (locked)
		graph_unlock();
	raw_local_irq_restore(flags);

	/*
	 * Wait for any possible iterators from look_up_lock_class() to pass
	 * before continuing to free the memory they refer to.
	 *
	 * sync_sched() is sufficient because the read-side is IRQ disable.
	 */
	synchronize_sched();

	/*
	 * XXX at this point we could return the resources to the pool;
	 * instead we leak them. We would need to change to bitmap allocators
	 * instead of the linear allocators we have now.
	 */
}

void lockdep_reset_lock(struct lockdep_map *lock)
{
	struct lock_class *class;
	struct hlist_head *head;
	unsigned long flags;
	int i, j;
	int locked;

	raw_local_irq_save(flags);

	/*
	 * Remove all classes this lock might have:
	 */
	for (j = 0; j < MAX_LOCKDEP_SUBCLASSES; j++) {
		/*
		 * If the class exists we look it up and zap it:
		 */
		class = look_up_lock_class(lock, j);
		if (class)
			zap_class(class);
	}
	/*
	 * Debug check: in the end all mapped classes should
	 * be gone.
	 */
	locked = graph_lock();
	for (i = 0; i < CLASSHASH_SIZE; i++) {
		head = classhash_table + i;
		hlist_for_each_entry_rcu(class, head, hash_entry) {
			int match = 0;

			for (j = 0; j < NR_LOCKDEP_CACHING_CLASSES; j++)
				match |= class == lock->class_cache[j];

			if (unlikely(match)) {
				if (debug_locks_off_graph_unlock()) {
					/*
					 * We all just reset everything, how did it match?
					 */
					WARN_ON(1);
				}
				goto out_restore;
			}
		}
	}
	if (locked)
		graph_unlock();

out_restore:
	raw_local_irq_restore(flags);
}

void __init lockdep_init(void)
{
	printk("Lock dependency validator: Copyright (c) 2006 Red Hat, Inc., Ingo Molnar\n");

	printk("... MAX_LOCKDEP_SUBCLASSES:  %lu\n", MAX_LOCKDEP_SUBCLASSES);
	printk("... MAX_LOCK_DEPTH:          %lu\n", MAX_LOCK_DEPTH);
	printk("... MAX_LOCKDEP_KEYS:        %lu\n", MAX_LOCKDEP_KEYS);
	printk("... CLASSHASH_SIZE:          %lu\n", CLASSHASH_SIZE);
	printk("... MAX_LOCKDEP_ENTRIES:     %lu\n", MAX_LOCKDEP_ENTRIES);
	printk("... MAX_LOCKDEP_CHAINS:      %lu\n", MAX_LOCKDEP_CHAINS);
	printk("... CHAINHASH_SIZE:          %lu\n", CHAINHASH_SIZE);

	printk(" memory used by lock dependency info: %lu kB\n",
		(sizeof(struct lock_class) * MAX_LOCKDEP_KEYS +
		sizeof(struct list_head) * CLASSHASH_SIZE +
		sizeof(struct lock_list) * MAX_LOCKDEP_ENTRIES +
		sizeof(struct lock_chain) * MAX_LOCKDEP_CHAINS +
		sizeof(struct list_head) * CHAINHASH_SIZE
#ifdef CONFIG_PROVE_LOCKING
		+ sizeof(struct circular_queue)
#endif
		) / 1024
		);

	printk(" per task-struct memory footprint: %lu bytes\n",
		sizeof(struct held_lock) * MAX_LOCK_DEPTH);
}

static void
print_freed_lock_bug(struct task_struct *curr, const void *mem_from,
		     const void *mem_to, struct held_lock *hlock)
{
	if (!debug_locks_off())
		return;
	if (debug_locks_silent)
		return;

	pr_warn("\n");
	pr_warn("=========================\n");
	pr_warn("WARNING: held lock freed!\n");
	print_kernel_ident();
	pr_warn("-------------------------\n");
	pr_warn("%s/%d is freeing memory %px-%px, with a lock still held there!\n",
		curr->comm, task_pid_nr(curr), mem_from, mem_to-1);
	print_lock(hlock);
	lockdep_print_held_locks(curr);

	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();
}

static inline int not_in_range(const void* mem_from, unsigned long mem_len,
				const void* lock_from, unsigned long lock_len)
{
	return lock_from + lock_len <= mem_from ||
		mem_from + mem_len <= lock_from;
}

/*
 * Called when kernel memory is freed (or unmapped), or if a lock
 * is destroyed or reinitialized - this code checks whether there is
 * any held lock in the memory range of <from> to <to>:
 */
void debug_check_no_locks_freed(const void *mem_from, unsigned long mem_len)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	unsigned long flags;
	int i;

	if (unlikely(!debug_locks))
		return;

	raw_local_irq_save(flags);
	for (i = 0; i < curr->lockdep_depth; i++) {
		hlock = curr->held_locks + i;

		if (not_in_range(mem_from, mem_len, hlock->instance,
					sizeof(*hlock->instance)))
			continue;

		print_freed_lock_bug(curr, mem_from, mem_from + mem_len, hlock);
		break;
	}
	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(debug_check_no_locks_freed);

static void print_held_locks_bug(void)
{
	if (!debug_locks_off())
		return;
	if (debug_locks_silent)
		return;

	pr_warn("\n");
	pr_warn("====================================\n");
	pr_warn("WARNING: %s/%d still has locks held!\n",
	       current->comm, task_pid_nr(current));
	print_kernel_ident();
	pr_warn("------------------------------------\n");
	lockdep_print_held_locks(current);
	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();
}

void debug_check_no_locks_held(void)
{
	if (unlikely(current->lockdep_depth > 0))
		print_held_locks_bug();
}
EXPORT_SYMBOL_GPL(debug_check_no_locks_held);

#ifdef __KERNEL__
void debug_show_all_locks(void)
{
	struct task_struct *g, *p;

	if (unlikely(!debug_locks) && !lock_mon_enable) {
		pr_warn("INFO: lockdep is turned off.\n");
		return;
	}
	pr_warn("\nShowing all locks held in the system:\n");

	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (!p->lockdep_depth)
			continue;
		lockdep_print_held_locks(p);
		touch_nmi_watchdog();
		touch_all_softlockup_watchdogs();
	}
	rcu_read_unlock();

	pr_warn("\n");
	pr_warn("=============================================\n\n");
}
EXPORT_SYMBOL_GPL(debug_show_all_locks);
#endif

/*
 * Careful: only use this function if you are sure that
 * the task cannot run in parallel!
 */
void debug_show_held_locks(struct task_struct *task)
{
	if (unlikely(!debug_locks) && !lock_mon_enable) {
		printk("INFO: lockdep is turned off.\n");
		return;
	}
	lockdep_print_held_locks(task);
}
EXPORT_SYMBOL_GPL(debug_show_held_locks);

asmlinkage __visible void lockdep_sys_exit(void)
{
	struct task_struct *curr = current;

	if (unlikely(curr->lockdep_depth)) {
		if (!debug_locks_off())
			return;
		pr_warn("\n");
		pr_warn("================================================\n");
		pr_warn("WARNING: lock held when returning to user space!\n");
		print_kernel_ident();
		pr_warn("------------------------------------------------\n");
		pr_warn("%s/%d is leaving the kernel with locks still held!\n",
				curr->comm, curr->pid);
		lockdep_print_held_locks(curr);
		lockdep_aee();
	}

	/*
	 * The lock history for each syscall should be independent. So wipe the
	 * slate clean on return to userspace.
	 */
	lockdep_invariant_state(false);
}

void lockdep_rcu_suspicious(const char *file, const int line, const char *s)
{
	struct task_struct *curr = current;

	/* To avoid recursive aee dump in this warning */
	if (!debug_locks_off())
		return;

	/* Note: the following can be executed concurrently, so be careful. */
	pr_warn("\n");
	pr_warn("=============================\n");
	pr_warn("WARNING: suspicious RCU usage\n");
	print_kernel_ident();
	pr_warn("-----------------------------\n");
	pr_warn("%s:%d %s!\n", file, line, s);
	pr_warn("\nother info that might help us debug this:\n\n");
	pr_warn("\n%srcu_scheduler_active = %d, debug_locks = %d\n",
	       !rcu_lockdep_current_cpu_online()
			? "RCU used illegally from offline CPU!\n"
			: !rcu_is_watching()
				? "RCU used illegally from idle CPU!\n"
				: "",
	       rcu_scheduler_active, debug_locks);

	/*
	 * If a CPU is in the RCU-free window in idle (ie: in the section
	 * between rcu_idle_enter() and rcu_idle_exit(), then RCU
	 * considers that CPU to be in an "extended quiescent state",
	 * which means that RCU will be completely ignoring that CPU.
	 * Therefore, rcu_read_lock() and friends have absolutely no
	 * effect on a CPU running in that state. In other words, even if
	 * such an RCU-idle CPU has called rcu_read_lock(), RCU might well
	 * delete data structures out from under it.  RCU really has no
	 * choice here: we need to keep an RCU-free window in idle where
	 * the CPU may possibly enter into low power mode. This way we can
	 * notice an extended quiescent state to other CPUs that started a grace
	 * period. Otherwise we would delay any grace period as long as we run
	 * in the idle task.
	 *
	 * So complain bitterly if someone does call rcu_read_lock(),
	 * rcu_read_lock_bh() and so on from extended quiescent states.
	 */
	if (!rcu_is_watching())
		pr_warn("RCU used illegally from extended quiescent state!\n");

	lockdep_print_held_locks(curr);
	pr_warn("\nstack backtrace:\n");
	dump_stack();
	lockdep_aee();
}
EXPORT_SYMBOL_GPL(lockdep_rcu_suspicious);

#ifdef MTK_LOCK_DEBUG_NEW_DEPENDENCY
/*
 * This is a debug function to catch the lock dependency at runtime.
 * This help to catch the first time of the lock dependency appeared
 * and show the backtrace of the held locks.
 *
 * 1. use "cat /proc/lockdep_chains" to see the full lock name and
 *    which lock dependency you want to catch.
 * 2. According to the order of lock dependency to fill full lock names
 *    into PREV_LOCK_NAME and NEXT_LOCK_NAME.
 * 3. According to the depth of lock dependency to adjust LOCK_DEPTH.
 *    This help to limit the lock dependency scope and increase the
 *    possibility to catch the lock dependency which you want.
 *
 * e.g.
 *    cat /proc/lockdep_chains
 *
 *    irq_context: 0
 *    [ffffff800a1b1860] &(&lockA)->rlock
 *    [ffffff800a1d5b98] &(&lockB)->rlock
 *    [ffffff800b179b40] &(&lockC)->rlock
 *
 *    You have to set as following to catch this dependency.
 *    set PREV_LOCK_NAME as "&(&lockB)->rlock"
 *    set NEXT_LOCK_NAME as "&(&lockC)->rlock"
 *    set LOCK_DEPTH as 2 (because there are 2 locks held before
 *    the final &(&lockC)->rlock)
 *
 *    You can set LOCK_DEPTH as 0 to skip this condition.
 *    Without this condition, you might get more than one dependency.
 *
 * e.g.
 *    irq_context: 0
 *    [ffffff800a1d5b98] &(&lockB)->rlock
 *    [ffffff800b179b40] &(&lockC)->rlock
 *
 *    irq_context: 0
 *    [ffffff800a1d3b90] &(&lockE)->rlock
 *    [ffffff800a1d4b08] &(&lockA)->rlock
 *    [ffffff800a1d5b98] &(&lockB)->rlock
 *    [ffffff800b179b40] &(&lockC)->rlock
 */
#define PREV_LOCK_NAME  "&(&lockB)->rlock"
#define NEXT_LOCK_NAME  "&(&lockC)->rlock"
#define LOCK_DEPTH  0
#ifdef CONFIG_MTK_LOCKING_AEE
//#define MTK_LOCK_DEBUG_NEW_DEPENDENCY_AEE
#endif

static int check_lock_name(struct lock_class *class, const char *lock_name)
{
	char str[KSYM_NAME_LEN];
	const char *name;
	char full_name[128], tmp[16];

	name = class->name;
	if (!name) {
		name = __get_key_name(class->key, str);
		snprintf(full_name, sizeof(full_name), "%s", name);
	} else {
		snprintf(full_name, sizeof(full_name), "%s", name);
		if (class->name_version > 1) {
			snprintf(tmp, sizeof(tmp), "#%d", class->name_version);
			strlcat(full_name, tmp, sizeof(full_name));
		}
		if (class->subclass) {
			snprintf(tmp, sizeof(tmp), "/%d", class->subclass);
			strlcat(full_name, tmp, sizeof(full_name));
		}
	}

	if (!strcmp(full_name, lock_name))
		return 1;

	return 0;
}

static void check_new_dependency(struct held_lock *prev, struct held_lock *next)
{
	if (check_lock_name(hlock_class(prev), PREV_LOCK_NAME) &&
	    check_lock_name(hlock_class(next), NEXT_LOCK_NAME)) {
		if (current->lockdep_depth != LOCK_DEPTH && LOCK_DEPTH > 0)
			return;

		pr_info("[Lockdep] new dependency: (%s) => (%s)\n",
			PREV_LOCK_NAME, NEXT_LOCK_NAME);
		show_stack(current, NULL);
		lockdep_print_held_locks(current);

#ifdef MTK_LOCK_DEBUG_NEW_DEPENDENCY_AEE
		aee_kernel_warning_api(__FILE__, __LINE__,
				       DB_OPT_DUMMY_DUMP,
				       "[Lockdep] new dependency",
				       "\nnew dependency: (%s) => (%s)]\n",
				       PREV_LOCK_NAME, NEXT_LOCK_NAME);
#endif
	}
}
#endif /* MTK_LOCK_DEBUG_NEW_DEPENDENCY */

#if defined(CONFIG_PROVE_LOCKING) || defined(CONFIG_MTK_LOCKING_AEE) || \
	defined(MTK_LOCK_MONITOR) || defined(MTK_LOCK_DEBUG_HELD_LOCK)

#define MAX_LOCK_NAME  128
static void get_lock_name(struct lock_class *class, char name[MAX_LOCK_NAME])
{
	char str[KSYM_NAME_LEN];
	const char *lock_name;
	char name_version[8] = { 0 };
	char subclass[8] = { 0 };

	lock_name = class->name;
	if (!lock_name) {
		lock_name = __get_key_name(class->key, str);
		snprintf(name, MAX_LOCK_NAME, "%s", lock_name);
	} else {
		if (class->name_version > 1)
			snprintf(name_version, 8, "#%d", class->name_version);
		if (class->subclass)
			snprintf(subclass, 8, "/%d", class->subclass);
		snprintf(name, MAX_LOCK_NAME, "%s%s%s",
			lock_name, name_version, subclass);
	}
}
#endif

#ifdef CONFIG_PROVE_LOCKING /* MTK_LOCK_LOG_TO_FTRACE */
static bool is_log_lock_held(struct task_struct *curr)
{
	int i, depth = curr->lockdep_depth;
	struct held_lock *hlock = curr->held_locks;
	struct lockdep_map *instance;

	if (!depth)
		return false;

	for (i = 0; i < depth; i++) {
		instance = (hlock + i)->instance;
		/* used by pr_xxx */
		if (!strcmp(instance->name, "logbuf_lock"))
			return true;
		/* used by console */
		if (!strcmp(instance->name, "&(&port->lock)->rlock"))
			return true;
		if (!strcmp(instance->name, "&port_lock_key"))
			return true;
	}
	return false;
}

static void lock_dbg(const char *msg, ...)
{
	char buf[256];
	va_list args;

	va_start(args, msg);
	vsnprintf(buf, sizeof(buf), msg, args);
	va_end(args);

	trace_lock_dbg(buf);
}

/*
 * Duplicate debug functions and print log to trace.
 * This prevents deadlock from some special locks.
 */

static void get_lock_name_usage(struct lock_class *class, char *lk_name)
{
	char buf[MAX_LOCK_NAME];
	char usage[LOCK_USAGE_CHARS];

	get_lock_name(class, buf);
	get_usage_chars(class, usage);
	snprintf(lk_name, sizeof(buf), "(%s){%s}", buf, usage);
}

static void trace_lock(struct held_lock *hlock)
{
	char name[MAX_LOCK_NAME];
	/*
	 * We can be called locklessly through debug_show_all_locks() so be
	 * extra careful, the hlock might have been released and cleared.
	 */
	unsigned int class_idx = hlock->class_idx;

	/* Don't re-read hlock->class_idx,
	 * can't use READ_ONCE() on bitfields:
	 */
	barrier();

	if (!class_idx || (class_idx - 1) >= MAX_LOCKDEP_KEYS) {
		lock_dbg("<RELEASED>");
		return;
	}

	get_lock_name_usage(lock_classes + class_idx - 1, name);
	lock_dbg("%s, at: %pS", name, (void *)hlock->acquire_ip);
}

static void lockdep_trace_held_locks(struct task_struct *curr)
{
	int i, j, depth = curr->lockdep_depth;
	struct held_lock *hlock;

	if (!depth) {
		lock_dbg("no locks held by %s/%d.",
			 curr->comm, task_pid_nr(curr));
		return;
	}
	lock_dbg("%d lock%s held by %s/%d/[%ld] on CPU#%d:",
		 depth, depth > 1 ? "s" : "", curr->comm,
		 curr->pid, curr->state, task_cpu(curr));

	hlock = curr->held_locks;
	for (i = 0; i < depth; i++) {
		char buf[256];
		struct stack_trace *trace = &(hlock + i)->trace;

		lock_dbg(" #%d: ", i);
		trace_lock(hlock + i);

		/* show backtrace */
		if (!trace || !trace->entries)
			continue;

		for (j = 0; j < trace->nr_entries; j++) {
			snprintf(buf, sizeof(buf), "%*c%pS", 6, ' ',
				 (void *)trace->entries[j]);
			lock_dbg("%s", buf);
		}
	}
}

static void dump_stack_to_trace(struct stack_trace *trace)
{
	int i;

	if (!trace || !trace->entries)
		return;

	for (i = 0; i < trace->nr_entries; i++)
		lock_dbg("%*c%pS", 7, ' ', (void *)trace->entries[i]);
}

static void dump_curr_stack(void)
{
	struct stack_trace trace;
	unsigned long entries[32];

	trace.nr_entries = 0;
	trace.max_entries = ARRAY_SIZE(entries);
	trace.entries = entries;
	trace.skip = 0;

	save_stack_trace_tsk(current, &trace);
	dump_stack_to_trace(&trace);
}

static noinline int
trace_circular_bug_entry(struct lock_list *target, int depth)
{
	char name[MAX_LOCK_NAME];

	get_lock_name_usage(target->class, name);
	lock_dbg("-> #%u: %s", depth, name);
	dump_stack_to_trace(&target->trace);

	return 0;
}

static noinline int
trace_circular_bug_header(struct lock_list *entry, unsigned int depth,
			  struct held_lock *check_src,
			  struct held_lock *check_tgt)
{
	char name[MAX_LOCK_NAME];
	struct task_struct *curr = current;

	if (debug_locks_silent)
		return 0;

	lock_dbg("======================================================");
	lock_dbg("WARNING: possible circular locking dependency detected");
	lock_dbg("%s %.*s %s", init_utsname()->release,
		 (int)strcspn(init_utsname()->version, " "),
		 init_utsname()->version,
		 print_tainted());
	lock_dbg("------------------------------------------------------");
	lock_dbg("%s/%d is trying to acquire lock:",
		 curr->comm, task_pid_nr(curr));
	trace_lock(check_src);

	lock_dbg("but task is already holding lock:");

	trace_lock(check_tgt);
	lock_dbg("which lock already depends on the new lock.");
	lock_dbg("the existing dependency chain (in reverse order) is:");

	get_lock_name_usage(entry->class, name);
	lock_dbg("-> #%u: %s", depth, name);
	dump_stack_to_trace(&entry->trace);
	lockdep_aee();

	return 0;
}

static noinline int
trace_circular_bug(struct lock_list *this,
		   struct lock_list *target,
		   struct held_lock *check_src,
		   struct held_lock *check_tgt,
		   struct stack_trace *trace)
{
	struct task_struct *curr = current;
	struct lock_list *parent;
	struct lock_list *first_parent;
	int depth;

	if (!debug_locks_off_graph_unlock() || debug_locks_silent)
		return 0;

	if (!save_trace(&this->trace))
		return 0;

	depth = get_lock_depth(target);

	trace_circular_bug_header(target, depth, check_src, check_tgt);

	parent = get_lock_parent(target);
	first_parent = parent;

	while (parent) {
		trace_circular_bug_entry(parent, --depth);
		parent = get_lock_parent(parent);
	}

	lock_dbg("other info that might help us debug this:");
	lockdep_trace_held_locks(curr);

	lock_dbg("current backtrace:");
	dump_curr_stack();

	return 0;
}
#endif /* CONFIG_PROVE_LOCKING */

#ifdef CONFIG_MTK_LOCKING_AEE
/* check all locks held by the current task */
bool is_critical_lock_held(void)
{
	int cpu;
	int i, j;
	struct rq *rq;
	struct held_lock *hlock;
	struct lock_class *class;
	char name[MAX_LOCK_NAME];
	unsigned int class_idx;

	/* check if current rq->lock is held by someone */
	cpu = raw_smp_processor_id();
	rq = cpu_rq(cpu);

	if (raw_spin_is_locked(&rq->lock))
		return true;

	/* check locks held by current task */
	if (!current->lockdep_depth)
		return false;

	for (i = 0; i < current->lockdep_depth; i++) {

		hlock = current->held_locks + i;
		class_idx = hlock->class_idx;

		/* Don't re-read hlock->class_idx */
		barrier();

		if (!class_idx || (class_idx - 1) >= MAX_LOCKDEP_KEYS)
			continue;

		class = lock_classes + class_idx - 1;
		get_lock_name(class, name);

		/* check if critical locks are held */
		for (j = 0; j < ARRAY_SIZE(critical_lock_list); j++)
			if (!strcmp(name, critical_lock_list[j]))
				return true;
	}

	return false;
}

static void lockdep_aee(void)
{
	char aee_str[64];

	if (!is_critical_lock_held()) {
		snprintf(aee_str, sizeof(aee_str),
			"[%s]LockProve Warning", current->comm);
		aee_kernel_warning_api(__FILE__, __LINE__,
			DB_OPT_DUMMY_DUMP | DB_OPT_FTRACE,
			aee_str, "LockProve Debug\n");
	}
}

#ifdef MTK_LOCK_MONITOR
static void lock_monitor_aee(void)
{
	char aee_str[40];

	if (is_critical_lock_held())
		return;

	snprintf(aee_str, sizeof(aee_str),
		 "[%s]held locks too much", current->comm);
	aee_kernel_warning_api(__FILE__, __LINE__,
			       DB_OPT_DUMMY_DUMP | DB_OPT_FTRACE,
			       aee_str, "Lock Monitor Warning\n");
}
#endif
#else
static void lockdep_aee(void) {}
#endif /* CONFIG_MTK_LOCKING_AEE */

#if defined(MTK_LOCK_MONITOR) || defined(MTK_LOCK_DEBUG_HELD_LOCK)
static unsigned long long sec_high(unsigned long long nsec)
{
	do_div(nsec, 1000000000);
	return nsec;
}

static unsigned long sec_low(unsigned long long nsec)
{
	/* exclude part of nsec */
	return do_div(nsec, 1000000000) / 1000;
}

#define T_KERNEL    0x1
#define T_FTRACE    0x2
#define T_DEFERRED  0x4
#define T_SRAM      0x8
#define T_BOTH      (T_KERNEL | T_FTRACE)
#define M_LOCK_MON  0x100  /* dump trace by lock monitor */

#define add_suffix(out)	(out) == T_SRAM ? "\n" : ""
#define MAX_WARN_MSG 120
static unsigned int warn_msgs;

static void lock_mon_msg(char *buf, int out)
{
	if (out & T_FTRACE)
		trace_lock_monitor(buf);
	/* check warn_msgs to avoid printing too much log */
	if (out & T_KERNEL && warn_msgs++ < MAX_WARN_MSG) {
		pr_info("%s\n", buf);
		if (warn_msgs == MAX_WARN_MSG)
			pr_info(" See SYS_FTRACE for more log\n");
	}
#ifdef CONFIG_MTK_AEE_IPANIC
	if (out & T_SRAM)
		aee_sram_fiq_log(buf);
#endif
}

#ifndef GO_UNWIND_FRAME
static void dump_task_stack(struct task_struct *tsk, int output)
{
	struct stack_trace trace;
	unsigned long entries[32];
	char buf[256];
	int i;

	if (!mutex_trylock(&tsk->signal->cred_guard_mutex))
		return;

	trace.nr_entries = 0;
	trace.max_entries = ARRAY_SIZE(entries);
	trace.entries = entries;
	trace.skip = 1;
	save_stack_trace_tsk(tsk, &trace);

	if (trace.nr_entries != 0 &&
	    trace.entries[trace.nr_entries - 1] == ULONG_MAX)
		trace.nr_entries--;

	if (trace.nr_entries < 3) {
		mutex_unlock(&tsk->signal->cred_guard_mutex);
		return;
	}

	snprintf(buf, sizeof(buf), "  stack of %s/%d:%s",
		 tsk->comm, tsk->pid, (output == TO_SRAM) ? "\n" : "");
	lock_mon_msg(buf, output);

	for (i = 0; i < trace.nr_entries; i++) {
		snprintf(buf, sizeof(buf), "%*c%pS%s", 6, ' ',
			 (void *)entries[i], (output == TO_SRAM) ? "\n" : "");
		lock_mon_msg(buf, output);
	}

	mutex_unlock(&tsk->signal->cred_guard_mutex);
}
#else
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
static void dump_task_stack(struct task_struct *tsk, int output)
{
	struct stackframe frame;
	char buf[256];
	int i = 0;

	if (!tsk || !try_get_task_stack(tsk))
		return;

#ifdef CONFIG_ARM64
	if (tsk == current) {
		frame.fp = (unsigned long)__builtin_frame_address(0);
		frame.pc = (unsigned long)dump_task_stack;
	} else {
		frame.fp = thread_saved_fp(tsk);
		frame.pc = thread_saved_pc(tsk);
	}
#else
	if (tsk == current) {
		frame.fp = (unsigned long)__builtin_frame_address(0);
		frame.sp = current_stack_pointer;
		frame.lr = (unsigned long)__builtin_return_address(0);
		frame.pc = (unsigned long)dump_task_stack;

	} else {
		frame.fp = thread_saved_fp(tsk);
		frame.sp = thread_saved_sp(tsk);
		frame.lr = 0;
		frame.pc = thread_saved_pc(tsk);
	}
#endif

	snprintf(buf, sizeof(buf), "  stack of %s/%d:%s",
		 tsk->comm, tsk->pid, add_suffix(output));
	lock_mon_msg(buf, output);

	while (1) {
		if (i++ >= 2)
			snprintf(buf, sizeof(buf), "%*c%pS%s", 6, ' ',
				 (void *)frame.pc, add_suffix(output));
#ifdef CONFIG_ARM64
		if (unwind_frame(tsk, &frame) < 0)
			break;
#else
		if (unwind_frame(&frame) < 0)
			break;
#endif
		else if (i > 2)
			lock_mon_msg(buf, output);
	}

	put_task_stack(tsk);
}
#else /* CONFIG_ARM || CONFIG_ARM64 */
static void dump_task_stack(struct task_struct *tsk, int output) {}
#endif
#endif
#endif /* MTK_LOCK_MONITOR || MTK_LOCK_DEBUG_HELD_LOCK */

#ifdef MTK_LOCK_DEBUG_HELD_LOCK
static void
held_lock_save_trace(struct stack_trace *trace, unsigned long *entries)
{
	trace->nr_entries = 0;
	trace->max_entries = HELD_LOCK_STACK_TRACE_DEPTH;
	trace->entries = entries;
	trace->skip = 2;

	save_stack_trace(trace);

	if (trace->nr_entries != 0 &&
	    trace->entries[trace->nr_entries - 1] == ULONG_MAX)
		trace->nr_entries--;
	if (trace->nr_entries > 0)
		trace->nr_entries--;
}

static void held_lock_show_trace(struct held_lock *hlock, int output)
{
	int i;
	char buf[256], ts[32] = "";

	if (!hlock || !hlock->trace.entries)
		return;

	if (!(output & M_LOCK_MON))
		snprintf(ts, sizeof(ts), " from [%lld.%06lu]",
			 sec_high(hlock->timestamp),
			 sec_low(hlock->timestamp));

	for (i = 0; i < hlock->trace.nr_entries; i++) {
		snprintf(buf, sizeof(buf), "%*c%pS%s%s", 6, ' ',
			 (void *)hlock->trace.entries[i],
			 (i == 0) ? ts : "", add_suffix(output));
		lock_mon_msg(buf, output);
	}
}

static void lockdep_print_held_locks(struct task_struct *p)
{
	int i, depth = READ_ONCE(p->lockdep_depth);
	struct held_lock *hlock;
	struct lock_class *class;
	char name[MAX_LOCK_NAME];

	if (!depth)
		pr_info("no locks held by %s/%d.\n", p->comm, task_pid_nr(p));
	else
		pr_info("%d lock%s held by %s/%d:\n", depth,
			depth > 1 ? "s" : "", p->comm, task_pid_nr(p));
	/*
	 * It's not reliable to print a task's held locks if it's not sleeping
	 * and it's not the current task.
	 * But we still think the information from running tasks are valuable.
	 */
	if (p->state == TASK_RUNNING && p != current && 0)
		return;

	/* MTK_LOCK_DEBUG_HELD_LOCK */
	dump_task_stack(p, T_KERNEL);

	for (i = 0; i < depth; i++) {
		hlock = p->held_locks + i;
		class = hlock_class(hlock);
		get_lock_name(class, name);

		pr_info("#%d: (%s), at: [<%p>] %pS\n", i, name,
			(void *)hlock->acquire_ip, (void *)hlock->acquire_ip);
		/* MTK_LOCK_DEBUG_HELD_LOCK */
		held_lock_show_trace(hlock, T_KERNEL);
	}
}
#endif /* MTK_LOCK_DEBUG_HELD_LOCK */

#ifndef CONFIG_LOCK_STAT
#ifdef MTK_LOCK_MONITOR
static void
set_acquired(struct lockdep_map *lock, unsigned long ip, bool acquired)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	unsigned int depth = curr->lockdep_depth;
	int i;

	if (DEBUG_LOCKS_WARN_ON(!depth))
		return;

	hlock = find_held_lock(curr, lock, depth, &i);
	if (!hlock)
		return;

	if (hlock->instance != lock)
		return;

	hlock->acquired = acquired;
}

static void
set_lock_acquired(struct lockdep_map *lock, unsigned long ip, bool acquired)
{
	unsigned long flags;

	if (unlikely(current->lockdep_recursion))
		return;

	raw_local_irq_save(flags);
	check_flags(flags);
	current->lockdep_recursion = 1;
	set_acquired(lock, ip, acquired);
	current->lockdep_recursion = 0;
	raw_local_irq_restore(flags);
}

void lock_contended(struct lockdep_map *lock, unsigned long ip)
{
	set_lock_acquired(lock, ip, false);
}

void lock_acquired(struct lockdep_map *lock, unsigned long ip)
{
	set_lock_acquired(lock, ip, true);
}
#else /* !MTK_LOCK_MONITOR */
void lock_contended(struct lockdep_map *lock, unsigned long ip) {}
void lock_acquired(struct lockdep_map *lock, unsigned long ip) {}
#endif
#endif /* !CONFIG_LOCK_STAT */

#ifdef MTK_LOCK_MONITOR

#ifdef MTK_ENG_BUILD
#define MAX_WARN_LOCKS 5
#else
#define MAX_WARN_LOCKS 100
#endif
static char buf_lock[256];

#ifdef MTK_ENG_BUILD
static unsigned int lock_mon_1st_th_ms = 4000; /* show name */
static unsigned int lock_mon_2nd_th_ms = 6000; /* show backtrace */
static unsigned int lock_mon_3rd_th_ms = 16000; /* trigger Kernel API dump */
static unsigned int lock_mon_period_ms = 10000; /* check held locks */
static unsigned int lock_mon_period_cnt = 10; /* show more information */
#else
static unsigned int lock_mon_1st_th_ms = 2000; /* show name */
static unsigned int lock_mon_2nd_th_ms = 5000; /* show backtrace */
static unsigned int lock_mon_3rd_th_ms = 15000; /* trigger Kernel API dump */
static unsigned int lock_mon_period_ms = 2500; /* check held locks */
static unsigned int lock_mon_period_cnt = 4; /* show more information */
#endif
static unsigned int lock_mon_door;
static unsigned int warn_locks;
static bool lock_mon_aee = 1;

static const char * const held_lock_white_list[] = {
	"&tty->ldisc_sem",
	"&ldata->atomic_read_lock",
	"&f->f_pos_lock",
	"&p->lock",
	"&of->mutex",
	"&epfile->mutex",
	"&session->notif_wait_lock",
	"(&item_q->work)",
	"\"%s\"\"cmdq_flushq\"",
	"\"events\"",                /* aee workqueue */
	"(wq_completion)\"events\""  /* aee workqueue */
};

static inline struct lock_class *
__look_up_lock_class(struct lockdep_map *lock, unsigned int subclass)
{
	struct lockdep_subclass_key *key;
	struct hlist_head *hash_head;
	struct lock_class *class;
	bool is_static = false;

	if (unlikely(subclass >= MAX_LOCKDEP_SUBCLASSES))
		return NULL;

	/*
	 * Static locks do not have their class-keys yet - for them the key
	 * is the lock object itself. If the lock is in the per cpu area,
	 * the canonical address of the lock (per cpu offset removed) is
	 * used.
	 */
	if (unlikely(!lock->key)) {
		unsigned long can_addr, addr = (unsigned long)lock;

		if (__is_kernel_percpu_address(addr, &can_addr))
			lock->key = (void *)can_addr;
		else if (__is_module_percpu_address(addr, &can_addr))
			lock->key = (void *)can_addr;
		else if (static_obj(lock))
			lock->key = (void *)lock;
		else
			return ERR_PTR(-EINVAL);
		is_static = true;
	}

	/*
	 * NOTE: the class-key must be unique. For dynamic locks, a static
	 * lock_class_key variable is passed in through the mutex_init()
	 * (or spin_lock_init()) call - which acts as the key. For static
	 * locks we use the lock object itself as the key.
	 */
	BUILD_BUG_ON(sizeof(struct lock_class_key) >
			sizeof(struct lockdep_map));

	key = lock->key->subkeys + subclass;

	hash_head = classhashentry(key);

	/*
	 * We do an RCU walk of the hash, see lockdep_free_key_range().
	 */
	if (DEBUG_LOCKS_WARN_ON(!irqs_disabled()))
		return NULL;

	hlist_for_each_entry_rcu(class, hash_head, hash_entry) {
		if (class->key == key) {
			/* Don't check class->name and lock->name. */
			return class;
		}
	}

	return NULL;
}

static struct lock_class *
__register_lock_class(struct lockdep_map *lock, unsigned int subclass,
		      int force)
{
	struct lockdep_subclass_key *key;
	struct hlist_head *hash_head;
	struct lock_class *class;

	DEBUG_LOCKS_WARN_ON(!irqs_disabled());

	class = __look_up_lock_class(lock, subclass);
	if (likely(!IS_ERR_OR_NULL(class)))
		goto out_set_class_cache;

	/*
	 * Debug-check: all keys must be persistent!
	 */
	if (IS_ERR(class))
		return NULL;

	key = lock->key->subkeys + subclass;
	hash_head = classhashentry(key);

	arch_spin_lock(&lockdep_lock);
	current->lockdep_recursion++;

	/*
	 * We have to do the hash-walk again, to avoid races
	 * with another CPU:
	 */
	hlist_for_each_entry_rcu(class, hash_head, hash_entry) {
		if (class->key == key)
			goto out_unlock_set;
	}

	/*
	 * Allocate a new key from the static array, and add it to
	 * the hash:
	 */
	if (nr_lock_classes >= MAX_LOCKDEP_KEYS) {
		arch_spin_unlock(&lockdep_lock);
		return NULL;
	}
	class = lock_classes + nr_lock_classes++;
	debug_atomic_inc(nr_unused_locks);
	class->key = key;
	class->name = lock->name;
	class->subclass = subclass;
	INIT_LIST_HEAD(&class->lock_entry);
	INIT_LIST_HEAD(&class->locks_before);
	INIT_LIST_HEAD(&class->locks_after);
	class->name_version = count_matching_names(class);
	/*
	 * We use RCU's safe list-add method to make
	 * parallel walking of the hash-list safe:
	 */
	hlist_add_head_rcu(&class->hash_entry, hash_head);
	/*
	 * Add it to the global list of classes:
	 */
	list_add_tail_rcu(&class->lock_entry, &all_lock_classes);

out_unlock_set:
	current->lockdep_recursion--;
	arch_spin_unlock(&lockdep_lock);

out_set_class_cache:
	if (!subclass || force)
		lock->class_cache[0] = class;
	else if (subclass < NR_LOCKDEP_CACHING_CLASSES)
		lock->class_cache[subclass] = class;

	/*
	 * Hash collision, did we smoke some? We found a class with a matching
	 * hash but the subclass -- which is hashed in -- didn't match.
	 */
	if (DEBUG_LOCKS_WARN_ON(class->subclass != subclass))
		return NULL;

	return class;
}

static void __add_held_lock(struct lockdep_map *lock, unsigned int subclass,
			    int trylock, int read, int check, int hardirqs_off,
			    struct lockdep_map *nest_lock, unsigned long ip,
			    int references, int pin_count)
{
	struct task_struct *curr = current;
	struct lock_class *class = NULL;
	struct held_lock *hlock;
	unsigned int depth;
	int class_idx;

	if (subclass < NR_LOCKDEP_CACHING_CLASSES)
		class = lock->class_cache[subclass];

	if (unlikely(!class)) {
		class = __register_lock_class(lock, subclass, 0);
		if (!class)
			return;
	}
	atomic_inc((atomic_t *)&class->ops);

	depth = curr->lockdep_depth;
	if (depth >= MAX_LOCK_DEPTH)
		return;

	class_idx = class - lock_classes + 1;

	hlock = curr->held_locks + depth;

	hlock->class_idx = class_idx;
	hlock->acquire_ip = ip;
	hlock->instance = lock;
	hlock->nest_lock = nest_lock;
	hlock->irq_context = task_irq_context(curr);
	hlock->trylock = trylock;
	hlock->read = read;
	hlock->check = check;
	hlock->hardirqs_off = !!hardirqs_off;
	hlock->references = references;
#ifdef CONFIG_LOCK_STAT
	hlock->waittime_stamp = 0;
	hlock->holdtime_stamp = lockstat_clock();
#endif
	hlock->pin_count = pin_count;

#ifdef MTK_LOCK_DEBUG_HELD_LOCK
	held_lock_save_trace(&hlock->trace, hlock->entries);
#endif

	/* MTK_LOCK_MONITOR */
	hlock->timestamp = sched_clock();

	curr->lockdep_depth++;
}

static void __del_held_lock(struct lockdep_map *lock)
{
	struct task_struct *curr = current;
	struct held_lock *hlock;
	unsigned int depth;
	int i;

	depth = curr->lockdep_depth;
	if (depth <= 0)
		return;

	hlock = find_held_lock(curr, lock, depth, &i);
	if (!hlock)
		return;

	/* MTK_LOCK_MONITOR */
	hlock->timestamp = 0;

	curr->lockdep_depth = i;
}

static void
lockdep_check_held_locks(struct task_struct *curr, bool en, bool aee)
{
	int i, show_task_stack = 1;
	struct held_lock *hlock;
	unsigned long long timestamp_bak;

	if (!curr->lockdep_depth)
		return;

	for (i = 0; i < curr->lockdep_depth; i++) {
		unsigned long long t_diff;

		hlock = curr->held_locks + i;
		if (hlock->timestamp == 0)
			continue;

		timestamp_bak = hlock->timestamp;
		t_diff = sched_clock() - timestamp_bak;
		do_div(t_diff, 1000000);

		if (hlock->timestamp == 0 || hlock->timestamp != timestamp_bak)
			continue;

		if (t_diff > lock_mon_1st_th_ms || aee) {
			struct lock_class *class;
			char name[MAX_LOCK_NAME];
			unsigned int class_idx = hlock->class_idx;
			bool skip_this = 0;
			int j, output1, output2 = T_FTRACE | M_LOCK_MON;
			int list_num = ARRAY_SIZE(held_lock_white_list);

			/* Don't re-read hlock->class_idx */
			barrier();

			if (!class_idx || (class_idx - 1) >= MAX_LOCKDEP_KEYS)
				continue;

			class = lock_classes + class_idx - 1;
			get_lock_name(class, name);

			/* check white list */
			for (j = 0; j < list_num; j++) {
				if (!strcmp(name, held_lock_white_list[j]))
					skip_this = 1;
			}

			/* ignore special cases */
			if (!strncmp(name, "s_active", 8))
				skip_this = 1;
			if (!strncmp(name, "kn->count", 9)) {
				struct kernfs_node *kn;

				kn = container_of(hlock->instance,
						  struct kernfs_node,
						  dep_map);

				if (kn && kn->name &&
				    !strncmp(kn->name, "wakeup_count", 12))
					skip_this = 1;
			}

			/* skip this warning */
			if (skip_this)
				continue;

			output1 = aee ? T_SRAM : T_BOTH;

			/* locks might be released in runtime */
			if (hlock->timestamp == 0 ||
			    hlock->timestamp != timestamp_bak)
				continue;

			warn_locks++;
			snprintf(buf_lock, sizeof(buf_lock),
				 "[%p] (%s) %s by %s/%d[%c] on CPU#%d from [%lld.%06lu]%s",
				 hlock->instance, name,
				 hlock->acquired ? "held" : "needed",
				 curr->comm, curr->pid,
				 task_state_to_char(curr), task_cpu(curr),
				 sec_high(hlock->timestamp),
				 sec_low(hlock->timestamp),
				 add_suffix(output1));
			lock_mon_msg(buf_lock, output1);

			if (t_diff > lock_mon_2nd_th_ms && en)
				output2 = T_BOTH | M_LOCK_MON;
			output2 = aee ? T_SRAM : output2;

			/* Catch the condition that too many locks are held */
			if (lock_mon_aee && warn_locks >= MAX_WARN_LOCKS) {
				lock_mon_aee = 0;
#ifdef CONFIG_MTK_LOCKING_AEE
				lock_monitor_aee();
#endif
			}

			/* locks might be released in runtime */
			if (hlock->timestamp == 0 ||
			    hlock->timestamp != timestamp_bak)
				continue;

#ifdef MTK_LOCK_DEBUG_HELD_LOCK
			held_lock_show_trace(hlock, output2);
#endif
			/* locks might be released in runtime */
			if (hlock->timestamp == 0 ||
			    hlock->timestamp != timestamp_bak)
				continue;

			if (show_task_stack && curr->state != TASK_RUNNING) {
				dump_task_stack(curr, output2);
				show_task_stack = 0;
			}
		}
	}
}

void mt_aee_dump_held_locks(void)
{
	struct task_struct *g, *p;
	int count = 10;
	int unlock = 1;
	char buf[32];

retry:
	if (!read_trylock(&tasklist_lock)) {
		if (count) {
			count--;
			mdelay(200);
			goto retry;
		}
		unlock = 0;
	}

	snprintf(buf, sizeof(buf), "===== Lock Monitor =====\n");
	lock_mon_msg(buf, TO_SRAM);

	do_each_thread(g, p) {
		if (p->lockdep_depth)
			lockdep_check_held_locks(p, 0, 1);
		if (!unlock)
			if (read_trylock(&tasklist_lock))
				unlock = 1;
	} while_each_thread(g, p);

	if (unlock)
		read_unlock(&tasklist_lock);

	snprintf(buf, sizeof(buf), "\n");
	lock_mon_msg(buf, TO_SRAM);
}

static DEFINE_SPINLOCK(lock_mon_lock);
void check_held_locks(int force)
{
	struct task_struct *g, *p;
	int count = 10;
	int unlock = 1;

	if (!spin_trylock(&lock_mon_lock))
		return;

	/*
	 * Here we try to get the tasklist_lock as hard as possible,
	 * if not successful after 2 seconds we ignore it (but keep
	 * trying). This is to enable a debug printout even if a
	 * tasklist_lock-holding task deadlocks or crashes.
	 */
retry:
	if (!read_trylock(&tasklist_lock)) {
		if (count) {
			count--;
			mdelay(200);
			goto retry;
		}
		unlock = 0;
	}

	warn_locks = 0;
	warn_msgs = 0;

	do_each_thread(g, p) {
		if (p->lockdep_depth)
			lockdep_check_held_locks(p, force, 0);
		if (!unlock)
			if (read_trylock(&tasklist_lock))
				unlock = 1;
	} while_each_thread(g, p);

	if (unlock)
		read_unlock(&tasklist_lock);

	spin_unlock(&lock_mon_lock);
}
EXPORT_SYMBOL_GPL(check_held_locks);

static void show_debug_locks_state(void)
{
	static unsigned long long pre_time_sec;
	unsigned long long time_sec = sec_high(sched_clock());

	/* check debug_locks per 15 seconds */
	if (debug_locks || time_sec - pre_time_sec < 15)
		return;
	pre_time_sec = time_sec;

	pr_info("debug_locks is off at [%lld.%06lu]\n",
		sec_high(debug_locks_off_ts),
		sec_low(debug_locks_off_ts));

	if (nr_lock_classes >= MAX_LOCKDEP_KEYS ||
	    nr_list_entries >= MAX_LOCKDEP_ENTRIES ||
	    nr_stack_trace_entries >= MAX_STACK_TRACE_ENTRIES - 1)
		pr_info("lock_classes[%lu] list_entries[%lu] stack_trace_entries[%lu]\n",
			nr_lock_classes, nr_list_entries,
			nr_stack_trace_entries);
#ifdef CONFIG_PROVE_LOCKING
	if (nr_lock_chains >= MAX_LOCKDEP_CHAINS ||
	    nr_chain_hlocks > MAX_LOCKDEP_CHAIN_HLOCKS)
		pr_info("lock_chains[%lu] chain_hlocks[%d]\n",
			nr_lock_chains, nr_chain_hlocks);
#endif
}

static int lock_monitor_work(void *data)
{
	static int count;
	int force;

	while (lock_mon_enable) {
		force = 0;

		/* print backtrace or not */
		if (++count >= lock_mon_period_cnt) {
			count = 0;
			force = 1;
		}

		if (!debug_locks)
			show_debug_locks_state();
		check_held_locks(force);

		msleep(lock_mon_period_ms);
	}

	return 0;
}

static ssize_t
lock_mon_enable_write(struct file *filp, const char *ubuf,
		      size_t cnt, loff_t *data)
{
	char buf[64];
	int ret, lock_mon_disabled = !lock_mon_enable;

	if (!lock_mon_door)
		return cnt;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt] = 0;

	ret = kstrtouint(buf, 10, &lock_mon_enable);
	if (ret)
		return ret;

	lock_mon_enable = !!lock_mon_enable;
	if (lock_mon_disabled && lock_mon_enable)
		kthread_run(lock_monitor_work, NULL, "lock_monitor");

	return cnt;
}

static ssize_t
lock_mon_enable_read(struct file *file, char __user *user_buf,
		     size_t count, loff_t *ppos)
{
	char buf[32];
	int len;

	len = snprintf(buf, sizeof(buf), "%d\n", lock_mon_enable);
	len = len > sizeof(buf) ? sizeof(buf) : len;
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations lock_mon_enable_fops = {
	.open = simple_open,
	.read = lock_mon_enable_read,
	.write = lock_mon_enable_write,
};

#define DECLARE_LOCK_MONITOR_MATCH(name, param) \
static ssize_t lock_mon_##name##_write(struct file *filp, \
	const char *ubuf, size_t cnt, loff_t *data) \
{ \
	char buf[64]; \
	int ret; \
	\
	if (!lock_mon_door) \
		return cnt; \
	\
	if (cnt >= sizeof(buf)) \
		return -EINVAL; \
	\
	if (copy_from_user(&buf, ubuf, cnt)) \
		return -EFAULT; \
	\
	buf[cnt] = 0; \
	\
	ret = kstrtouint(buf, 10, &(param)); \
	if (ret) \
		return ret; \
	\
	return cnt; \
} \
\
static ssize_t lock_mon_##name##_read(struct file *file, \
	char __user *user_buf, size_t count, loff_t *ppos) \
{ \
	char buf[32]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%d\n", param); \
	len = len > sizeof(buf) ? sizeof(buf) : len; \
	return simple_read_from_buffer(user_buf, count, ppos, buf, len); \
} \
\
static const struct file_operations lock_mon_##name##_fops = { \
	.open = simple_open, \
	.read = lock_mon_##name##_read, \
	.write = lock_mon_##name##_write, \
}

DECLARE_LOCK_MONITOR_MATCH(period, lock_mon_period_ms);
DECLARE_LOCK_MONITOR_MATCH(1st_th, lock_mon_1st_th_ms);
DECLARE_LOCK_MONITOR_MATCH(2nd_th, lock_mon_2nd_th_ms);
DECLARE_LOCK_MONITOR_MATCH(3rd_th, lock_mon_3rd_th_ms);
DECLARE_LOCK_MONITOR_MATCH(period_cnt, lock_mon_period_cnt);

static ssize_t
lock_monitor_door_write(struct file *filp, const char *ubuf,
			size_t cnt, loff_t *data)
{
	char buf[8];

	if (cnt >= sizeof(buf) || cnt <= 1UL)
		return cnt;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt - 1UL] = 0;

	if (strncmp("open", buf, 4) == 0)
		lock_mon_door = 1;
	if (strncmp("close", buf, 5) == 0)
		lock_mon_door = 0;

	return cnt;
}

static const struct file_operations lock_mon_door_fops = {
	.open = simple_open,
	.write = lock_monitor_door_write,
};

/*
 * Lock monitor checks the hold time of all held locks periodly.
 * Type of spinlock, mutex, rw_semaphore, and RCU are supported.
 * Type of semaphore and rt_mutex are not supported.
 */
static int __init lock_monitor_init(void)
{
	struct proc_dir_entry *root;

	root = proc_mkdir("lockmon", NULL);
	if (!root)
		return -ENOMEM;
	proc_create("door", 0220, root, &lock_mon_door_fops);
	proc_create("enable", 0664, root, &lock_mon_enable_fops);
	proc_create("period_ms", 0664, root, &lock_mon_period_fops);
	proc_create("1st_th_ms", 0664, root, &lock_mon_1st_th_fops);
	proc_create("2nd_th_ms", 0664, root, &lock_mon_2nd_th_fops);
	proc_create("3rd_th_ms", 0664, root, &lock_mon_3rd_th_fops);
	proc_create("period_cnt", 0664, root, &lock_mon_period_cnt_fops);

	lock_mon_enable = 1;
	kthread_run(lock_monitor_work, NULL, "lock_monitor");

#ifdef CONFIG_MTK_BOOT
	if (get_boot_mode() == META_BOOT)
		debug_locks_off();
#endif
	return 0;
}
subsys_initcall(lock_monitor_init);
#else /* !MTK_LOCK_MONITOR */
#ifdef CONFIG_LOCKDEP
void check_held_locks(int force) {}
void mt_aee_dump_held_locks(void) {}
#endif
#endif
