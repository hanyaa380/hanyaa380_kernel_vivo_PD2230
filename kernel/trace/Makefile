# SPDX-License-Identifier: GPL-2.0

# Do not instrument the tracer itself:

ifdef CONFIG_FUNCTION_TRACER
ORIG_CFLAGS := $(KBUILD_CFLAGS)
KBUILD_CFLAGS = $(subst $(CC_FLAGS_FTRACE),,$(ORIG_CFLAGS))

ifdef CONFIG_FTRACE_SELFTEST
# selftest needs instrumentation
CFLAGS_trace_selftest_dynamic.o = $(CC_FLAGS_FTRACE)
obj-y += trace_selftest_dynamic.o
endif
endif

ifdef CONFIG_FTRACE_STARTUP_TEST
CFLAGS_trace_kprobe_selftest.o = $(CC_FLAGS_FTRACE)
obj-$(CONFIG_KPROBE_EVENTS) += trace_kprobe_selftest.o
endif

# If unlikely tracing is enabled, do not trace these files
ifdef CONFIG_TRACING_BRANCHES
KBUILD_CFLAGS += -DDISABLE_BRANCH_PROFILING
endif

# for GCOV coverage profiling
ifdef CONFIG_GCOV_PROFILE_FTRACE
GCOV_PROFILE := y
endif

CFLAGS_trace_benchmark.o := -I$(src)
CFLAGS_trace_events_filter.o := -I$(src)

obj-$(CONFIG_TRACE_CLOCK) += trace_clock.o

obj-$(CONFIG_FUNCTION_TRACER) += libftrace.o
obj-$(CONFIG_RING_BUFFER) += ring_buffer.o
obj-$(CONFIG_RING_BUFFER_BENCHMARK) += ring_buffer_benchmark.o

obj-$(CONFIG_TRACING) += trace.o
obj-$(CONFIG_TRACING) += trace_output.o
obj-$(CONFIG_TRACING) += trace_seq.o
obj-$(CONFIG_TRACING) += trace_stat.o
obj-$(CONFIG_TRACING) += trace_printk.o
obj-$(CONFIG_TRACING_MAP) += tracing_map.o
obj-$(CONFIG_PREEMPTIRQ_DELAY_TEST) += preemptirq_delay_test.o
obj-$(CONFIG_CONTEXT_SWITCH_TRACER) += trace_sched_switch.o
obj-$(CONFIG_FUNCTION_TRACER) += trace_functions.o
obj-$(CONFIG_PREEMPTIRQ_TRACEPOINTS) += trace_preemptirq.o
obj-$(CONFIG_IRQSOFF_TRACER) += trace_irqsoff.o
obj-$(CONFIG_PREEMPT_TRACER) += trace_irqsoff.o
obj-$(CONFIG_TRACE_IRQFLAGS) += trace_irqsoff.o
obj-$(CONFIG_SCHED_TRACER) += trace_sched_wakeup.o
obj-$(CONFIG_HWLAT_TRACER) += trace_hwlat.o
obj-$(CONFIG_NOP_TRACER) += trace_nop.o
obj-$(CONFIG_STACK_TRACER) += trace_stack.o
obj-$(CONFIG_MMIOTRACE) += trace_mmiotrace.o
obj-$(CONFIG_FUNCTION_GRAPH_TRACER) += trace_functions_graph.o
obj-$(CONFIG_TRACE_BRANCH_PROFILING) += trace_branch.o
obj-$(CONFIG_BLK_DEV_IO_TRACE) += blktrace.o
ifeq ($(CONFIG_BLOCK),y)
obj-$(CONFIG_EVENT_TRACING) += blktrace.o
endif
obj-$(CONFIG_EVENT_TRACING) += trace_events.o
obj-$(CONFIG_EVENT_TRACING) += trace_export.o
obj-$(CONFIG_FTRACE_SYSCALLS) += trace_syscalls.o
ifeq ($(CONFIG_PERF_EVENTS),y)
obj-$(CONFIG_EVENT_TRACING) += trace_event_perf.o
endif
obj-$(CONFIG_EVENT_TRACING) += trace_events_filter.o
obj-$(CONFIG_EVENT_TRACING) += trace_events_trigger.o
obj-$(CONFIG_HIST_TRIGGERS) += trace_events_hist.o
obj-$(CONFIG_BPF_EVENTS) += bpf_trace.o
obj-$(CONFIG_KPROBE_EVENTS) += trace_kprobe.o
obj-$(CONFIG_TRACEPOINTS) += power-traces.o
ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_TRACEPOINTS) += rpm-traces.o
endif
ifeq ($(CONFIG_TRACING),y)
obj-$(CONFIG_KGDB_KDB) += trace_kdb.o
endif
obj-$(CONFIG_PROBE_EVENTS) += trace_probe.o
obj-$(CONFIG_UPROBE_EVENTS) += trace_uprobe.o

obj-$(CONFIG_TRACEPOINT_BENCHMARK) += trace_benchmark.o

libftrace-y := ftrace.o

obj-$(CONFIG_TRACING) += mtk_trace.o
