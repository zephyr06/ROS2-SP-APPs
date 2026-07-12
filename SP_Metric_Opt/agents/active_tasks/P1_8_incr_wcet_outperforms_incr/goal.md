# P1.8 — INCR_WCET outperforms INCR in the eval-suite run (theoretically implausible)

**Priority:** P1 (investigation — a Q3-gate-relevant anomaly; user: "this is
theoretically highly unlikely, need to investigate why and fix this issue")
**Status:** FILED 2026-07-12 (task folder + investigation STARTED — read-only
root-cause analysis; **NO implementation**). The user's two-part directive:
"first add the task, then share investigation while working on finding reasons
and root causes." This file is the task; the running root-cause log is in
`dev_log.md`. The findings so far are recorded below under **Investigation
(preliminary)**; they are a hypothesis, not yet confirmed end-to-end.

## The user's directive (verbatim)

> "in [evalsuite_run_test_dur600_interval10_seed1000_tasks4/sim/
> tasks4_dur600_interval10_seed1000], INCR_WCET outperforms other INCR tasks,
> this is theoretically highly unlikely, need to investigate why and fix this
> issue. first add the task, then share investigation while working on finding
> reasons and root causes"

## The anomaly (the data)

`comparison_summary.csv` for the run (N=4, dur=600s, interval=10s, seed=1000,
10 tasksets averaged):

| Scheduler | Mean_SP_Metric | Mean_Miss_Rate | Important_Miss_Rate |
|-----------|---------------:|---------------:|--------------------:|
| INCR          | 0.552747 | 0.195486 | 0.430000 |
| INCR_Reopt_1  | 0.571043 | 0.194476 | 0.430000 |
| INCR_Reopt_5  | 0.554137 | 0.195486 | 0.430000 |
| INCR_Reopt_10 | 0.552747 | 0.195486 | 0.430000 |
| INCR_Reopt_30 | 0.549053 | 0.195486 | 0.430000 |
| INCR_Reopt_60 | 0.545583 | 0.195486 | 0.430000 |
| **INCR_WCET** | **0.599168** | **0.111513** | **0.281667** |
| BF            | 0.612907 | 0.021834 | 0.168333 |
| RM            | 0.429412 | 0.000000 | 0.000000 |
| CFS           | 0.427563 | 0.000000 | 0.000000 |
| INCR_NO_TL    | 0.440006 | 0.301885 | 0.500000 |

**INCR_WCET (0.5992) beats every plain INCR arm** (0.5456–0.5710), including
`INCR_Reopt_1` (the max-reopt arm, 0.5710). It sits just below BF (0.6129) and
— most strikingly — has a **lower miss rate** (0.1115 vs 0.1955) and a **lower
important-miss rate** (0.2817 vs 0.4300) than the entire INCR family.

## Why this is theoretically implausible (the expected ordering)

`INCR_WCET` is an **ablation** of `INCR`, not a smarter optimizer. In dispatch
(`SimulationOrchestrator.cpp:332-339`) it does exactly one thing differently
from `INCR`: it sets `GlobalVariables::use_wcet_execution_time = true` for the
duration of the optimizer call, then restores it. `ApplyWCETAblationIfRequired`
(`OptimizeSP_TL_Incre.cpp:482-492`) then, for every task, **collapses the ET
distribution to a deterministic point mass at `max_time`**:
```cpp
task.execution_time_dist = GetUnitExecutionTimeDist(max_et);  // {{max_et, 1.0}}
task.setExecGaussian(GaussianDist(max_et, 0.01));
task.setExecutionTime(max_et);
```
`GetUnitExecutionTimeDist` (`Probability.h:173`) is `{{time_limit, 1.0}}` — a
single-value distribution. So `INCR_WCET` throws away the ET distribution and
optimizes against a degenerate WCET-only signal. It is explicitly a **degraded
baseline**:

- The Q3 gate (`evaluation_suite.py:94`, `Q3_BASELINES = ["RM","CFS",
  "INCR_NO_TL","INCR_WCET"]`) asserts `INCR >= max(Q3_BASELINES)` at large N —
  i.e. INCR is *expected* to beat INCR_WCET. The test fixtures
  (`tests/python/test_evaluation_suite.py:292`) encode
  `(8, "INCR_WCET"): {"mean_sp_norm": 0.55}` with INCR above it.
- `INCR_WCET` is the **amnesiac-by-construction** sibling: it cannot exploit the
  ET distribution shape, so by any sound theory it should be weakly dominated
  by `INCR` (which sees the full distribution + the same optimizer).

A degraded ablation beating the real optimizer is the signature of either (a)
the real optimizer committing a *structurally bad* configuration on some
tasksets, or (b) the WCET ablation accidentally side-stepping a bug in the
scoring/optimization path. Both are defects. **This is why the user flagged it
as "theoretically highly unlikely."**

## User's reasoning to evaluate (added 2026-07-12)

> "UNDER incr_wcet, SINCE TASKS'S et IS more pessimistic, tasks' RTA is more
> pessimistic, therefore, during time limit optimization, it is much more likely
> that time limit configurations will adopt fast time limit and therefore less
> SP values. follow this reasoning, evaluate whether it's indeed the case for
> INCR_WCET."

This is a concrete, falsifiable channel — **the pessimistic-RTA channel**:

```
use_wcet_execution_time=true
  → ApplyWCETAblationIfRequired collapses every task's ET dist to a point mass
    at max_et (GetUnitExecutionTimeDist(max_et))            [OptimizeSP_TL_Incre.cpp:482-492]
  → ProbabilisticRTA over the point-mass ET dist is MORE pessimistic
    (interference = full max_et at every release, no Gaussian thinning)
  → the optimizer, scoring against this pessimistic RTA, is pushed toward
    ADOPTING SMALLER (tighter) time limits (to buy back RT headroom)
  → tighter TLs ⇒ lower performance coefficient ⇒ lower SP
```

**Critical observation about this reasoning:** it predicts **INCR_WCET ≤ INCR** (the
WCET arm should score *less* SP because it adopts tighter TLs). The data is the
**opposite** — INCR_WCET (0.5992) > INCR (0.5527) aggregate, and on taskset_1
INCR_WCET (0.383) >> INCR (0.0133). So either (a) the pessimistic-RTA channel is
real but is *dominated* by a second effect that pushes the other way, or (b) the
channel does not actually fire on taskset_1 because **most tasks have no TL freedom
to give up** (see Finding 4 — tasks 0/1/2 have `{-1}`-only TL grids; only task 3
has `timePerformancePairs`). The evaluation below (Finding 5) tests both.

The reasoning is **correct in mechanism** (the WCET collapse IS more pessimistic
for the RTA — confirmed in source, `ApplyWCETAblationIfRequired` →
`GetUnitExecutionTimeDist(max_et)` feeds `ProbabilisticRTA_TaskSet`). The question
is whether it is the *dominant* effect on taskset_1, or whether a different channel
(the analytic-vs-schedulability gap on the no-TL-freedom WCET>deadline tasks)
overwhelms it. **Verdict recorded in Finding 5.**

## Investigation (preliminary — recorded 2026-07-12; hypothesis, NOT yet confirmed)

### Finding 1 — the aggregate is dominated by ONE catastrophic taskset

Per-taskset `sp_metrics_summary.txt` (the whole-run analytical SP per taskset):

| taskset | INCR     | INCR_WCET | who wins |
|---------|---------:|----------:|----------|
| 0 | 0.672293 | 0.660612 | INCR ✓ |
| **1** | **0.013331** | **0.383344** | **INCR_WCET (INCR ≈ 0!)** |
| 2 | 0.500000 | 0.500000 | tie |
| 3 | 0.514286 | 0.514286 | tie |
| 4 | 0.556118 | 0.551353 | INCR ✓ |
| 5 | 0.659167 | 0.659167 | tie |
| 6 | 0.513865 | 0.566667 | INCR_WCET |
| 7 | 0.562474 | 0.562474 | tie |
| 8 | 0.548263 | 0.606097 | INCR_WCET |
| 9 | 0.987678 | 0.987678 | tie |

**On 8 of 10 tasksets INCR ≥ INCR_WCET or ties** (exactly as theory predicts).
The aggregate flip is driven almost entirely by **taskset_1**, where INCR's SP
collapses to **0.0133** (essentially zero) while INCR_WCET holds at 0.383.
Taskset_6 and taskset_8 contribute smaller wins for INCR_WCET, but taskset_1
alone (a 0.37 SP gap × 1/10 weight ≈ 0.037 aggregate gap) accounts for the bulk
of the 0.047 aggregate difference (0.5992 − 0.5527).

**Reframing:** the anomaly is *not* "INCR_WCET is mysteriously great." It is
**"INCR catastrophically fails on taskset_1 (and mildly on 6, 8), while the
degraded INCR_WCET ablation survives those same tasksets."** That is a much
more pointed root-cause question.

### Finding 2 — the failure is in the ANALYTIC SP, not the actual schedule

On taskset_1, the actual schedule misses **zero** deadlines under both arms:
- `INCR/INCR/miss_rate_summary.txt`: `47470,0,0` — total_jobs=47470, missed=0.
- `INCR_WCET/INCR_WCET/miss_rate_summary.txt`: `45079,0,0` — missed=0.

Yet INCR's *analytical* SP per interval collapses. The `interval_sp_metrics.txt`
trajectory on taskset_1:
- **INCR**: interval 0 = 0.0578, oscillates near 0, then **0 from interval 52
  through 59** (the last 8 intervals are all literal `0`).
- **INCR_WCET**: steady at ~0.40 early, ~0.38 late — never collapses.

This is consistent with the SP being **analytic** (`ObtainSP_TaskSet_And_TimeLimits`
→ `ProbabilisticRTA_TaskSet`, as traced in P2.6's goal doc): the RTA *predicts*
near-100% DDL-miss chance for some task under INCR's chosen config, even though
the actual `RunQueue` schedule misses nothing. The analytic-vs-sim disconnection
that P2.6 addresses is exactly what makes this kind of failure invisible in the
miss-rate columns.

### Finding 3 — taskset_1 contains inherently-unschedulable tasks (WCET > deadline)

`taskset_characteristics_interval_0.yaml` for taskset_1 shows:
- **task 0**: `period=33, deadline=18, ET_mu=27.22, ET_max=27.22` (sigma=1.0 →
  effectively deterministic). **WCET (27.22) > deadline (18)** — the task's own
  execution time exceeds its deadline.
- **task 2**: `period=20, deadline=14, ET_mu=21.40, ET_max=21.40`. **WCET
  (21.40) > deadline (14)** — same pattern.

A task with WCET > deadline is *analytically unschedulable* in the RTA sense
unless a **time limit strictly less than the deadline** is adopted: the SP path
`ApplyTimeLimitsToTasksExecutionTime` (`SP_Metric.cpp:70-80`) replaces the task's
ET dist with `GetUnitExecutionTimeDist(time_limits[i])` — so the scored ET
*becomes* the adopted TL. If TL ≥ deadline → deterministic miss →
`GetDDL_MissProbability` → ~1 → `SP_Func(1, threshold)` = 0 for that task → SP
contribution 0 (and cascades through interference to other tasks).

### The leading hypothesis (to confirm)

**INCR's optimizer, on taskset_1, adopts a time-limit ≥ the deadline for an
inherently-unschedulable task (task 0 and/or task 2), driving the analytic SP to
~0 and getting stuck there (compare-and-keep cannot escape because every
neighboring config also scores ~0 — a flat-zero basin). INCR_WCET avoids this
because `ApplyWCETAblationIfRequired` collapses the ET dist to the WCET point
mass *before* the TL search, which changes the `timePerformancePairs` option grid
and/or the perf-coefficient the optimizer sees, leading it to adopt a TL < deadline
(or a config the RTA scores as schedulable).**

Sub-hypotheses to disambiguate (NOT yet confirmed):
- **H1a (TL-grid):** the WCET ablation changes which TL options are enumerated
  (`timePerformancePairs` are built from the ET dist), so INCR_WCET's grid
  contains a TL < deadline that INCR's grid lacks (or vice versa).
- **H1b (perf-coefficient / SP_Func shape):** with a deterministic ET dist, the
  perf coefficient `GetPerfCoefficient()` and the miss-probability integral behave
  differently than under the Gaussian — a TL just under deadline yields a much
  better SP under the degenerate dist than under the real Gaussian (whose tail
  still crosses the deadline).
- **H1c (incumbent corruption — the P1.1/P1.2 family):** INCR commits a bad
  permutation early (interval 0–6, where SP is already oscillating near 0) and
  the carried incumbent + compare-and-keep can't recover; INCR_WCET, with a
  flatter scoring landscape, never falls in. This would make P1.8 a sibling of
  P1.2 (reopt incumbent degradation) and P1.1 (residual investigation).
- **H1d (generator defect):** a task with WCET > deadline is itself a
  generator-level infeasibility (the `per_core_cpu_util` calibration producing
  an unschedulable taskset). If so, the root cause is upstream of the optimizer
  — INCR is faithfully reporting that the taskset is unschedulable, and the
  "fix" is the generator, not the optimizer. (P1.7's prerequisite fix touched
  exactly this calibration; see the P1.7 memory.)

These are NOT mutually exclusive. H1d may be the underlying generator defect,
and H1a/H1b the reason the WCET ablation masks it.

### Finding 4 — STRUCTURAL: only task 3 has TL freedom in taskset_1 (recorded 2026-07-12)

`RecordTimeLimitOptions` (`OptimizeSP_TL_BF.cpp:18-34`) enumerates a task's TL
options **directly from its `timePerformancePairs`** — one option per pair. If a
task has **no** `timePerformancePairs`, the only TL option is the sentinel `{-1}`
("apply no time limit"; `OptimizeSP_TL_BF.cpp:29-31`, mirrored in
`RecordCloseTimeLimitOptions` at `OptimizeSP_TL_Incre.cpp:75-77`). `{-1}` means
`ApplyTimeLimitsToTasksExecutionTime` leaves the ET dist untouched
(`SP_Metric.cpp:73-77`: only `time_limits[i] != -1` triggers the substitution).

In `taskset_characteristics_interval_*.yaml` for taskset_1, **only task 3 carries
`performance_records_time`/`performance_records_perf`** (the `5.0…90.0` /
`0.1…1.0` ladder). Tasks 0, 1, 2 have **none** — confirmed across sampled intervals
0, 10, 20, 30, 40, 50, 52, 59 (always exactly 1 task with records = task 3).

**Consequence (refutes the original H1a as stated):** INCR **cannot** adopt a TL <
deadline for the WCET>deadline tasks (0 and 2) — it has no TL option for them
other than `-1` (the raw ET dist). The "INCR adopts TL ≥ deadline" mechanism in
the leading hypothesis is **wrong as written**: INCR adopts `-1` (no TL), and the
scored ET for tasks 0/2 is the **raw Gaussian/deterministic dist**, whose tail (or
point mass, since sigma=1.0 ≈ deterministic) sits at/above the deadline. So the
SP→0 mechanism is: **RTA over the raw (unlimited) ET dist of a WCET>deadline task
predicts ~100% DDL miss → `SP_Func(~1, threshold)` ≈ 0.** It is NOT a TL ≥ deadline
being adopted; it is the *absence* of any adoptable TL that could rescue the task.

This is symmetric across INCR and INCR_WCET: the ablation collapses the ET dist
(`ApplyWCETAblationIfRequired`), it does **not** add `timePerformancePairs`. So
both arms see `{-1}`-only TL grids for tasks 0/1/2. The TL-grid channel (H1a)
**cannot** be the differentiator on taskset_1.

### Finding 5 — evaluating the user's pessimistic-RTA reasoning (recorded 2026-07-12; CORRECTED)

The user's channel (WCET collapse → more pessimistic RTA → optimizer adopts
tighter TLs → less SP) is **mechanistically correct** —
`ApplyWCETAblationIfRequired` → `GetUnitExecutionTimeDist(max_et)` feeds
`ProbabilisticRTA_TaskSet`, and a point-mass-at-WCET interference term is the
maximally pessimistic case (vs a Gaussian whose tail thins). **But it predicts the
wrong sign on taskset_1:** it says INCR_WCET ≤ INCR, and the data is INCR_WCET
(0.383) >> INCR (0.0133). So the channel is either not firing here, or is
dominated by a larger opposite-sign effect. Tracing which:

1. **Does the WCET collapse reach the *scored* SP, or only the optimizer's
   search?** — **CORRECTION (was wrong in the first draft): the WCET collapse is
   CONFINED to the optimizer's internal search; it does NOT reach the scored
   metric.** `Optimize_w_TL_ScratchOrIncre` takes `const DAG_Model&
   dag_tasks_update` (`OptimizeSP_TL_Incre.h:87`) — by const reference. Inside,
   `dag_tasks_ = dag_tasks_update;` (`OptimizeSP_TL_Incre.cpp:313` / `:452`) is a
   **value copy** into the member `dag_tasks_` (DAG_Model has value semantics:
   `TaskSet tasks` is a value member, `DAG_Model.h:112`), and
   `ApplyWCETAblationIfRequired(dag_tasks_)` mutates that **copy**, not the
   orchestrator's `dag_tasks`. The scored SP at `SimulationOrchestrator.cpp:553`
   (`ObtainSP_TaskSet_And_TimeLimits(dag_tasks.tasks, sp_parameters,
   time_limits)`) reads the orchestrator's `dag_tasks_vecs_[interval]` — the
   **uncollapsed** original. `ApplyTaskConfigurations` (`:507`) only sets
   `priority` (from `res.priority_vec`) and `setExecutionTime(avg)` — it does NOT
   touch `execution_time_dist`. **So the scored ET dist is the original Gaussian
   (truncated at max_et) for BOTH arms.** The pessimistic-RTA channel the user
   describes operates ONLY on the optimizer's internal objective (which TL/PA it
   commits to); the metric itself is scored on the original dists.

2. **On taskset_1, is the "adopt tighter TL" channel even available?** For tasks
   0/1/2 (no `timePerformancePairs`), there is **no tighter TL to adopt** — the
   grid is `{-1}`. So the pessimistic-RTA channel can only act on **task 3** (the
   one task with a TL ladder). On task 3, INCR_WCET would indeed be pushed toward
   a tighter TL and a lower perf-coef — a *small* SP *decrease* on task 3. That is
   the user's channel firing, and it is real but **minor** (one task, partial
   weight) and **wrong sign** (it lowers INCR_WCET's scored SP, not raises it).

3. **How, then, do the arms differ in the scored SP?** ONLY through (a) the
   adopted `time_limits` vector (only task 3 can differ; 0/1/2 are `-1` both
   arms) and (b) the adopted `priority_vec` (PA — can differ across ALL tasks, and
   IS baked into the scored `dag_tasks.tasks[*].priority` by
   `ApplyTaskConfigurations` at `:507`, which `ProbabilisticRTA_TaskSet` reads).
   The ET dist is identical across arms. **So the dominant differentiator is the
   PRIORITY ASSIGNMENT**, not the TL. This re-points the root cause from H1a/H1b
   (TL-grid / perf-coef) toward **H1c (search-landscape / incumbent)**: INCR and
   INCR_WCET commit to different PAs because their internal search objectives
   differ (INCR_WCET's collapsed landscape is flatter / lands on a different
   basin), and INCR's committed PA catastrophically worsens the RTA interference
   on the WCET>deadline tasks 0/2 → their response-time dists blow past the
   deadline → `SP_Func(miss≈1, threshold)` goes **negative** (PenaltyFunc
   `-0.01·exp(10·|threshold−vp|)` is unbounded below) → SP contribution strongly
   negative → aggregate near 0.

4. **Hand-model check (node terms only, ignoring chain/path):** task 0 (vp≈1,
   threshold 0.4) → `SP_Func(1, 0.4)` = `interpolate(PenaltyFunc(1,0.4), min, 0,
   max, 1)`; `PenaltyFunc(1, 0.4) = -0.01·exp(10·0.6) = -0.01·exp(6) ≈ -4.03`;
   min_val = `PenaltyFunc(1, 0.4)` itself = -0.546... **wait** — min_val is
   `PenaltyFunc(1, threshold)` = -0.546 (for threshold 0.4: -0.01·exp(10·0.6) ≈
   -4.03, NOT -0.546; the -0.546 was for threshold 0.6). So `SP_Func(1, 0.4) ≈
   interpolate(-4.03, -4.03, 0, 0.47, 1) = (-4.03−(-4.03))/(0.47−(−4.03)) = 0`...
   which gives SP_0 = 0, not negative. **The hand-model is ambiguous on the sign
   of the catastrophe** — needs the actual RTA response-time dist (interference
   included), not the raw ET point mass. The certain-miss floor gives SP_Func=0
   (not negative) because min_val = PenaltyFunc(1, threshold) is the floor by
   construction. So a vp of *exactly* 1 yields SP_Func = 0, but a vp slightly
   below 1 with the penalty branch can dip below 0. **This means the catastrophe
   is NOT "task 0 contributes a huge negative" — it's "task 0 contributes ~0 and
   its interference pushes OTHER tasks' vp from <threshold to >threshold,
   dropping them from +positive to 0/negative."** The PA controls who interferes
   with whom → PA is the lever. (NEEDS the actual per-task RTA under each arm's
   PA to confirm — a debug dump of `rtas[i]` per interval per arm.)

### Finding 6 — the unschedulable-task problem is SYSTEMIC (recorded 2026-07-12)

The user flagged a second generator issue on taskset_1: task id=2 has
`execution_time_mu=21.40` while `period=20` — the **average** ET exceeds the
period. The user: "i should have explicitly requested that tasks's avg ET cannot
exceed a certain portion of period." Sweeping all 10 generated tasksets
(`taskset_characteristics_interval_0.yaml`, 40 tasks) to see how widespread this
is:

| violation | tasks affected | tasksets affected |
|---|---|---|
| `mu > period` (the user's exact flag) | 2/40 | 2/10 (ts1 t2: 21.4/20; ts7 t2: 518.1/500) |
| `execution_time_max > deadline` (WCET>DDL, Finding 3's substrate) | ~18/40 | **9/10** (only ts9 is clean) |
| `mu > deadline` with σ=1.0 (deterministic certain-miss — the lethal combo) | 5/40 | 5/10 (ts0 t3, ts1 t0+t2, ts5 t3, ts7 t3) |

**Verdict: the unschedulable-task problem is systemic, not a one-off.** The user
flagged the `mu > period` case; the broader `WCET > deadline` case is ~9× more
prevalent and was already Finding 3's substrate. A WCET>DDL task is analytically
unschedulable unless a TL strictly < deadline is adopted — and **most of these
tasks have no `timePerformancePairs`** (no TL ladder), so no TL can rescue them.
**This strongly supports H1d (generator defect).**

**H1d CONFIRMED in generator source** (`Gen_Taskset/lib/taskset_generator.py`,
called via `run_full_generation_pipeline` in `Gen_Taskset/lib/orchestrator.py`
from `simulation_experiments/compare_optimizers.py:451-480`). Two compounding
defects, both with **zero feasibility guard** (grep-confirmed across
`Gen_Taskset/lib/` — no `deadline > et` comparison, no reject/regenerate, no
clamp; the only `while True` at `orchestrator.py:133` is a path-file index scan):

1. **Deadline drawn INDEPENDENTLY of ET** — `taskset_generator.py:533`:
   `deadline = int(round(period * random.uniform(0.5, 1.0)))`. The deadline/
   period *ratio* is bounded (0.5–1.0), but deadline-vs-ET is not. Direct cause
   of `mu > deadline` (taskset_1 t0: mu=27.2, deadline=18; t2: mu=21.4,
   deadline=14).
2. **WCET = `et_mean ± 2*sigma` with no cap** — `taskset_generator.py:598-599`:
   for non-perf tasks, `execution_time_max = max(1.0, et_mean + 2*sigma)`. For
   env tasks (large sigma) this can blow past period AND deadline; for
   deterministic tasks (sigma≈1.0, the certain-miss tasks) `max ≈ mu > deadline`.
   No check that `execution_time_max ≤ deadline` (or `≤ period`).

The UUniFast step (`:429`, `:496` `et_mean = max(1.0, u_i * period)`, `u_i <
MAX_UTIL_PER_TASK=0.95`) bounds the *mean* below period in principle — but it is
undone by (1) the independent deadline draw and (2) the ±2σ WCET. The `mu >
period` cases (the user's flag, 2/40) arise when UUniFast's `max_util_cap`
fallback (`:163-164`: raises the cap when `required_min_cap >= cap`) fires under
high total util (`CPU_UTIL_RANDOM_RANGE=[0.5,1.5]` × `N_CORES=2` → up to 3.0
total across 4 tasks). `per_core_cpu_util` calibration = `:393` (`random.uniform`
over `CPU_UTIL_RANDOM_RANGE`), `cpu_util = per_core_cpu_util * n_cores` (`:394`).
So the util draw is the upstream knob, but the *unschedulability* is the missing
deadline-vs-ET and WCET-vs-deadline guard, not the util value per se.

**Necessary-but-not-sufficient (refines H1c vs H1d):** WCET>DDL is endemic
(9/10 tasksets) yet only taskset_1 collapses INCR to 0.0133. So WCET>DDL alone
does not cause the INCR collapse — the PA/interference topology (H1c) is what
makes taskset_1 catastrophic. **H1d is the underlying generator defect (the
substrate); H1c is the mechanism of the collapse on top of it.** They are not
mutually exclusive (as the hypothesis section already notes) — the cleanest read
is: the generator produces unschedulable tasksets (H1d), and INCR's PA search
then falls into a flat-zero basin on the specific topology of taskset_1 (H1c),
while INCR_WCET's flatter search landscape happens to avoid that basin. A
generator fix (F1) would remove the substrate entirely; an incumbent-escape fix
(F3) would treat the collapse mechanism on tasksets that remain unschedulable
by design (if the user decides some stress is intentional).

**Cross-recorded to P1.7** (`finished_tasks/P1_7_cpu_partition_mismatch/goal.md`,
"Follow-up note — generator feasibility, the D1 separate pass"): P1.7's D1
always deferred the generator-calibration pass as a separate decision after the
simulator fix; this is that pass, now active under P1.8's D1. P1.7 stays
RESOLVED (its simulator partitioning fix is correct and standalone).

## F1 fix design (proposed 2026-07-12 — NOT implemented; awaits user greenlight)

The generator source is now fully traced (see `dev_log.md` 2026-07-12 cont'd
entry). The two H1d defects are confirmed end-to-end, and the WCET>DDL
catastrophe is traced all the way to the scored SP (`RegularTasks.cpp:77-79`
makes `execution_time_max` the scored `FiniteDist` support bound; `RTA.cpp` +
`Probability.cpp:145-157` `CompressDeadlineMissProbability` lumps all
above-deadline mass into a single certain-miss bin → `SP_Func` floors at 0).
So the tasksets are **genuinely analytically unschedulable**, INCR's ≈0 on
taskset_1 is **honest**, and INCR_WCET's 0.383 is the artifact (a PA that
dodges the worst interference). The clean fix is F1 (generator), not F2/F3/F4.

### Where the guard goes

A **deterministic feasibility pass at the end of `generate_taskset_parameters`**
(`Gen_Taskset/lib/taskset_generator.py`), AFTER the deadline draw (`:533`) and
the `execution_time_max` compute (`:598-599`), BEFORE the `tasks_dict_list`
serialization (`:562`). At this point every task has its final `period`,
`deadline`, `et_mean`, `et_sigma`, `execution_time_min/max`, and
`time_limit_task` flag — exactly the fields the guard needs, and the last
chance to repair them before they are baked into the C++-facing YAML.

### What the guard enforces (per task)

1. **Mean bound** (the user's "avg ET ≤ a certain portion of period"):
   `et_mean ≤ k · period`. UUniFast's `MAX_UTIL_PER_TASK=0.95` cap
   (`:429`/`:496`) approximates this in principle but is undone by its own
   fallback (`:163-164` raises the cap under high total util) and by the
   ±2σ WCET. An explicit post-hoc clamp closes the gap. Candidate `k = 0.9`
   (matches `FINAL_Et_OVER_PERIOD_RANGE[1]` and `MAX_UTIL_PER_TASK`, so perf
   and non-perf tasks live under one consistent ET/period ceiling).
2. **WCET bound**: `execution_time_max ≤ deadline`. This is the direct
   unschedulability guard — the scored `FiniteDist` support must not cross
   the deadline. For perf tasks `execution_time_max = period*0.9` already <
   period, but can still exceed a `0.5·period` deadline, so the guard applies
   to them too.

### Three implementation shapes (the user picks — do NOT decide unilaterally)

These differ in *what they preserve* and *how they act*. They are NOT
mutually exclusive; (A)+(B) compose.

- **(A) Clamp-in-place** — repair the offending fields directly:
  - clamp `execution_time_max = min(execution_time_max, deadline)` (shrinks
    the scored ET support; preserves the deadline draw and the UUniFast
    `et_mean`/util vector);
  - if the mean bound fires, clamp `et_mean = min(et_mean, k·period)` and
    recompute `execution_time_min/max` from the clamped mean ± 2σ (or just
    re-clamp `execution_time_max`).
  - *Preserves:* periods, deadline distribution, util vector. *Changes:* the
    ET support of the offending tasks (shrinks toward the deadline). Cheapest,
    fully deterministic, no retry loop, no seed-sensitivity shift beyond the
    clamp.
- **(B) ET-aware deadline draw** — replace `:533`'s
  `deadline = period*uniform(0.5,1.0)` with
  `deadline = uniform(et_max, period)` (drawn AFTER `execution_time_max` is
  known, so `deadline ≥ et_max + ε` by construction). Eliminates the
  WCET>DDL class entirely at the source. *Preserves:* periods, util vector,
  ET distribution. *Changes:* the **deadline distribution** (deadlines
  become ET-aware ⇒ generally larger / correlated with ET ⇒ changes the SP
  threshold landscape the gate fixtures encode). This is the most principled
  fix but the most invasive to the experiment's statistics.
- **(C) Reject-and-regenerate** — if any task violates `et_max > deadline` or
  `et_mean > k·period`, reject the whole taskset and re-run
  `generate_taskset_parameters` with a fresh RNG draw (bounded retry count,
  else fall back to (A) clamp so generation never hard-fails). *Preserves:*
  all distributions (the surviving tasksets are unmodified samples). *Cost:*
  rejection rate is HIGH (9/10 tasksets currently violate) ⇒ many regenerations
  ⇒ shifts the realized sample distribution toward the feasible region
  (survivorship bias) and changes per-seed reproducibility. Use alone is
  risky; best as a safety-net over (A).

### Recommendation (for the user to confirm or override)

**(A) clamp-in-place as the primary guard** + the explicit `et_mean ≤ k·period`
mean bound, `k=0.9`. Rationale: it is the smallest change that removes the
unschedulable substrate, it preserves the deadline distribution and the
UUniFast util vector (so the gate fixtures and the per-seed load sweep stay
meaningful), it is fully deterministic (no retry loop, no survivorship bias),
and it composes with a later (B) if the user wants a more principled
deadline-ET coupling later. (C) is kept only as a defensive backstop, not the
primary mechanism, because of the high rejection rate.

### TDD test (red before green, per `agent_coding_rules.md`)

A new test (extending `test_integration.py` or a new
`Gen_Taskset/tests/test_feasibility.py`) that:
- constructs a config that, on the current code, produces a WCET>DDL task
  (e.g. `CPU_UTIL_RANDOM_RANGE=[1.5,1.5]`, `N_CORES=2`, `N_TASKS=4`,
  `SIGMA_OVER_Et_RANGE=[0.5,0.6]` — high util + large sigma reproduces the
  taskset_1 condition), runs `generate_taskset_parameters`, and asserts
  `execution_time_max ≤ deadline` AND `et_mean ≤ k·period` for EVERY task.
  RED on current code (the sweep in Finding 6 already proved 9/10 violate),
  GREEN after the guard. Also asserts the existing
  `0.5·period ≤ deadline ≤ period` invariant (`test_integration.py:90`)
  stays intact.
- Optionally: a regression test that re-seeds the eval-suite config
  (`taskset_cfg_paper_4.json`) and asserts no task has `et_max > deadline`,
  guarding the shipped configs directly.

### Scope guardrails (what F1 does NOT touch)

- The optimizer (`OptimizeSP_TL_Incre.cpp`), the SP scoring path
  (`SP_Metric.cpp`, `RTA.cpp`), and `ApplyWCETAblationIfRequired` are
  **untouched**. F1 removes the substrate; the H1c collapse mechanism
  (PA search) becomes unreachable on a feasible taskset, so no F2/F3 is
  needed alongside it.
- The Q3 gate fixtures (`tests/python/test_evaluation_suite.py`) may shift
  once the eval-suite tasksets are regenerated under the guard (the SP values
  were partly inflated by the unschedulable-task collapse). Updating those
  fixtures is part of the fix, after the user re-runs the A/B.
- `per_core_cpu_util` / `CPU_UTIL_RANDOM_RANGE` are NOT changed — the load
  sweep is the experiment's point; the guard makes the *consequence* of high
  util feasible, it does not forbid high util.

### Open sub-decisions for the greenlight (the ask)

- **(a)** the `k` in `et_mean ≤ k·period` (recommend 0.9; user's "certain
  portion" — confirm or pick another).
- **(b)** primary mechanism: (A) clamp-in-place [recommended], (B) ET-aware
  deadline draw, or (C) reject-and-regenerate.
- **(c)** if (A): clamp `execution_time_max ≤ deadline` only (preserves
  deadline draw, shrinks ET support) [recommended], or also re-draw
  `deadline = uniform(et_max, period)` (more principled, changes deadline
  distribution)?
- **(d)** D4 re-run scope: regenerate + re-run the N=4 / taskset_1 probe
  first, or the full eval suite?

## F1.2 fix design (USER-GREENLIT 2026-07-12 — supersedes F1's (A)/(B)/(C))

> User replaced the regenerate-loop / ET-aware-deadline draft with a simpler
> post-generation clamp. Verbatim: "after task set and all intervals' yaml
> files are generated, we go through all of them once. for each task whose
> avg ET exceeds 0.95*period, we just clamp avg ET back to 0.95*period. if
> that happens, we also re-generate deadline as period. otherwise, nothing
> changes." The earlier (A)/(B)/(C) shapes and the ≤5-retry regenerate loop
> are DROPPED.

### The pass

A single deterministic transform over the emitted C++-facing YAMLs, run AFTER
`generate_additional_execution_traces` and BEFORE
`run_full_generation_pipeline` returns. New module
`Gen_Taskset/lib/feasibility_clamp.py`:

```
clamp_avg_et_to_period(yaml_dir, et_over_period_cap=0.95):
  for each taskset_characteristics*.yaml in yaml_dir:
    for each task t in tasks:
      if t has performance_records_time (perf task): SKIP   # min/max are TL-grid bounds
      if t.execution_time_mu > cap * t.period:
        t.execution_time_mu      = cap * t.period
        t.execution_time_max     = min(t.execution_time_max, cap * t.period)
        t.execution_time_min     = min(t.execution_time_min, t.execution_time_max)
        t.deadline               = t.period
    rewrite the yaml with SpaceSeparatedListDumper (byte-compatible)
```

`taskset_param.yaml` is NOT touched (the C++ never reads its `Et_mean` for
scoring — only the characteristics YAMLs).

### Why `max`/`min` are clamped too (not just `mu`)

Clamping `mu` alone is a **no-op on the scored SP**. The C++ scores the ET
distribution as `FiniteDist(GaussianDist(mu, sigma), execution_time_min,
execution_time_max, granularity)` (`RegularTasks.cpp:75-79`), and `FiniteDist`
truncates the Gaussian at `execution_time_max` — it bins mass from `min` to
`max` and dumps all upper-tail mass onto the `max` bin
(`Probability.cpp:18-43`). So the scored support's upper bound is
`execution_time_max`, not `mu`. Leaving `max > cap*period` means the scored
support still crosses the period → the unschedulability persists. The `max`
(and `min`, if it exceeds the clamped `max`) clamp is the necessary
consequence of the user's "clamp avg ET" intent reaching the scored metric.

### Why perf tasks are skipped

A perf task's `execution_time_min`/`execution_time_max` are the
**time-limit-option grid bounds** (`period*0.05 .. period*0.9`,
`taskset_generator.py:582-583`, re-asserted at `yaml_exporter.py:74-75`) —
semantically distinct from the ET-distribution support. Clamping them would
corrupt the TL grid the optimizer searches. Verified non-hypothetical:
taskset_7 task 2 is a perf task with `mu=518 > 0.95*500` (`min=25, max=450`
are its grid bounds). → gate the clamp on
`not bool(performance_records_time)`.

### Empirical firing (existing N=4 run, 10 tasksets / 40 tasks)

The clamp fires on **2 tasks** total; **1 is actually clamped** after the perf
gate:
- taskset_1 task 2 (non-env deterministic, `mu=21.4, period=20`) → CLAMPED to
  `mu=max=19.0, deadline=20`. This is the user's original flag.
- taskset_7 task 2 (perf, `mu=518, period=500`) → SKIPPED by the perf gate.
The other 38 tasks already satisfy `mu ≤ 0.95*period`. Surgical: repairs
taskset_1's `mu>period` substrate without disturbing the 9 well-behaved
tasksets.

### Open sub-decision (flagged, NOT blocking — proceeding with the principled choice)

The user said "clamp avg ET" (mu only). I am also clamping `max`/`min` on the
same tasks because mu-only does not change the scored SP (see above). If the
user wants mu-only (a labeling-only change leaving the scored dist intact),
that is a 1-line narrower version — but it would not achieve the stated goal.
Proceeding with mu+max+min + deadline=period; surfaced in the reply.

## Goal

1. **Confirm the root cause** — distinguish H1a/H1b/H1c/H1d with instrumented
   evidence: dump INCR's adopted TLs + priorities on taskset_1 interval-by-
   interval, dump the `timePerformancePairs` grid INCR sees vs the grid
   INCR_WCET sees, and confirm whether the TL ≥ deadline for the unschedulable
   task(s). Reconcile against the zero-actual-miss schedule.
2. **Decide the fix with the user** (do NOT decide unilaterally — per
   `agent_coding_rules.md`). Candidate fixes span very different leverage
   points:
   - **(F1) Generator:** reject / regenerate tasksets with WCET > deadline (or
     clamp so ET_max ≤ deadline). Treats H1d. **Source located
     (`Gen_Taskset/lib/taskset_generator.py`)** — concrete fix shapes: (a) draw
     deadline as `uniform(et_max, period)` after ET is known, so `deadline ≥
     et_max + ε` by construction (`:533` currently draws deadline independent of
     ET); (b) clamp `execution_time_max ≤ deadline` (`:598-599` currently
     uncapped); (c) reject+regenerate the whole taskset if any task has
     `et_max > deadline` or `et_mean > k*period` (the user's "avg ET ≤ a portion
     of period" — also enforce at `:496`/`:525`, where the UUniFast fallback
     `:163-164` can otherwise assign `u_i ≥ 1.0`). (a)+(b) change the
     deadline/ET distributions; (c) preserves them but adds a rejection loop.
     User's framing favors the "avg ET ≤ k·period" clamp.
   - **(F2) Optimizer / SP scoring:** make the optimizer never adopt a TL ≥
     deadline (a hard constraint in the TL search), OR make `SP_Func` /
     `GetDDL_MissProbability` robust to the WCET>deadline case. Treats H1a/H1b.
   - **(F3) Incumbent/compare-and-keep:** if H1c confirms a flat-zero basin
     trap, the fix is the P1.2-class escape heuristic (not P1.8-specific).
   - **(F4) Accept + reframe:** if the taskset is genuinely unschedulable and
     INCR is *correct* to report SP≈0, then INCR_WCET's 0.383 is the *bug*
     (it masks infeasibility), and the fix is to make INCR_WCET honest too.
3. **Only after the fix is chosen + greenlit**: implement (TDD, red before
   green), re-run the A/B, confirm INCR ≥ INCR_WCET on taskset_1 and the
   aggregate, and check whether Q3's verdict moves.

## Why this is P1 (not P0/P2)

- **Not P0** — no crash, no silent correctness corruption of the gate-facing
  metric on the *common* case (8/10 tasksets behave correctly). The eval suite
  still runs; Q3's `INCR >= max(Q3_BASELINES)` is violated on this run, but Q3
  is scoped to large N (N=8/10) and this run is N=4, so the gate itself may not
  be at its verdict-moving sizing here (to confirm in Step 0).
- **Not P2** — it directly concerns the Q3 gate's premise (`INCR >=
  INCR_WCET`) and the credibility of the INCR-vs-ablation story the paper
  tells. A degraded ablation beating the real optimizer, even on one taskset,
  undermines the narrative; left unfixed it will surface in the publication
  figure run (P0.3). It is an active defect, not hygiene.
- **P1** = "investigate + fix; blocks a clean paper claim but not the build."

## Open design decisions (settle with the user BEFORE any fix — do NOT decide unilaterally)

Per `agent_coding_rules.md` ("Ask users if you're not certain about design
choices, don't make design decisions yourself"):

- **D1 — Is a task with WCET > deadline (or avg ET > period) a generator bug or
  an intentional stress case?** This determines whether the fix is F1 (generator)
  or F2/F4 (optimizer/scoring). **Finding 6 (2026-07-12): the problem is
  systemic** — 9/10 generated tasksets have ≥1 WCET>deadline task, 2/10 have
  avg-ET>period (the user's flag), 5/10 have a deterministic certain-miss task
  (mu>DDL, σ=1.0). So this is not a rare stress case to grandfather; the
  generator enforces no feasibility bound. The generator's `per_core_cpu_util`
  calibration (touched by P1.7's D1, which deferred this exact generator pass) is
  the likely source. Confirm with the user whether unschedulable tasks should
  exist by design (→ F2/F4 make the optimizer/scoring robust to them) or whether
  the generator must reject/clamp them (→ F1). Note F1's threshold: clamp
  `ET_max ≤ deadline`, or `mu ≤ k·period` (the user's "certain portion of
  period"), or both?
- **D2 — Should the optimizer be *forbidden* from adopting TL ≥ deadline, or
  is that a legitimate (if low-SP) choice it may make?** I.e. is the defect
  "INCR chose a TL ≥ deadline" (→ F2 hard constraint) or "INCR chose correctly
  but the scoring/compare-and-keep trapped it" (→ F3)? Depends on H1a vs H1c.
- **D3 — Is INCR_WCET's 0.383 the *correct* SP for taskset_1, or is it the
  bug?** If the taskset is genuinely unschedulable, INCR's ≈0 is honest and
  INCR_WCET's 0.383 is the artifact (F4); if the taskset is schedulable and
  INCR is failing to find the config, INCR_WCET's 0.383 is honest and INCR's ≈0
  is the bug (F2/F3). The direction of the fix flips on this.
- **D4 — Scope of the re-run.** After a fix, does the user re-run the full
  eval suite, or just the N=4 / taskset_1 probe first? (User runs
  `run_end_to_end.sh`; standing constraint.)

## Done when

- [ ] Root cause confirmed with instrumented evidence (adopted TLs + priorities
      on taskset_1 for INCR; the TL grid INCR vs INCR_WCET sees; the
      WCET-vs-deadline relationship per task). Hypothesis H1a/b/c/d
      disambiguated.
- [ ] D1–D4 settled with the user.
- [ ] Fix implemented (TDD: a test that reproduces the taskset_1 collapse red,
      then green after the fix) per the chosen F1–F4.
- [ ] User re-runs the A/B; INCR ≥ INCR_WCET on taskset_1 + the aggregate;
      Q3 verdict checked.
- [ ] `agents/overall_tasks.md` + top-level `agents/dev_log.md` updated;
      memory entry added/updated.
- [ ] `git add` staged; user reviews (no commit).

## Out of scope

- **No implementation until the root cause is confirmed AND the user greenlights
  a fix.** The 2026-07-12 work = task filing + read-only investigation.
- `git commit` — user's standing constraint (`git add` only).
- Running the A/B myself — user runs `run_end_to_end.sh`.
- Re-litigating P1.1 (residual ET growth) or P1.2 (reopt incumbent degradation)
  — P1.8 may turn out to be a sibling (H1c), but it asks a distinct question
  (WCET-ablation masking an INCR failure on an unschedulable taskset). If H1c
  confirms overlap, merge the fix; do not assume it up front.
- P2.6 (sim-RT SP) — P2.6 would *surface* this anomaly (the sim SP would be
  nonzero on taskset_1 because the actual schedule misses nothing), but P2.6
  is deferred and is a cross-check, not a fix for the analytic SP collapse.
  P1.8 stands on its own.

## Reference docs

- The run: `simulation_experiments/optimizer_comparison/runs/evalsuite_run_test_dur600_interval10_seed1000_tasks4/sim/tasks4_dur600_interval10_seed1000/comparison_summary.csv` + `taskset_1/`.
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:332-339` — the
  `INCR_WCET` dispatch (sets `use_wcet_execution_time`).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:482-492` —
  `ApplyWCETAblationIfRequired` (collapses ET dist to WCET point mass).
- `sources/Safety_Performance_Metric/SP_Metric.cpp:70-87` —
  `ApplyTimeLimitsToTasksExecutionTime` + `ObtainSP_TaskSet_And_TimeLimits`
  (the TL→scored-ET substitution; the analytic SP path).
- `sources/Safety_Performance_Metric/Probability.h:173` —
  `GetUnitExecutionTimeDist` (the `{{v,1.0}}` point mass).
- `simulation_experiments/evaluation_suite.py:94` — `Q3_BASELINES` (the gate
  that asserts `INCR >= INCR_WCET`).
- Memory [`cpu-partition-mismatch.md`](../../../) (P1.7 — touched the
  `per_core_cpu_util` calibration; relevant to H1d).
- Memory [`sim-rt-based-sp-metric.md`](../../../) (P2.6 — the analytic-vs-sim
  gap that hides this kind of failure in the miss-rate columns).
