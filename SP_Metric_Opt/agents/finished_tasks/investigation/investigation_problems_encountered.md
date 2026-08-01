# Investigation problems, confusions, and dead ends (P25 INCR-ET grows-with-period)

This file records the problems encountered during the P25 "per-activation
execution time grows with `ReoptimizationPeriod`" investigation — the
measurement confusions, the contradictions between different measurement
methods, the fixes that turned out to be inert, and the claims that are
verified vs. unverified. It exists so the next pass does not re-walk the
same dead ends.

Companion files:
- `agents/runtime_profiling_guide.md` — how to measure the two timers yourself.
- `agents/debug_runtime0704_incr.md` — the full running debug log (§10/§11
  contain the claims this file reconciles).
- Memory: `p25-incr-et-grows-with-period.md` (carries the same claims as
  the debug log; needs the same corrections — see §5 below).

All file:line references verified against HEAD `de4e9636`.

---

## Problem 1 — Two timers were conflated (the "14 ms vs 90 ms" gap)

**Symptom.** The instrumented per-interval optimizer time (`opt_ms` in the
`[INCR-ET-DBG]` stderr lines) is ~14 ms for an INCRE interval, but the A/B
tables report ~90 ms/activation for the same arm. These two numbers describe
DIFFERENT quantities and were being compared as if they were the same.

**Verified distinction.**

| | Timer A — per-interval optimizer time | Timer B — whole-process scheduler time |
|---|---|---|
| What it times | only `incr_optimizer_.Optimize_w_TL_ScratchOrIncre(...)` | the entire `RunSimulation()` process, start→end |
| Where it is timed | `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:302-308` | `tests/RunOrchestrator.cpp:83` (start) → `:124` (end) |
| Where it is written | `[INCR-ET-DBG] ... opt_ms=...` on stderr | `scheduler_execution_time.txt` in the output folder |
| Gating | `debugMode == 1` only | always on |
| Per-activation value in A/B tables | NOT this one | this one (whole-process time ÷ interval count) |

Timer A excludes everything `SimulateInterval` does outside the optimizer
call: `ApplyTaskConfigurations`, the run-queue tick loop
(`SimulationOrchestrator.cpp:527-533`, which is O(interval_duration) —
10 000 ticks for a 10 s interval), and `ObtainSP_TaskSet_And_TimeLimits`
at `SimulationOrchestrator.cpp:542` (a SEPARATE safety/performance
evaluation that is NOT counted by `g_incr_et_debug_sp_dag_calls`, because
that counter is incremented only inside `ObtainSP_DAG` at
`sources/Safety_Performance_Metric/SP_Metric.cpp:99`).

**Resolution.** The gap between 14 ms and 90 ms is not a bug; it is
Timer A measuring a strict subset of what Timer B measures. The
non-optimizer per-interval work (run-queue loop + the
`ObtainSP_TaskSet_And_TimeLimits` eval) is the difference. To attribute it
precisely, add the 6-line `sim_ms` timer around `SimulateInterval(...)`
documented in §3 of `runtime_profiling_guide.md`; then
`sim_ms − opt_ms` is exactly the non-optimizer work.

---

## Problem 2 — Three different `ndiff` numbers for the same taskset

The "changed-task count" (number of tasks whose execution-time distribution
differs between two consecutive intervals — called `ndiff` in the debug log)
has been reported as THREE different values by three different methods, and
the record has not reconciled them.

**Method A — ground-truth C++ test on the raw interval YAMLs.**
`tests/testOptimizeIncrePA.cpp` test `FindTaskWithDifferentEt.N10IntervalYamlGroundTruth`
reads `taskset_characteristics_interval_0.yaml` and `interval_1.yaml` for
the N=10 `tasks10_dur600_interval10_seed1000/taskset_0` taskset through the
real `ReadDAG_Tasks` → `FiniteDist` path and calls `FindTaskWithDifferentEt`.
**Verified result (re-run 2026-07-05):**
```
[N10-NDIFF-DBG] dag0->dag1: N=10 ndiff=2 tasks=[4- 9- ]
```
So the raw YAML diff flags **2 tasks** (ids 4 and 9, both decreases), not 8.

**Method B — instrumented runtime (`debugMode==1`).** The
`[INCR-ET-DBG]` instrumentation reports the changed-task count seen by
`OptimizeIncre` at runtime. The debug log (`debug_runtime0704_incr.md` §10)
records this as **≈8** at N=10, modal across the run.

**Method C — prior-session summary.** The compacted summary carries a claim
of "modal ndiff=1, never 8". **This claim is UNSUPPORTED by any evidence I
can find or reproduce.** No instrumented stderr log preserving a modal-1
result is on disk, and the ground-truth test (Method A) gives 2, not 1.
Treat Method C as unverified until someone re-runs the instrumented binary
and attaches the log.

**Why the gap matters.** Method A (2) and Method B (8) differ by 6. The
debug log offers two explanations, neither fully verified:

- *Explanation 1 (§10, baseline lag):* `OptimizeIncre` diffs
  `prev_optimizer_.dag_tasks_` against the current DAG. `prev_optimizer_`
  is only advanced at reopt boundaries (`SeedStateFromIncumbent`→`UpdateDAG`),
  so between reopts the baseline is frozen at the last reopt interval, not
  the immediately-prior interval. A frozen-vs-fresh diff sees more drift
  than a consecutive-interval diff. Fix A (commit `986a9cfe`) adds
  `dag_tasks_ = dag_tasks_update;` in `OptimizeIncre`
  (`sources/Optimization/OptimizeSP_Incre.cpp` near the `return`) to make
  the diff consecutive. **Plausible, but: this would make the runtime
  changed-task count converge toward the ground-truth 2, not towards 1.**
  If Method B still reports 8 AFTER Fix A, baseline lag is not the whole
  story.

- *Explanation 2 (§11, float-noise through `operator!=`):*
  `FindTaskWithDifferentEt` (`sources/Optimization/OptimizeSP_Incre.cpp:140-155`)
  tests `dag_tasks.tasks[i].execution_time_dist != dag_tasks_updated.tasks[i].execution_time_dist`
  at line 145 — i.e. it calls `FiniteDist::operator!=`, which is
  `!FiniteDist::operator==`. `operator==`
  (`sources/Safety_Performance_Metric/Probability.cpp:355-364`) compares
  each `Value_Proba` sample via `Value_Proba::operator==`
  (`Probability.cpp:12-16`), which uses a RELATIVE tolerance of 1e-1 via
  `approx_equal_double` (`Probability.h:45-49`: `|a-b|/|a| < tol`). A
  1e-13 round-off in a re-estimated `mu` can re-discretize into a sample
  whose tiny `value` makes the relative delta flip past 0.1, so two
  numerically-equal distributions compare unequal. §11 blames this for the
  8-vs-2 gap.

  **But this is in tension with the ground-truth test.** The Method A test
  uses the SAME `operator!=` path and gets 2, not 8. If `operator!=` noise
  were inflating the count, the ground-truth test would also be inflated.
  It is not. So either (a) the noise only appears after
  `UpdateExtDistBasedOnTimeLimit` mutates the distribution at runtime (the
  ground-truth test bypasses that path), or (b) §11's noise theory is wrong
  and the 8 comes from baseline lag (Explanation 1) or something else
  entirely. **This is unresolved.**

**What would settle it.** Re-run the instrumented binary
(`debugMode:1`) on the N=10 `taskset_0` taskset for, say, INCR_P10, capture
the `[INCR-NDIFF-DBG]` (or `[INCR-ET-DBG]`) stderr, and grep the
changed-task counts. If they are 2 → Fix A closed it and §11's noise theory
was a misdiagnosis. If they are still 8 → neither explanation as stated is
correct; the runtime path through `UpdateExtDistBasedOnTimeLimit` is doing
something the raw-YAML diff does not capture, and that path needs
instrumentation, not the diff predicate.

---

## Problem 3 — Fix D (the "swap `!=` for `approx_not_equal`" fix) is inert

**The proposed Fix D (debug log §11).** In `FindTaskWithDifferentEt`
(`sources/Optimization/OptimizeSP_Incre.cpp:145`), swap
`!=` → `approx_not_equal(..., 1e-2)`, plus add a band requiring
`|ΔGetAvgValue| / avg > 1%` before counting a task as changed. Expected ~3×
drop in changed-task count and downstream full-DAG evaluations, zero
correctness risk.

**Verified: the swap is a no-op on the per-sample comparison.**
`FiniteDist::approx_equal` (`sources/Safety_Performance_Metric/Probability.cpp:345-353`)
takes a `tolerance` parameter but **never reads it**:
```cpp
bool FiniteDist::approx_equal(const FiniteDist& other, double tolerance) const {
    if (distribution.size() != other.distribution.size()) return false;
    for (uint i = 0; i < distribution.size(); i++) {
        if (distribution[i] != other.distribution[i]) return false;   // Value_Proba::operator!=, hardcoded 1e-1
    }
    if (near(min_time, other.min_time) && near(max_time, other.max_time))  // near() = abs(a-b) < 1e-6
        return true;
    return false;
}
```
`approx_not_equal(other, tol)` is `!approx_equal(other, tol)` (`Probability.h:149-150`).
So the only behavioral difference between `operator!=` and `approx_not_equal(..., 1e-2)` is
the `min_time`/`max_time` check: `operator==` uses `approx_equal_double(..., 1e-1)`
(relative 10%) for min/max, whereas `approx_equal` uses `near()` (absolute 1e-6).
That is a TIGHTENING of the min/max check, not a loosening, and it does not touch the
per-`Value_Proba` comparison at all. So the swap alone changes almost nothing.

**The only part of Fix D that would do work** is the `|ΔGetAvgValue| / avg > 1%`
band — a NEW predicate not present in the current code. That band would
suppress tasks whose average changed by <1%, regardless of what the
per-sample `operator!=` says.

**Conclusion.** Fix D as literally specified ("swap `!=`→`approx_not_equal`")
is dead. The GetAvgValue band is a real lever, but it is a new check, not a
tolerance tweak, and its payoff depends on Problem 2 being resolved: if the
runtime changed-task count is genuinely 2 (ground truth), there is nothing
to suppress and the band buys nothing; if it is genuinely 8, the band
helps only if the 6 extra tasks are sub-1%-average drift (which §11 asserts
but has not shown with a logged distribution dump). Do not implement Fix D
until Problem 2 is settled with a captured instrumented log.

---

## Problem 4 — `ObtainSP_DAG` call-count ≠ optimizer cost (the 21× misdivision)

**Symptom.** An earlier pass divided per-interval optimizer time by the
per-interval `ObtainSP_DAG` call count and concluded INCRE pays "21× more
per DAG evaluation" than REOPT. This is a misdivision artifact.

**Why it is wrong.** `opt_ms` is the wall time of the WHOLE
`Optimize_w_TL_ScratchOrIncre` call — it includes priority-variation
generation, the diff loop, RTA bookkeeping, and N `ObtainSP_DAG` calls.
Dividing the total by the call count attributes all the non-`ObtainSP_DAG`
overhead to `ObtainSP_DAG` and treats the result as a per-evaluation cost.
REOPT and INCRE have different amounts of non-`ObtainSP_DAG` work per call,
so the quotient is not comparable across arms. The call count itself
(`g_incr_et_debug_sp_dag_calls`, `sources/Safety_Performance_Metric/SP_Metric.cpp:93,99`)
is a valid measure of how many full-DAG evaluations ran; the per-evaluation
cost derived by division is not.

**Resolution.** Use `opt_ms` as the cost and `sp_dag_calls` as the count,
separately. Do not divide. (Recorded as a common-mistake warning in
`runtime_profiling_guide.md` §7.)

---

## Problem 5 — The DAG-chains explanation was wrong

An earlier pass speculated that the simulation-experiment tasksets have DAG
chains/edges that make `ObtainSP_DAG` expensive. **Retracted:** the
tasksets have **zero chains, zero edges, zero predecessors** — every task is
independent. `ObtainSP_DAG` cost on these tasksets is per-task RTA
convolution recomputed across all N tasks, not chain traversal. Do not
revive the chains explanation.

---

## Problem 6 — Cryptic abbreviations obscured the investigation

Throughout the debug log and memory, the quantities are referred to as
`ndiff`, `nvar`, `opt_ms`, `R`, `I`. These are opaque on re-reading.
Descriptive names, used in this file and in `runtime_profiling_guide.md`:

| abbreviation | descriptive name |
|---|---|
| `ndiff` | changed-task count (tasks whose ET distribution differs between two intervals) |
| `nvar` | priority-variation count (priority vectors evaluated per `OptimizeIncre` call) |
| `opt_ms` | per-interval optimizer time (Timer A) |
| `R` / `I` | REOPT path / INCRE path (the two branches of `Optimize_w_TL_ScratchOrIncre`) |

The abbreviations are kept ONLY in code snippets and log-line examples,
where they are the literal field names the user will see in
`[INCR-ET-DBG]` output.

---

## §5 — Outstanding corrections needed in the companion files

These were not made yet (the user redirected to the profiling guide); they
remain pending and should be done before any further Fix C/Fix D work:

1. `agents/debug_runtime0704_incr.md` §10 — record that the instrumented
   changed-task count (≈8) is in tension with the ground-truth test (2) and
   that the discrepancy is UNRESOLVED, not fully explained by either
   baseline lag or float-noise.
2. `agents/debug_runtime0704_incr.md` §11 — correct the Fix D description:
   `FiniteDist::approx_equal` ignores its `tolerance` argument, so the
   `!=`→`approx_not_equal` swap is effectively a no-op on the per-sample
   comparison; only the GetAvgValue band would change behavior.
3. Memory `p25-incr-et-grows-with-period.md` — same two corrections, plus
   drop the unsupported "modal ndiff=1" claim if it appears (the verified
   ground-truth value is 2).
4. Task #11 ("Implement Fix D via TDD") in the session task list is stale:
   Fix D as specified is inert. Re-scope to "capture an instrumented
   changed-task-count log on N=10 taskset_0 INCR_P10 and reconcile 2-vs-8"
   before deciding whether the GetAvgValue band is worth implementing.

---

## Verified facts (for the record)

- Ground-truth changed-task count for
  `tasks10_dur600_interval10_seed1000/taskset_0`, interval_0 → interval_1:
  **2** (task ids 4 and 9, both decreases). Re-run 2026-07-05 via
  `FindTaskWithDifferentEt.N10IntervalYamlGroundTruth`.
- `FindTaskWithDifferentEt` uses `FiniteDist::operator!=` at
  `sources/Optimization/OptimizeSP_Incre.cpp:145`.
- `FiniteDist::operator==` uses `Value_Proba::operator==` (relative
  tolerance 1e-1) per sample, and `approx_equal_double(..., 1e-1)` for
  min_time/max_time. `sources/Safety_Performance_Metric/Probability.cpp:355-364`.
- `FiniteDist::approx_equal` IGNORES its `tolerance` parameter; it uses
  `Value_Proba::operator!=` (hardcoded 1e-1) per sample and `near()`
  (absolute 1e-6) for min_time/max_time.
  `sources/Safety_Performance_Metric/Probability.cpp:345-353`.
- `g_incr_et_debug_sp_dag_calls` is incremented only inside `ObtainSP_DAG`
  at `sources/Safety_Performance_Metric/SP_Metric.cpp:99`.
- Timer A is gated by `debugMode == 1`; Timer B is always on.
- The simulation-experiment tasksets have zero DAG chains/edges/predecessors.
