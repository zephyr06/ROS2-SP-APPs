# P1.15 — Dev Log

## 2026-07-19 — Investigation (Phase 0 complete)

Filed from the user's report: in
`runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/tasks4_dur600_interval10_seed1000`,
`comparison_summary.csv` shows BF (0.6129) losing to INCR_Reopt_5/10/30/60
(~0.622). User manually checked several tasksets, found many had no
optimization records, and suspected an aggregation/reuse bug rather than an
optimizer-correctness bug. Tasked: investigate why.

### Findings

1. **The CSV's "perfect" Reopt_5+ rows are the tell.** Reopt_5/10/30/60
   all report `Mean_Miss_Rate=0.000000, Std=0.000000,
   Important_Miss_Rate=0.000000`. A real scheduler does not produce exactly
   zero miss rate with exactly zero variance across 10 random tasksets —
   this is the signature of empty input lists (`np.mean([])→0.0`).

2. **Per-taskset output map.** Walked all 10 tasksets × 6 arms, checking
   `<ts>/<arm>/<arm>/interval_sp_metrics.txt`. BF and INCR_Reopt_1 have
   output on all 10 tasksets; Reopt_5/10/30/60 have output on only 5
   (missing on tasksets 0, 3, 6, 7, 8). The empty arm dirs contain only the
   dir itself (created by `os.makedirs(sched_dir)` at
   `compare_optimizers.py:510`) — no nested `<arm>/<arm>/` and no output
   files.

3. **The dropped tasksets are exactly where BF beats Reopt_1.** taskset_3
   (BF 0.5433 vs Reopt_1 0.5143), taskset_6 (0.5922 vs 0.5139), taskset_8
   (0.6231 vs 0.5486). BF's mean includes these hard tasksets; Reopt_5+'s
   mean excludes them → BF appears worse. On taskset_0, BF and Reopt_1 are
   byte-identical (60/60 interval SP values match, summary 0.672911,
   4300/39900 misses) → the optimizer is not producing divergent SP for the
   same inputs across arms on the tasksets that ran.

4. **Silent-failure path in the harness.**
   - `compare_optimizers.py:519-527`: arms are submitted via
     `executor.submit(run_single_simulation, ...)` then
     `concurrent.futures.wait(sim_futures)`. `wait` does not raise and
     `.result()` is never called → a `CalledProcessError` from
     `run_single_simulation`'s `subprocess.run(..., check=True)` is
     discarded.
   - `compare_optimizers.py:534-572`: the analyze loop wraps
     `analyze_single_instance` in `try/except Exception` that only `print`s
     and continues.
   - `run_sim_experiments.py:80`: missing `interval_sp_metrics.txt` → empty
     `sp_values_run`, `miss_rate=0` (silent zero, not NaN).
   - `utils.py:206-218`: `np.mean(sp_arr) if len>0 else 0.0` over whatever
     survived → means over unequal, per-scheduler taskset sets.

5. **The binary aborts on the dropped arms.** Reproduced directly on
   taskset_3 with the current release binary (rebuilt 2026-07-19 14:49 from
   HEAD — NOT stale):
   Reopt_5/10/30/60 all exit **134 (SIGABRT, core dumped)**, BF and Reopt_1
   exit 0. The abort fires after interval 0 prints
   `opt_sp_ = 0.514286` (matching the recorded Reopt_1 value) → interval 0
   is fine, the crash is downstream (likely the first Reopt_X>1
   compare-and-keep boundary, or the P1.12 RTA-cache path). No C++
   assert/exception text in stdout/stderr — the abort is text-silent; the
   only signal is the exit code, which the harness ignores.

6. **Git state (corrected).** `git status` shows NO uncommitted changes to
   `sources/` or `tests/` — the optimizer is fully committed at HEAD.
   P1.14's `BFDLSharedBudget` guard = commit `ecf0c597`; P1.12's `:285`
   cache flip + `rta_cache_` integration = the `8e18c39b`/`5a172973`/
   `bfbec7e5` series. (Memory entries `p112-...` and `p114-...` still say
   "working tree, NOT committed" — STALE relative to git; they have since
   been committed.) So the crash is in **committed, current** code. The
   P1.12 cache path + P1.14 guard remain candidate crash sources (recent +
   live in the INCR/Reopt path); BF does not crash (P1.14 touched the BF
   path), consistent with the crash being in the INCR/Reopt path P1.12
   touched. Must gdb-classify before assuming pre-existing.

### Conclusion

User's hypothesis confirmed: not an optimizer-correctness bug in the SP
sense. The optimizer *does* crash on some (taskset, arm) pairs, but the
harness swallows the crash and the aggregator then compares schedulers over
different taskset sets — manufacturing BF's apparent loss. Two independent
fixes needed (harness layer A, optimizer layer B); see `goal.md` Decision
needed and `tasks.md` Phases 1–3.

No code changed. Awaiting user direction on fix scope/ordering before
Phase 1.

## 2026-07-19 (later) — Phase 0 deepened: exact crash stack + throw site

Independently re-verified the load-bearing Phase 0 claims and captured the
gdb stack, pinning layer B precisely.

### Re-verification (all PASS)

- **Git state.** `git status -- sources/ tests/` is clean (only an
  untracked `tests/debug_analysis/__pycache__/`). P1.14 `BFDLSharedBudget`
  = `ecf0c597` (in history), P1.12 cache series = `8e18c39b`→
  `5a172973`→`bfbec7e5` (all in history). The optimizer IS fully committed
  at HEAD; memory entries `p112-...`/`p114-...` saying "working tree, NOT
  committed" are STALE. Binary mtime 2026-07-19 14:49:38, HEAD commit
  2026-07-19 14:48:30 → binary is FRESH from HEAD.
- **BF vs Reopt_1 byte-identical on taskset_0.** `diff -q` on
  `interval_sp_metrics.txt` = identical; both `sp_metrics_summary.txt` =
  0.672911. Confirms the optimizer does not produce divergent SP across
  arms on the tasksets that ran.
- **Per-taskset output map.** Re-walked 10×6: BF + Reopt_1 = all 10;
  Reopt_5/10/30/60 = only 5 (missing on 0,3,6,7,8). Matches the table in
  `goal.md`.
- **SIGABRT reproduced.** `taskset_3 INCR_Reopt_5` direct run → exit 134
  ("timeout: the monitored command dumped core"), right after interval 0
  prints `opt_sp_ = 0.514286` + `TraverseTimeLimitOptions`. Interval 0
  completes; the crash is at the first Reopt_X>1 boundary.

### The crash stack (gdb -batch, taskset_3 INCR_Reopt_5)

```
Program received signal SIGABRT, Aborted.
__pthread_kill_implementation ... pthread_kill.c:44
__GI_raise / __GI_abort
__cxxabativ1::__terminate -> std::terminate -> __cxa_throw   ← UNCAUGHT throw
#8  SP_OPT_PA::RTACache::ComputeTaskSetDifference(...) [clone .cold]   ← THROW SITE
#9  SP_OPT_PA::RTACache::ClassifyReusePerTask(...)
#10 SP_OPT_PA::RTACache::Evaluate(...)
#11 OptimizePA_Incre_with_TimeLimits::EvaluateTimeLimitConfig_SubIncremental   ← OptimizeSP_TL_Incre.cpp:279
#12 OptimizePA_Incre_with_TimeLimits::OptimizeSingleTaskTimeLimit_Impl
#13 OptimizePA_Incre_with_TimeLimits::PerformSerializedTaskQueueOptimization   ← :439 arms rta_cache_active_=true
#14 OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL
#15 OptimizePA_Incre_with_TimeLimits::Optimize_w_TL_ScratchOrIncre            ← :616 INCR BFDLSharedBudget scope
#16 FixedTaskPrioritySchedulingOrchestrator::DeterminePrioritiesAndBudgets
#17 ...::SimulateInterval -> ...::RunSimulation -> main
```

The `__cxa_throw` at #7 is `std::runtime_error` (typeinfo confirmed by gdb).
So the abort is NOT a segfault — it is an **uncaught C++ exception**, the
`std::runtime_error` thrown by design at `RTA_Cache.cpp:357-364`.

### The throw site (RTA_Cache.cpp:350-366)

`ComputeTaskSetDifference` delegates to `TryComputeSingleChange`; when that
returns `false` (candidate differs from champion by >1 task, violating the
P1.10 single-change invariant) it `throw std::runtime_error("...differs
from champion by more than one task — violates the P1.10 single-change
invariant...")`. The throw propagates uncaught through
`ClassifyReusePerTask → Evaluate → EvaluateTimeLimitConfig_SubIncremental
→ ... → main` → `terminate` → `abort` (exit 134). Nothing in the INCR path
catches it (the `try/except` that swallows it is in the Python harness, one
layer up — and only because the binary is dead by then).

### Why this is the P1.12 cache path (not P1.14, not pre-P1.12)

- The throw is in `RTACache`, which is P1.12's class (`843f9603`+). BF does
  not enter `OptimizeIncre`/the cache, so BF never throws — consistent with
  the 10×6 map.
- `Evaluate` is reached only when `rta_cache_active_` is true
  (`OptimizeSP_TL_Incre.cpp:439` arms it inside
  `PerformSerializedTaskQueueOptimization`; `:830` disarms in
  `ResetIncumbentBaseline`). The cache is OFF in the reopt path by design
  (`OptimizeSP_TL_Incre.h:291-299` comment: reopt can commit a >1 change
  via memoryless `OptimizeFromScratch`).
- So the crash is a **single-change-invariant violation *inside* the
  serialized walk** — the candidate triple
  `(dag_tasks_cur, challenger.opt_pa_, time_limits)` scored at
  `OptimizeSP_TL_Incre.cpp:279` differs from the adopted champion by >1
  task. The design comment at `:252-262` asserts this can only be Type-L
  (|diff|==1, one TL moved) or Type-E (|diff|==0); the throw says neither
  holds.

### `TryComputeSingleChange` returns false in exactly 4 ways (RTA_Cache.cpp:252-339)

1. `:270` — `FindTaskWithDifferentEt(champ_baked, cand_baked)` size > 1
   (more than one task's baked ET differs).
2. `:285` — `AnalyzePrioritySwitch(...).status == NotSingle` (the per-core
   priority-order diff is not a single relocation).
3. `:304` — ET diff + priority move on DIFFERENT cores (2 changes).
4. `:329-331` — after removing the ET-diff task, the rest of the per-core
   order still differs (a SECOND task also moved).

Why this fires only at Reopt_X>1 and only on tasksets {0,3,6,7,8} (not
Reopt_1, not the other 5) is the open Phase 2 question — it requires
tracing which champion `CommitIncumbent→AdoptChampion` stored vs which
candidate `EvaluateTimeLimitConfig_SubIncremental:279` scores at the
reopt boundary, on a failing fixture. Reopt_1 reopts every interval (cache
state realigns each interval); Reopt_X>1 carries the champion across
multiple env moves, and `ResetIncumbentBaseline:829` resets the cache at
each interval — so the question is whether the *first* serialized eval
after a reset, or a later eval within the walk, drifts to >1.

### Updated fix guidance

Layer B is now precisely scoped: the throw at `RTA_Cache.cpp:357-364` is
the kill site. Two viable fix shapes (decide in Phase 2 B2/B3):
  (i) **Make the throw impossible** — prove and enforce that the candidate
  scored at `:279` always satisfies |diff|<=1 (fix the invariant
  violation at its source; requires finding which of the 4 false-returns
  fires and why the champion drifted).
  (ii) **Make `Evaluate` degrade gracefully on >1** instead of throwing —
  fall back to a full RTA recompute (NoReuse for every task) when
  `!TryComputeSingleChange`, mirroring the `!HasChampion()` path. This is
  the safer, behavior-preserving option: it keeps the cache a pure
  optimization (a cache miss = recompute, never a crash), matching how
  `IsSingleTaskChange` is documented as the "ASK first" guard. The throw
  is the wrong default for a cache.
Recommendation: **(ii) for the immediate unblock** (one-line-grade change,
no SP regression — a >1 diff just means a cache miss), then **(i)** as a
follow-up if the invariant violation reveals a real optimizer-logic bug.
Either way, Phase 1 (harness) must land too so a future crash is loud, not
silent.

## 2026-07-19 (later still) — Phase 1 start: harness loud-failure fix

User direction: **(A) harness first**, then (B) crash investigation. Two
explicit requirements for the harness:
1. **Stop the experiment when one optimizer crashes** — do not continue
   the remaining arms/tasksets as if nothing happened.
2. **Show the crash loudly** — never treat a crashed arm's missing output
   as a silent `0.000000` row; capture the C++ stdout/stderr so the throw
   text survives for Phase 2; print a loud report; write `crash_report.txt`
   + the per-taskset ✅/❌ map; exit non-zero.

Decisions locked (see `goal.md` "Decisions"): aggregate =
per-taskset-mean-then-average AND fail-loudly on unequal N; `--resume`
re-attempts partial/empty dirs (file must exist *and* be non-empty).

### Re-verified git state before coding

`git status --short -- sources/ tests/` = clean (only untracked
`tests/debug_analysis/__pycache__/`). HEAD = `bfbec7e5` (branch
`clean_simulation`). Confirmed committed sites:
- P1.14 `BFDLSharedBudget`: BF install `OptimizeSP_TL_BF.cpp:89`, INCR
  mirror `OptimizeSP_TL_Incre.cpp:653`, `UpdateSP` poll
  `OptimizeSP_Incre.cpp:71`.
- P1.12 RTA-cache: `rta_cache_` member `OptimizeSP_TL_Incre.h:290`, read-side
  `OptimizeSP_TL_Incre.cpp:279`/`:287`, re-arm `:439`, `CommitIncumbent`
  AdoptChampion `:782-785`, reset `:829-830`.

So the "working tree, NOT committed / STAGED" framing in the
`p112-`/`p114-` memory entries + the P1.14 finished-task files was STALE —
corrected those records (memory + `P1_14_*/goal.md`+`tasks.md`) to reflect
committed state, and marked P1.14 Phase 3 (A/B re-run) BLOCKED on P1.15.

### Harness code mapped (where the silent-failure lives)

- `compare_optimizers.py:506-527` — `ProcessPoolExecutor` +
  `concurrent.futures.wait(sim_futures)` with NO `.result()` →
  `CalledProcessError` from the worker discarded.
- `compare_optimizers.py:534-572` — analyze loop wraps
  `analyze_single_instance` in `try/except Exception` that only `print`s.
- `run_sim_experiments.py:53-55` — `subprocess.run(..., stdout=DEVNULL,
  stderr=DEVNULL)` unless `verbose>=2` → C++ throw text lost on crash.
- `run_sim_experiments.py:80` — missing `interval_sp_metrics.txt` → empty
  `sp_values_run`, `miss_rate=0` (silent zero).
- `utils.py:206-218` — `np.mean(sp_arr) if len>0 else 0.0` over whatever
  survived → means over unequal per-scheduler taskset sets.
- `compare_optimizers.py:574` — `results_by_taskset` already collected
  (per-taskset dict) → thread into `write_summary_and_plots` for
  per-taskset-mean-then-average.
- Existing tests to keep green: `tests/python/test_compare_optimizers.py`,
  `test_run_sim_experiments.py`, `test_aggregate.py` (all `unittest`,
  temp-dir + mock style).

TDD-first per `goal.md` gate: A0 RED tests, then A1–A6 GREEN.

---

## 2026-07-19 (session 2) — Phase 1 DONE + requirement simplified

### Requirement simplified (user, verbatim)
> "as long as there is one crash, then no results are useful anymore, no need
> to report any numbers, fail it loudly. For one task number such as N=4/6/8,
> if any method has a crash at any task set's any interval, then results are
> not useful anymore. fail loudly, we'll restart all exp after fixing it."

This OVERRIDES the earlier per-taskset-mean-then-average / NaN-row / unequal-N
aggregation design (old A4). Rationale: if a partial aggregate is never
written, its internal averaging semantics cannot manufacture a false verdict.
Simpler and stronger.

### Phase 1 (layer A) — DONE, all GREEN
Discovered at session start that Phase 1 was already ~90% implemented in the
working tree (compare_optimizers.py, run_sim_experiments.py, utils.py, +
test_compare_optimizers_crash.py). Session 2 work = validate + close the gap
the simplified requirement exposed.

**Gap found + fixed:** the crash branch in `compare_optimizers.py main()`
(formerly `:815-846`) still called `write_summary_and_plots` and wrote a
PARTIAL `comparison_summary.csv` (crashed arm = NaN row, OK arms = real
numbers). That is "reporting numbers" — exactly what the user does not want.
**Fix:** on any crash, write ONLY `crash_report.txt` + `taskset_arm_status.csv`
+ exit non-zero. No `comparison_summary.csv`, no plots. Removed the now-dead
`_append_crash_report_note` helper.

**Test updated:** `test_no_silent_zero_row_for_crashed_arm` (asserted the
partial summary exists with non-zero rows) → renamed
`test_no_summary_written_on_crash`: now asserts `comparison_summary.csv` +
`comparison_plots.png` are ABSENT on crash, diagnostics ARE present, exit
non-zero.

### Validation
- `test_compare_optimizers_crash.py`: 9/9 GREEN.
- Full `tests/python/` suite: 300/300 GREEN (no regression).
- E2E (stub crashing binary, via the test's `_run_main_with_crashing_arm`
  factory which drives real `main()`): stop-on-crash fires, loud `[CRASH]`
  stderr, `crash_report.txt` with C++ throw-text tail, `taskset_arm_status.csv`
  (BF=OK, INCR_Reopt_5=CRASHED), NO `comparison_summary.csv`, NO plots,
  exit 1.

### Phase 2 (layer B) — DONE by P1.16 (NOT by this task)
The C++ `SIGABRT` (exit 134) that *caused* the silent-failure bug is fixed by
P1.16 (`agents/active_tasks/P1_16_rta_cache_desync_fix/`):
`UpdateRecords` returns `bool`; `EvaluateTimeLimitConfig_SubIncremental` backs
up `rta_cache_` at entry and reverts on rejected candidate. Reproduced
`taskset_3 INCR_Reopt_5/10/30/60` → all exit 0 (was 134). So both layers are
resolved; Phase 3 (re-run the P25 A/B on fixed binary + fixed harness) can
proceed.

### Files touched this session (Python only — no C++)
- `simulation_experiments/compare_optimizers.py` — crash branch no longer
  writes partial summary; removed dead `_append_crash_report_note`.
- `tests/python/test_compare_optimizers_crash.py` —
  `test_no_silent_zero_row_for_crashed_arm` →
  `test_no_summary_written_on_crash`.

### Next (Phase 3)
Re-run the P25 A/B (`p25_period_ab_config`) on the fixed binary + fixed
harness; verify the correctness gate (every scheduler same N, all ✅ in the
status map, no crashes); record the true BF-vs-INCR_Reopt_X verdict.
