# Runtime Profiling Guide: Per-Interval Scheduler ET and Total Simulation Time

**Purpose.** Let you measure, yourself, two distinct quantities that this
codebase reports and that are easy to conflate:

1. **Per-interval optimizer time** — wall time of *just* the priority/time-limit
   optimizer call, one per interval. Already instrumented; gated by `debugMode`.
2. **Per-activation scheduler execution time** — wall time of the *entire*
   `RunSimulation()` process divided by the interval count. This is the number
   that appears in A/B tables as "ms/act" (~90 ms for N=6 INCR arms).

These are **not** the same quantity. The first wraps one function call inside
the per-interval loop; the second wraps the whole process (load tasksets,
simulate every interval, export results, process startup) and then divides.
Confusing them produces the "incre is 14 ms but the table says 90 ms"
contradiction. This guide keeps them separate.

All file:line references below were verified against the working tree at
HEAD `de4e9636` on 2026-07-05.

---

## 0. The two timers, side by side

| | Timer A: per-interval optimizer time | Timer B: whole-process scheduler time |
|---|---|---|
| What it measures | Only `incr_optimizer_.Optimize_w_TL_ScratchOrIncre(...)` | The whole `RunSimulation()` call: load + all intervals + export |
| Where it's timed | `SimulationOrchestrator.cpp:302-308` | `tests/RunOrchestrator.cpp:83` (start) → `:124` (end) |
| Where it's written | stderr as `[INCR-ET-DBG] ... opt_ms=...` | `scheduler_execution_time.txt` (one float, seconds) |
| Granularity | One line per interval | One number for the whole run |
| Gating | `GlobalVariables::debugMode == 1` | Always on |
| The number you saw | 14.5 ms (incremental) / 110 ms (reoptimization) | ~90 ms/act (≈ 2.7 s whole run / 30 intervals) |

**To get per-interval *total* simulation time** (not just the optimizer
part), you must add a small timer around `SimulateInterval(...)` in
`RunSimulation()` — see step 3 below. The codebase does not currently
emit that; only the optimizer sub-portion is timed per interval.

---

## 1. Profiling code pointers (read these first)

### Timer A — per-interval optimizer instrumentation (already in code)

**File:** `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`
**Function:** `FixedTaskPrioritySchedulingOrchestrator::DeterminePrioritiesAndBudgets`
**Lines:** 298-316

```cpp
int dbg_cnt_before = incr_optimizer_.reoptimization_interval_count_;
int dbg_period = GlobalVariables::ReoptimizationPeriod;
bool dbg_is_reopt = (dbg_cnt_before % dbg_period == 0);
int dbg_sp_calls_before = g_incr_et_debug_sp_dag_calls;
auto dbg_t0 = std::chrono::high_resolution_clock::now();
incr_optimizer_.Optimize_w_TL_ScratchOrIncre(
    dag_tasks,
    GlobalVariables::Layer_Node_During_Incremental_Optimization);
auto dbg_t1 = std::chrono::high_resolution_clock::now();
double dbg_ms = std::chrono::duration<double, std::milli>(
                    dbg_t1 - dbg_t0).count();
if (GlobalVariables::debugMode == 1) {
    std::cerr << "[INCR-ET-DBG] interval=" << dbg_cnt_before
              << " mode=" << scheduler_mode_
              << " path=" << (dbg_is_reopt ? "REOPT" : "INCRE")
              << " sp_dag_calls="
              << (g_incr_et_debug_sp_dag_calls - dbg_sp_calls_before)
              << " opt_ms=" << dbg_ms << "\n";
}
```

What each field means:
- `interval=` — the interval index (0-based) **before** the counter advances.
- `mode=` — the scheduler mode string (`INCR_P1`, `INCR_P10`, …).
- `path=` — `REOPT` if this interval took the wide-radius reoptimization
  branch, `INCRE` if it took the narrow-radius incremental branch. Decided
  by `interval_count % ReoptimizationPeriod == 0`.
- `sp_dag_calls=` — how many times `ObtainSP_DAG` (the full-DAG
  safety/performance evaluation) was called during *this* interval's
  optimizer call. Counted by the global `g_incr_et_debug_sp_dag_calls`.
- `opt_ms=` — wall time of the optimizer call in milliseconds. **This is
  Timer A.** It does NOT include taskset loading, the run-queue simulation
  loop, SP-metric export, or process startup.

### The `ObtainSP_DAG` call counter

**File:** `sources/Safety_Performance_Metric/SP_Metric.cpp`
**Lines:** 93 (definition), 99 (increment)
**Header:** `sources/Safety_Performance_Metric/SP_Metric.h:68` (`extern` decl)

```cpp
int g_incr_et_debug_sp_dag_calls = 0;            // line 93

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters) {
    if (GlobalVariables::debugMode == 1) {
        BeginTimer("ObtainSP_DAG");
        g_incr_et_debug_sp_dag_calls++;           // line 99
    }
    ...
}
```

`g_incr_et_debug_sp_dag_calls` is a process-global. The per-interval
instrumentation snapshots it before and after the optimizer call and prints
the delta, so each `[INCR-ET-DBG]` line shows exactly how many full-DAG
evaluations that one interval cost.

### Per-`OptimizeIncre` diff/variation breakdown (already in code)

**File:** `sources/Optimization/OptimizeSP_Incre.cpp`
**Lines:** 247-273

Emits, when `debugMode==1`, a line per `OptimizeIncre` call:
`[INCR-ET-DBG] OptimizeIncre: ndiff=<N> nvar=<V> sp_dag_calls=<V>`, where
`ndiff` = number of tasks whose execution-time distribution changed since
the previous interval (the diff that triggers incremental work), and
`nvar` = number of priority variations evaluated. This is the line that
exposed the old "ndiff=8" claim was a measurement artifact; the real
modal value is 1.

### Timer B — whole-process scheduler time

**File:** `tests/RunOrchestrator.cpp`
**Lines:** 83 (start), 124-131 (end + write)

```cpp
auto start_time = std::chrono::high_resolution_clock::now();   // line 83
...
FixedTaskPrioritySchedulingOrchestrator orchestrator(input_folder, output_folder, mode, duration);
orchestrator.RunSimulation();                                  // line 122
auto end_time = std::chrono::high_resolution_clock::now();     // line 124
double exec_seconds = std::chrono::duration<double>(end_time - start_time).count();
...
std::string exec_time_path = output_folder + "/" + mode + "/scheduler_execution_time.txt";
std::ofstream exec_time_file(exec_time_path);
if (exec_time_file.is_open()) { exec_time_file << exec_seconds << "\n"; ... }
```

`start_time` is captured **before** the orchestrator is constructed, so
Timer B includes orchestrator construction (which loads all interval
tasksets via `LoadIntervalConfigs`), the full interval loop, and
`ExportResults`. The per-activation number in A/B tables is
`exec_seconds / num_intervals`.

### What runs inside one interval (so you know what Timer A excludes)

**File:** `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`
**Function:** `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval`
**Lines:** 508-544

Per interval, in order:
1. `DeterminePrioritiesAndBudgets(dag_tasks, sp_parameters)` — **only this
   contains the optimizer call that Timer A measures.** Everything below is
   Timer-A-excluded work that nonetheless lands in Timer B.
2. `ApplyTaskConfigurations(dag_tasks, res)` — write priorities/time-limits
   back onto the tasks.
3. The run-queue simulation loop (lines 527-533) — iterates `time_now` from
   `start_time` to `end_time` step by step, releasing jobs, running the
   highest-priority job, recording finishes. **This is O(interval_duration)
   per task set**, i.e. it scales with the simulated horizon, not with N or
   the optimizer. For a 10 s interval this loop runs ~10 000 ticks.
4. `ObtainSP_TaskSet_And_TimeLimits(...)` (line 542) — computes the SP
   metric for the interval's results and pushes it onto
   `interval_sp_metrics_`. This is a *separate* SP evaluation from the
   optimizer's `ObtainSP_DAG` calls; it is NOT counted by
   `g_incr_et_debug_sp_dag_calls` (that counter only fires inside
   `ObtainSP_DAG`, not `ObtainSP_TaskSet_And_TimeLimits`).

So per interval, Timer B includes: the optimizer (Timer A) + config apply +
the 10 000-tick run-queue loop + one `ObtainSP_TaskSet_And_TimeLimits` +
trace loading. Across the whole run, Timer B also adds `LoadIntervalConfigs`
(once) and `ExportResults` (once).

### The `debugMode` switch

**File:** `sources/Utils/Parameters.cpp:8-11`

```cpp
YAML::Node loaded_doc = YAML::LoadFile(PROJECT_PATH + "sources/parameters.yaml");
int debugMode = loaded_doc["debugMode"].as<int>();
```

`debugMode` is read **once at process start** from `sources/parameters.yaml`
(field `debugMode`, currently `0`). It is a `GlobalVariables` member
declared in `sources/Utils/Parameters.h:13`. Flipping it to `1` turns on
every `[INCR-ET-DBG]` line and the `g_incr_et_debug_sp_dag_calls` counter.
It is inert in production runs (no perf cost when 0, because every
instrumented site gates on `if (GlobalVariables::debugMode == 1)`).

### The dispatcher that picks REOPT vs INCRE

**File:** `sources/Optimization/OptimizeSP_TL_Incre.cpp` —
`OptimizePA_Incre_with_TimeLimits::Optimize_w_TL_ScratchOrIncre`. Every
`ReoptimizationPeriod`-th call (counter `reoptimization_interval_count_`)
takes the wide-radius REOPT path; otherwise the narrow-radius INCRE path.
This is what makes `path=REOPT` vs `path=INCRE` appear in the debug lines,
and is why the period knob (`INCR_P1`/`P10`/`P30`/`P60`) shifts the
REOPT/INCRE mix.

---

## 2. Measure Timer A (per-interval optimizer time) — the existing path

These steps give you the `[INCR-ET-DBG]` per-interval breakdown for one arm
on one taskset, serially (no contention).

### Step 2.1 — Flip `debugMode` to 1

Edit `sources/parameters.yaml`:

```yaml
debugMode: 1
```

(The other fields — `TIME_LIMIT`, `Granularity`, `ReoptimizationPeriod`,
`ReoptimizationTimeLimitSearchRadius`, `IncrementalTimeLimitSearchRadius`,
`EXPORT_DETAIL_LEVEL`, `METRIC_SAMPLE_INTERVAL_SECONDS`, `printRTA` —
leave as-is. `ReoptimizationPeriod` here is the *default*; the `INCR_P<n>`
mode string overrides it at runtime via `MaybeOverrideReoptPeriod` in
`tests/RunOrchestrator.cpp:20-39`, so you do not need to edit it to sweep
periods.)

### Step 2.2 — Rebuild the release binary

The existing `release/` build is `CMAKE_BUILD_TYPE=Release` (`-O3 -DNDEBUG`);
`build/` is `DEBUG` (`-g`). Use **release** for timing — debug builds skew
the optimizer/simulation ratio because the run-queue loop is O(ticks) and
debug `-g` with no optimization slows it disproportionately.

```bash
cd /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt
cmake -B release -DCMAKE_BUILD_TYPE=Release
cmake --build release --target RunOrchestrator -j
```

The binary lands at `release/tests/RunOrchestrator`.

(If you also need `ctest` green, build the `testOptimizeIncrePA` and
`testIncreOpt_w_TL` targets the same way: `cmake --build release --target testOptimizeIncrePA testIncreOpt_w_TL -j`.)

### Step 2.3 — Pick a taskset

Reuse an existing prod-run taskset so results are comparable to the A/B
table (no regeneration). For N=6:

```
simulation_experiments/optimizer_comparison/runs/
  p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8/sim/
  tasks6_dur300_interval10_seed1000/taskset_0
```

For N=8 swap `tasks6` → `tasks8`. For N=10 use a generated taskset under
`simulation_experiments/optimizer_comparison/tasks10_dur600_interval10_seed1000/taskset_<i>`
(reuse the same one the A/B used; check `et_repro/` JSONs for which).

Confirm the interval count (the per-activation divisor) matches
expectations — 30 for the 300 s / 10 s prod tasksets, 60 for the 600 s / 10 s
N=10 tasksets:

```bash
ls simulation_experiments/optimizer_comparison/runs/p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8/sim/tasks6_dur300_interval10_seed1000/taskset_0/taskset_characteristics_interval_*.yaml | wc -l
```

### Step 2.4 — Run one arm, serially, capturing stderr

The RunOrchestrator CLI is:

```
release/tests/RunOrchestrator <input_folder> <output_folder> <mode> <duration_ms> [export_level] [sample_interval_sec]
```

`duration_ms` is the **per-interval horizon** in ms (see the memory
[[runorchestrator-duration-arg-semantics]]). For a 10 s interval pass
`10000`. **Do not** pass the whole-simulation duration — that mistake
produced the bogus 14.85 s ET in the earlier investigation.

```bash
TASKSET=simulation_experiments/optimizer_comparison/runs/p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8/sim/tasks6_dur300_interval10_seed1000/taskset_0
OUT=/tmp/ndiff_probe_incr_p10

mkdir -p "$OUT"
release/tests/RunOrchestrator "$TASKSET" "$OUT" INCR_P10 10000 1 \
  > "$OUT/stdout.log" 2> "$OUT/stderr.log"
```

- `INCR_P10` sets `ReoptimizationPeriod=10` (reoptimization every 10th
  interval; incremental in between).
- The 5th arg `1` keeps `EXPORT_DETAIL_LEVEL=1` (small disk usage; the
  optimizer timing is independent of export level).
- `2>` captures the `[INCR-ET-DBG]` lines. `stdout` gets the
  `ExecutionTime_s:` line and the average-SP line.

### Step 2.5 — Read the per-interval optimizer breakdown

```bash
grep '\[INCR-ET-DBG\] interval=' "$OUT/stderr.log" | head
```

Each line is one interval's Timer A. To split by path and average:

```bash
echo "=== REOPT intervals ==="
grep '\[INCR-ET-DBG\] interval=' "$OUT/stderr.log" | grep 'path=REOPT' \
  | grep -oP 'opt_ms=\K[0-9.]+' \
  | awk '{s+=$1; n++} END {printf "count=%d  mean_opt_ms=%.2f\n", n, s/n}'

echo "=== INCRE intervals ==="
grep '\[INCR-ET-DBG\] interval=' "$OUT/stderr.log" | grep 'path=INCRE' \
  | grep -oP 'opt_ms=\K[0-9.]+' \
  | awk '{s+=$1; n++} END {printf "count=%d  mean_opt_ms=%.2f\n", n, s/n}'
```

To see whether incremental-interval cost grows over the run (the original
"grows with period" symptom), print per-interval `opt_ms` for the INCRE
rows in interval order:

```bash
grep '\[INCR-ET-DBG\] interval=' "$OUT/stderr.log" \
  | grep 'path=INCRE' \
  | grep -oP 'interval=\K[0-9]+|opt_ms=\K[0-9.]+' \
  | paste - - \
  | awk '{printf "interval=%s  opt_ms=%s\n", $1, $2}'
```

If `opt_ms` climbs from interval 1 to interval 59 within a single INCR_P60
run, the growth is **inside the optimizer** (Timer A). If it's flat, the
growth is in Timer B's non-optimizer work — and you need step 3.

### Step 2.6 — Read the per-`OptimizeIncre` diff/variation breakdown

```bash
grep '\[INCR-ET-DBG\] OptimizeIncre:' "$OUT/stderr.log" | head
```

`ndiff` = changed-task count for that incremental call; `nvar` = priority
variations tried; `sp_dag_calls` = full-DAG evaluations for that call. The
modal `ndiff` should be 1 (never 8 — that old claim was an artifact).

---

## 3. Measure per-interval *total* simulation time (Timer B is whole-process only)

Timer B (`scheduler_execution_time.txt`) gives one number for the whole
run. To get the **per-interval** total — i.e. the full
`SimulateInterval` wall time, including the optimizer AND the run-queue
loop AND the SP-metric compute — add a small timer around the
`SimulateInterval` call in `RunSimulation()`.

### Step 3.1 — Add a per-interval total timer

**File:** `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`
**Function:** `FixedTaskPrioritySchedulingOrchestrator::RunSimulation`
**Lines:** 277-281 (the interval loop)

Current code:

```cpp
for (size_t i = 0; i < dag_tasks_vecs_.size(); i++) {
    LLint start_time = i * interval_duration_ms_;
    LLint end_time = (i + 1) * interval_duration_ms_;
    SimulateInterval(i, start_time, end_time);
}
```

Change to:

```cpp
for (size_t i = 0; i < dag_tasks_vecs_.size(); i++) {
    LLint start_time = i * interval_duration_ms_;
    LLint end_time = (i + 1) * interval_duration_ms_;
    auto sim_t0 = std::chrono::high_resolution_clock::now();
    SimulateInterval(i, start_time, end_time);
    auto sim_t1 = std::chrono::high_resolution_clock::now();
    double sim_ms = std::chrono::duration<double, std::milli>(
                        sim_t1 - sim_t0).count();
    if (GlobalVariables::debugMode == 1) {
        std::cerr << "[INCR-ET-DBG] interval=" << i
                  << " mode=" << scheduler_mode_
                  << " sim_ms=" << sim_ms << "\n";
    }
}
```

This `sim_ms` is the **per-interval total** — what Timer B would report if
it were per-interval instead of whole-process. The difference
`sim_ms − opt_ms` per interval is the non-optimizer work (run-queue loop +
`ObtainSP_TaskSet_And_TimeLimits` + trace loading + config apply). That
delta is the missing piece in the "incre is 14 ms but the table says 90 ms"
reconciliation.

Notes:
- `<chrono>` is already included at the top of the file (line 2).
- `GlobalVariables::debugMode` is visible here (it's used at line 309 in
  the same file).
- This is debug-gated, so it has zero cost in production. Leave it in.

### Step 3.2 — Rebuild and rerun

```bash
cmake --build release --target RunOrchestrator -j
release/tests/RunOrchestrator "$TASKSET" /tmp/ndiff_probe_incr_p10_v2 INCR_P10 10000 1 \
  > /tmp/ndiff_probe_incr_p10_v2/stdout.log 2> /tmp/ndiff_probe_incr_p10_v2/stderr.log
```

(Adjust `INCR_P10` to `INCR_P1` / `INCR_P30` / `INCR_P60` to sweep the
period. Use a fresh output dir per arm so the logs don't overwrite.)

### Step 3.3 — Compare per-interval total vs per-interval optimizer

For each interval you now have two lines: one with `opt_ms=` (Timer A, from
`DeterminePrioritiesAndBudgets`) and one with `sim_ms=` (per-interval
total, from `RunSimulation`). Join them by interval index:

```bash
awk '
/\[INCR-ET-DBG\] interval=.*opt_ms=/ {
    match($0, /interval=([0-9]+)/, a);
    match($0, /opt_ms=([0-9.]+)/, b);
    opt[a[1]] = b[1];
}
/\[INCR-ET-DBG\] interval=.*sim_ms=/ {
    match($0, /interval=([0-9]+)/, a);
    match($0, /sim_ms=([0-9.]+)/, b);
    sim[a[1]] = b[1];
}
END {
    for (i=0; i in sim || i in opt; i++) {
        if (i in sim) {
            o = (i in opt) ? opt[i] : "NA";
            nonopt = (i in opt) ? (sim[i] - opt[i]) : "NA";
            printf "interval=%-3s opt_ms=%-10s sim_ms=%-10s non_opt_ms=%s\n", i, o, sim[i], nonopt;
        }
    }
}' /tmp/ndiff_probe_incr_p10_v2/stderr.log | head -40
```

(The `non_opt_ms` column is the run-queue + SP-metric + trace-loading work.
It should be roughly constant per interval because the run-queue loop is
O(interval_duration) and independent of the period knob. If it is NOT
constant — e.g. it grows with interval index — that's a real signal worth
chasing, and it lives in `SimulateInterval` lines 527-543, not in the
optimizer.)

---

## 4. Measure Timer B (whole-process per-activation ET) — the A/B table number

This is what the existing repro harness already does. Use it rather than
re-implementing.

### Step 4.1 — Reset `debugMode` to 0 for clean timing

```yaml
# sources/parameters.yaml
debugMode: 0
```

The `[INCR-ET-DBG]` prints and the `g_incr_et_debug_sp_dag_calls` increments
add a little overhead (mostly the `BeginTimer` and stderr writes). For
Timer B numbers comparable to the A/B table, run with `debugMode=0`. Keep
`debugMode=1` only when you want the Timer A / `sim_ms` breakdown.

Rebuild after flipping:

```bash
cmake --build release --target RunOrchestrator -j
```

### Step 4.2 — Run the serial repro harness

**File:** `simulation_experiments/repro_et_grows_with_period.py`

This script runs each arm's `RunOrchestrator` **one at a time** (no
contention), reads back `scheduler_execution_time.txt`, divides by the
interval count, and prints an arm-vs-ET table. It is the source of the
A/B numbers in `agents/debug_runtime0704_incr.md`.

```bash
cd /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt

# N=6, taskset 0, all arms, 2 reps (min taken to filter OS-scheduling noise)
python3 -m simulation_experiments.repro_et_grows_with_period \
  --num_tasks 6 --taskset 0 --reps 2 \
  --arms INCR_P1 INCR_P10 INCR_P30 INCR_P60 INCR_SCRATCH \
  --bin_dir release --tag tasks6_ts0_reps2_dbgoff
```

Output lands in
`simulation_experiments/optimizer_comparison/et_repro/<tag>/et_repro_result.json`
and includes per-arm `wall_s` (whole-process seconds, = Timer B) and
`per_act_ms` (= `wall_s * 1000 / num_intervals`).

For N=10 on a freshly generated taskset (the prod run only has N=4/6/8):

```bash
# First generate or locate an N=10 taskset, then point --taskset_dir at it.
python3 -m simulation_experiments.repro_et_grows_with_period \
  --taskset_dir <abs path to tasks10 .../taskset_0> \
  --reps 2 --arms INCR_P1 INCR_P10 INCR_P30 INCR_P60 INCR_SCRATCH \
  --bin_dir release --tag tasks10_ts0_reps2_dbgoff
```

### Step 4.3 — Read the table

```bash
python3 -c "
import json
r = json.load(open('simulation_experiments/optimizer_comparison/et_repro/tasks6_ts0_reps2_dbgoff/et_repro_result.json'))
print(f'{\"arm\":<13} {\"wall_s\":>9} {\"per_act_ms\":>12}')
for arm, d in r['arms'].items():
    print(f'{arm:<13} {d[\"wall_s\"]:>9.4f} {d[\"per_act_ms\"]:>12.3f}')
"
```

`per_act_ms` is the number that should be compared across periods. The
symptom under investigation is `per_act_ms` growing with the period
(`INCR_P1` < `INCR_P10` < `INCR_P30` < `INCR_P60`).

---

## 5. Putting it together: the reconciliation recipe

To finally answer "does the growth live in the optimizer or in the
non-optimizer per-interval work?", run **both** timers on the same arm
with `debugMode=1` (step 3 covers this; step 2's `opt_ms` is emitted by
the same build), for the smallest and largest period:

```bash
for ARM in INCR_P1 INCR_P60; do
    OUT=/tmp/reconcile_${ARM}
    mkdir -p "$OUT"
    release/tests/RunOrchestrator "$TASKSET" "$OUT" "$ARM" 10000 1 \
      > "$OUT/stdout.log" 2> "$OUT/stderr.log"
done
```

Then for each arm, with `debugMode=1`, you have per interval:
- `opt_ms` — optimizer only (Timer A).
- `sim_ms` — whole interval (per-interval Timer B).
- `sim_ms − opt_ms` — non-optimizer per-interval work.

Three possible outcomes, each pointing to a different lever:

1. **`opt_ms` for INCRE intervals grows with interval index within INCR_P60,
   and is flat within INCR_P1.** → The growth is in the optimizer's
   incremental path. Resume the Fix C investigation (incremental RTA
   reuse) in `agents/debug_runtime0704_incr.md`.
2. **`opt_ms` is flat, but `sim_ms` (and thus `sim_ms − opt_ms`) grows.**
   → The growth is in `SimulateInterval`'s non-optimizer work (run-queue
   loop, `ObtainSP_TaskSet_And_TimeLimits`, or trace loading). The
   optimizer is not the culprit; look at `SimulationOrchestrator.cpp:527-543`.
3. **Both flat, but Timer B (`per_act_ms`) still grows with period.** →
   The growth is in the once-per-process work (`LoadIntervalConfigs` or
   `ExportResults`) or in process startup — i.e. Timer B's fixed overhead
   being divided by a different interval count. Check whether the interval
   count actually differs across arms (it should not for the same taskset,
   but verify).

This recipe is the one that will actually settle the open question from
the prior session. Run it before deciding whether Fix C is worth
implementing.

---

## 6. Quick reference — what to flip, what to run, what to read

| Want | Flip | Run | Read |
|---|---|---|---|
| Per-interval optimizer time (Timer A) | `debugMode: 1` | `RunOrchestrator … INCR_P<n> … 10000 1` | `stderr.log` → `[INCR-ET-DBG] interval= … opt_ms=` |
| Per-interval diff/variation | `debugMode: 1` | same | `stderr.log` → `[INCR-ET-DBG] OptimizeIncre: ndiff= nvar=` |
| Per-interval total (Timer B per-interval) | `debugMode: 1` + add timer in step 3.1 | same | `stderr.log` → `[INCR-ET-DBG] interval= … sim_ms=` |
| Whole-process per-activation (Timer B) | `debugMode: 0` | `repro_et_grows_with_period.py` | `et_repro_result.json` → `per_act_ms` |

**Always rebuild after flipping `debugMode` or editing the orchestrator:**

```bash
cmake --build release --target RunOrchestrator -j
```

**Always run serially** (one `RunOrchestrator` process at a time) for
timing numbers — parallel arms share cores and skew Timer B. The repro
harness already does this; if you run `RunOrchestrator` by hand, do not
background it.

---

## 7. Common mistakes (from the prior investigation)

- **Passing the wrong `duration_ms`.** The 4th CLI arg is the
  per-interval horizon, not the whole-simulation duration. For a 10 s
  interval pass `10000`. Passing `70000` produced a bogus 14.85 s ET.
  See [[runorchestrator-duration-arg-semantics]].
- **Comparing `opt_ms` to `per_act_ms` directly.** They have different
  numerators (optimizer call vs whole process) and the per-act one is
  divided by the interval count. Always label which timer a number came
  from.
- **Dividing `opt_ms` by `sp_dag_calls` to get a "per-DAG-eval cost".**
  Don't. `opt_ms` for the REOPT path includes the K-beam search
  (`GetRTA_OneTask` calls) that is NOT counted in `sp_dag_calls`, so the
  division overstates REOPT's per-call cost. The per-`ObtainSP_DAG` cost
  is effectively the same for both paths (same function).
- **Running with `debugMode=1` for Timer B numbers.** The instrumentation
  adds overhead. Use `debugMode=0` for any number you want to compare to
  the A/B table; use `debugMode=1` only for the breakdown.
- **Assuming DAG chains matter.** The simulation-experiment tasksets have
  zero chains/edges/predecessors. Do not invoke chains to explain cost
  differences; the DAG is flat.
- **Building debug for timing.** `build/` is `-g` (DEBUG); `release/` is
  `-O3 -DNDEBUG`. Always time against `release/`.

---

## 8. Pointers to the surrounding investigation

- Full prior record: `agents/debug_runtime0704_incr.md` (§10 call-count
  profiling, §11 ndiff ground-truth + Fix D death).
- Memory: `~/.claude/projects/-home-zephyr-Programming-ROS2-SP-APPs/memory/p25-incr-et-grows-with-period.md`.
- Related: [[p24-reoptimization-design]], [[runorchestrator-duration-arg-semantics]],
  [[interval-sweep-stale-flags-bug]].
- Repro harness: `simulation_experiments/repro_et_grows_with_period.py`.
- Dispatcher (REOPT vs INCRE selection):
  `sources/Optimization/OptimizeSP_TL_Incre.cpp` —
  `Optimize_w_TL_ScratchOrIncre`.
- Incremental optimizer (the INCRE path's per-variation work):
  `sources/Optimization/OptimizeSP_Incre.cpp` — `OptimizeIncre`.
- From-scratch optimizer (the REOPT path's beam search):
  `sources/Optimization/OptimizeSP_Incre.cpp` — `OptimizeFromScratch`.
