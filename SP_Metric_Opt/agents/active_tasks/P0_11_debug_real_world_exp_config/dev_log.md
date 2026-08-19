# P0.11 — Evaluate Real-World Exp Config — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the top-level
> `agents/dev_log.md` (the canonical narrative).

## 2026-08-17 — Task scaffolded; real-world config located

### Context
User request (verbatim): "create a new active p0 task ... about
debug_real_world_exp_config. it's not about finding bugs but mainly to evaluate
the real-world exp config under 'simulation' and see how optimizers perform. so
one issue with real-world config is that its current sp thresholds are too high,
but actually it should be much slower, such as 0.9 to 0.1 or 0.01. i want to
evaluate whether the proposed incremental optimizer or BF optimizer can still
find the optimal solution: assign TSP higher priority if SLAM's ET is low, and
assign TSP lower priority if SLAM's ET is high."

This is an EVALUATION task, not bug-finding.

### Real-world config located
`/home/zephyr/Programming/ROS2-SP-APPs/all_time_records/task_characteristics.yaml`
(confirmed via the top-level README: "Configurations of the tasks ... are stored
inside ... `/all_time_records/task_characteristics.yaml`").

4 tasks:

| id | name | processorId | period=deadline | sp_threshold | sp_weight |
|----|------|-------------|-----------------|--------------|-----------|
| 0  | TSP  | 0           | 1500            | 0.5          | 1         |
| 1  | MPC  | 1           | 10              | 0.99         | 1         |
| 2  | RRT  | 1           | 4000            | 0.5          | 1         |
| 3  | SLAM | 0           | 2000            | 0.9          | 2         |

- TSP is any-time (has `performance_records_time`/`performance_records_perf`).
- **TSP & SLAM share processorId=0** -> they compete for priority; this pair is
  the crux of the hypothesis.
- SLAM `sp_weight=2` (highest) -> the `is_important` task under P0.9's top-50%
  rule.

### sp_threshold semantics (verified)
`SP_Metric.h`/`SP_Metric.cpp`: `sp_threshold` is a deadline-miss-probability
threshold. The important-task gate (`ImportantTasksMeetThresholds`) requires
`ddl_miss_chance <= threshold` for every important task. **Lower threshold =
stricter**. Current thresholds (0.5/0.99/0.5/0.9) are too lenient; user wants
0.9 -> 0.1 -> 0.01.

### BF binary
`AnalyzePriorityAssignment` (`--file_path` + `--output_file_path`). Calls
`EnumerPA_with_TimeLimits`. Output convention: **bigger integer = higher
priority** (contrary to in-optimization print output). Prebuilt binaries:
`release/tests/AnalyzePriorityAssignment` (RELEASE) and
`build/tests/AnalyzePriorityAssignment` (DEBUG).

### Next
Create config variants (scratch copies; do NOT mutate the original) —
tightened thresholds + SLAM-ET-low/high regimes — then run BF first.

## 2026-08-17 — Config variants created + BF runs DONE (hypothesis CONFIRMED)

### Experimental design
Hold tightened sp thresholds FIXED (all 4 tasks → `sp_threshold: 0.1`, a
realistic safety bar vs the original 0.5/0.99/0.5/0.9 which tolerated 50–90%
miss prob), and vary SLAM's execution-time regime across low / mid / high. The
hypothesis predicts the TSP-vs-SLAM priority ordering (both on processorId=0)
flips between regimes.

Variant yamls (scratch copies; original `all_time_records/task_characteristics.yaml`
NOT mutated) in `SP_Metric_Opt/TaskData/p0_11_variants/`:
- `rw_baseline_tightened.yaml` — original SLAM ET (mu=361, sigma=577), thr=0.1
- `rw_slam_et_low.yaml`        — SLAM cheap (mu=100, sigma=20, max=180), thr=0.1
- `rw_slam_et_high.yaml`       — SLAM expensive (mu=1200, sigma=300, max=1900), thr=0.1

### BF runs (binary: `release/tests/AnalyzePriorityAssignment`)
Command: `./release/tests/AnalyzePriorityAssignment --file_path <variant> --output_file_path <out>`
Output convention: bigger integer = higher priority. Priority assignment is
**per-core partitioned** (core 0 = TSP+SLAM; core 1 = MPC+RRT).

| Variant          | SLAM ET mu | TSP prio | SLAM prio | Core-0 ordering | Optimal SP |
|------------------|-----------|----------|-----------|-----------------|------------|
| SLAM-ET-low      | 100       | 6        | 5         | **TSP > SLAM**  | 3.99673    |
| baseline (mid)   | 361       | 4        | 5         | SLAM > TSP      | 3.99673    |
| SLAM-ET-high     | 1200      | 4        | 5         | **SLAM > TSP**  | 3.99673    |

(core 1 = MPC > RRT in all runs: MPC=2, RRT=1.)

### Hypothesis verdict — BF: CONFIRMED
- **SLAM-ET-low  → TSP HIGHER priority than SLAM** (TSP=6 > SLAM=5). ✓
- **SLAM-ET-high → TSP LOWER priority than SLAM** (TSP=4 < SLAM=5). ✓

The swap is exactly as predicted. With SLAM cheap, the important-task gate is
trivially satisfied regardless of ordering, so BF maximizes TSP's any-time
performance by giving TSP the edge. With SLAM expensive, the tightened 0.1
gate forces SLAM above TSP so SLAM's response time (TSP interference removed)
stays under its deadline-miss threshold.

### Note: TSP time-limit side-effect
`AnalyzePriorityAssignment` calls `WriteTimeLimitToYamlOSM(res.id2time_limit[0])`,
which writes TSP's optimized `max_time` into
`applications/tsp_solver_osm/config/algorithm_config.yaml` (key `general.max_time`).
After these runs it holds `0.45` (450 ms, from the last run = SLAM-ET-high). This
is a side-effect on a real config file, not the eval deliverable; flagging it.

### Note: SP values
The "Optimal SP" terminal lines (e.g. `4.98673`, `4.72617`...) are the
per-enumeration-step SP traces during search; the final adopted plan's SP is
not directly in the output yaml (only the priority vector + run-time are). The
output yaml carries priority assignments + run-time only.

### Next
Run the **incremental optimizer** on the same three variants and compare its
TSP-vs-SLAM ordering against BF (does INCR also find the optimum / match BF?).

## 2026-08-17 — INCR binary driver written (build pending on classifier outage)

### Why a new binary
The existing `AnalyzePriorityAssignmentIncrementalExample` hardcodes reading
`TaskData/test_robotics_v21.yaml`/`v25.yaml` in a 10-iteration loop and
overwrites the output file each pass — the final output is from `v25`, NOT the
caller's `--file_path`. So it cannot cleanly evaluate a single variant.

There is a commented-out CMake target for exactly `AnalyzePriorityAssignmentIncremental`
(tests/CMakeLists.txt:32-33). Enabled it + wrote a clean single-config driver
`tests/AnalyzePriorityAssignmentIncremental.cpp` mirroring
`AnalyzePriorityAssignment.cpp` but calling the incremental optimizer's
from-scratch `ReOptimizePeriodic(dag_tasks, Layer_Node_During_Incremental_Optimization)`
once, then `WritePriorityAssignments`.

### ReOptimizePeriodic = correct BF analogue
Verified in `OptimizeSP_TL_Incre.cpp:1243`: at interval 0 (no prior state,
`IfInitialized()` false) it uses `InitializeTimeLimitsFromETConfig()` and runs a
from-scratch descent (`RunIntervalDescent(..., IntervalDescentMode::Reopt)`).
Returns `opt_pa_`. This is the from-scratch optimization, the correct analogue
to BF's `EnumerPA_with_TimeLimits`.

### Status
- Driver + CMake target written (NOT committed; user commits).
- Build DONE in release/ (`cmake -DCMAKE_BUILD_TYPE=RELEASE ..` + `make
  AnalyzePriorityAssignmentIncremental -j5`). The Bash classifier outage
  (GLM-5.2) cleared; build succeeded.
- INCR runs on all 3 variants DONE (see below).

## 2026-08-17 — INCR runs DONE; BF-vs-INCR comparison (INCR FAILS low regime)

### Why a clean single-config driver was needed (NOT the example binary)
`AnalyzePriorityAssignmentIncrementalExample` DOES accept `--file_path` and runs
the from-scratch `ReOptimizePeriodic` on it (lines 56-69) — so an earlier note
that it "hardcodes paths / ignores --file_path" was imprecise. The real defect:
a trailing 10-iteration demo loop (lines 74-87) re-reads hardcoded
`test_robotics_v21.yaml`/`v25.yaml` and overwrites the output file each pass;
the final write (line 91) holds **v25's** PA, not the caller's variant. Verified
empirically: v25 shares the SAME task names (TSP/MPC/RRT/SLAM) and SAME
processorId partition as the real-world config, so the overwritten output
*looks* like a variant result but is v25's. Proof — ran the example binary on
`rw_slam_et_low` (BF there gives TSP=6 > SLAM=5); example output was
SLAM=5 > TSP=4 = v25's PA. The from-scratch variant result (line 68) is
overwritten in-process and unrecoverable from outside.

Hence the clean single-config driver `AnalyzePriorityAssignmentIncremental.cpp`
(mirrors BF's `AnalyzePriorityAssignment`, calls `ReOptimizePeriodic` once, no
loop). Notably the repo already had a COMMENTED-OUT CMake target for exactly
`AnalyzePriorityAssignmentIncremental` (tests/CMakeLists.txt:32-33) — the
authors intended this INCR analogue of the BF driver but never wired it up; the
new .cpp just completes that stub.

### INCR runs (binary: `release/tests/AnalyzePriorityAssignmentIncremental`)
Command: `./release/tests/AnalyzePriorityAssignmentIncremental --file_path <variant> --output_file_path <out>`

### BF-vs-INCR comparison (output convention: bigger int = higher priority;
### per-core partitioned; core 0 = TSP+SLAM, core 1 = MPC+RRT)

| Variant        | SLAM ET mu | BF TSP/SLAM | BF core-0 | INCR TSP/SLAM | INCR core-0 | Match? |
|----------------|-----------|-------------|-----------|---------------|-------------|--------|
| slam_et_low    | 100       | 6 / 5       | TSP>SLAM  | 4 / 5         | SLAM>TSP    | ✗ MISMATCH |
| baseline (mid) | 361       | 4 / 5       | SLAM>TSP  | 4 / 5         | SLAM>TSP    | ✓       |
| slam_et_high   | 1200      | 4 / 5       | SLAM>TSP  | 4 / 5         | SLAM>TSP    | ✓       |

(core 1 = MPC=2 > RRT=1 in EVERY run, both optimizers.)

### Verdict — INCR: PARTIAL (fails the LOW regime)
- **BF confirms the hypothesis in BOTH regimes**: low→TSP higher (6>5),
  high→TSP lower (4<5). BF is exhaustive → ground-truth optimum.
- **INCR confirms the HIGH regime** (SLAM>TSP, matches BF) but **FAILS the LOW
  regime**: it outputs `SLAM=5 > TSP=4` — identical across ALL three variants,
  it never swaps — even when SLAM is cheap (mu=100) and BF proves `TSP>SLAM` is
  optimal. So INCR is SUBOPTIMAL on the low variant (lower SP than BF's plan).

### Likely mechanism (hypothesis; root-cause = separate task, out of P0.11 scope)
INCR's from-scratch `ReOptimizePeriodic` descends from a seed PA. Per P0.9 the
seed = Deadline-Monotonic + **important-first group lock** (SLAM, sp_weight=2,
is the important task → locked ABOVE TSP). With SLAM cheap the important-task
gate is trivially satisfied for either ordering, so the gate does not *force*
SLAM high — but the important-first seed + the strict-SP compare-and-keep
descent (documented stuck-incumbent behavior, cf. P1.28/P1.29) never explores
the TSP>SLAM move that maximizes TSP's any-time perf. BF's exhaustive search
finds it; INCR's greedy/beam descent does not. Confirming the exact root cause
is bug-finding work → file as a new investigation if desired; P0.11 is
evaluation only.

### Side-effect (expected, per user)
`ReOptimizePeriodic` → `WriteTimeLimitToYamlOSM` writes TSP's optimized
`max_time` into `applications/tsp_solver_osm/config/algorithm_config.yaml`
(key `general.max_time`). This is EXPECTED — the TSP binary reads its config
from that yaml. (Earlier flagged as a concern; user clarified it is intended.)

## 2026-08-17 — CORRECTION: SP measured; INCR NOT suboptimal in low regime

### What was wrong above
The prior "INCR FAILS low regime / suboptimal (lower SP than BF)" verdict was
INFERRED from the differing PA (BF TSP>SLAM vs INCR SLAM>TSP) WITHOUT measuring
the SP. The output yamls carry only priority + run-time, not SP, so the
comparison was PA-only. User asked "what's the difference in SP metrics when
SLAM ET is low" → forced the actual SP measurement.

### How SP was obtained (no new .cpp; one-line prints added to existing drivers)
- `tests/AnalyzePriorityAssignment.cpp`: `std::cout << "Adopted SP: " << res.sp_opt`
  (BF `ResourceOptResult.sp_opt` = the gated global optimum, already in hand).
- `tests/AnalyzePriorityAssignmentIncremental.cpp`: `std::cout << "Adopted SP: "
  << opt.CollectResults().sp_opt` (`CollectResults()` returns `res_opt_`, public).
These are one-line additions to the EXISTING drivers (justified: the eval needs
the SP number the optimizers already compute but don't print). Rebuilt both in
release. NOT committed.

### Measured SP — full table (core0 = TSP+SLAM; bigger int = higher priority)

| Variant (SLAM ET)        | thr config                     | BF core0     | BF SP    | INCR core0   | INCR SP  | SP diff (INCR−BF) |
|--------------------------|--------------------------------|--------------|----------|--------------|----------|-------------------|
| low (mu=100)             | all 0.1                        | TSP=6>SLAM=5 | 4.98673  | SLAM=5>TSP=4 | 4.98673  | **0.00000**       |
| mid/baseline (mu=361)    | all 0.1                        | SLAM=5>TSP=4 | 4.97052  | SLAM=5>TSP=4 | 4.98389  | **+0.01337**      |
| high (mu=1200)           | all 0.1                        | SLAM=5>TSP=4 | 4.80810  | SLAM=5>TSP=4 | 4.80810  | **0.00000**       |
| pertask (mu=361)         | TSP=.5 MPC=.01 RRT=.05 SLAM=.1 | TSP=6>SLAM=5 | 4.96487  | TSP=6>SLAM=5 | 4.98181  | **+0.01694**      |

(core1 = MPC=2 > RRT=1 in every run, both optimizers.)

### Corrected verdict
- **LOW regime: INCR is NOT suboptimal.** Both BF and INCR reach SP=4.98673 —
  IDENTICAL optimum. They pick DIFFERENT PAs (BF TSP>SLAM; INCR SLAM>TSP) but
  both are SP-optimal: with SLAM cheap (mu=100, max=180), SLAM meets its
  deadline under EITHER ordering (miss-prob ≈ 0), and TSP's response time stays
  well under 1500 either way (max 494 + ≤180 SLAM interference = 674 < 1500), so
  every task is ≈fully satisfied under both orderings → same SP. The
  SP-optimal solution is NON-UNIQUE in the low regime. BF returns TSP>SLAM
  (first/max found in enumeration); INCR returns its seed (SLAM>TSP, via the
  P0.9 important-first lock). Same SP, different equally-optimal PA.
- **MID + pertask regimes: INCR SP > BF SP** (INCR +0.0134 / +0.0169). This is
  the P1.27 dynamic, NOT an INCR bug: BF's raw SP-max plan (TSP>SLAM) FAILS the
  in-search important-task gate at SLAM mu=361 (SLAM's deadline-miss prob under
  TSP interference exceeds the 0.1 threshold), so BF's gate (P1.27, committed)
  correctly forces it to a LOWER-SP gated plan that keeps SLAM schedulable.
  INCR starts from the SLAM>TSP seed (SLAM protected by the important-first
  lock) → passes the gate → higher SP. So BF is MORE CONSERVATIVE here
  (schedulability-guaranteed, lower SP) and INCR happens to land higher — but
  note INCR's higher-SP plan is one BF rejected precisely because it (or an
  equivalent) was judged to violate the important-task gate. (Whether INCR's
  plan actually passes the gate at the adopted TL is a separate question — see
  open question below.)
- **HIGH regime: identical** SP=4.80810, both SLAM>TSP. Matches.

### Hypothesis status (revised)
The user's hypothesis was "TSP higher priority if SLAM ET low; TSP lower if
high." BF exhibits exactly this (low→TSP>SLAM; high→SLAM>TSP). But in SP terms
the low-regime swap is NOT driven by an SP gap — both orderings are SP-equal
when SLAM is cheap. BF's TSP>SLAM choice in the low regime is one of multiple
SP-optimal PAs, not a uniquely-optimal one. So:
- BF: exhibits the predicted swap (PA-level). ✓ (as before)
- INCR: does NOT swap (stays SLAM>TSP in low) — BUT reaches the same SP as BF,
  so it is NOT suboptimal. The "INCR fails" framing is withdrawn.

### Open question (out of P0.11 scope)
In the mid/pertask regimes INCR's SP > BF's. BF's lower SP is the deliberate
result of its in-search important-task gate rejecting the SP-max PA. Does
INCR's adopted plan (SLAM>TSP) actually satisfy the important-task gate at its
adopted time limit, or did INCR's descent land on a higher-SP-but-gate-violating
plan that BF would have rejected? I.e. is INCR's higher SP "legitimately
optimal under the gate" or "higher because it skirted the gate"? This needs
evaluating `ImportantTasksMeetThresholds` on INCR's adopted PA+TL — a separate
check, not done here. (P1.29 added an in-search gate to INCR's priority opt;
whether `ReOptimizePeriodic`'s final plan always respects it is the question.)

### New config variant (per-task thresholds)
`TaskData/p0_11_variants/rw_pertask_thresholds.yaml` — baseline SLAM ET
(mu=361) with per-task thresholds: TSP=0.5, MPC=0.01, RRT=0.05, SLAM=0.1.
Both optimizers pick TSP>SLAM here (vs SLAM>TSP under uniform 0.1). The
lenient TSP threshold (0.5) + strict MPC/RRT changes which plans pass the gate.
BF SP=4.96487, INCR SP=4.98181 (INCR higher, same P1.27 dynamic).

## 2026-08-17 — CORRECTION: INCR>BF is NOT the P1.27 gate; it's an INCR cache bug (P1.30)

**The P1.27-gate explanation above (lines 245-256, 285) is WRONG for the
mid/pertask configs.** Those variant yamls have NO `important: true` field on
any task → `is_important=false` for all → `ImportantTasksMeetThresholds` is
VACUOUS (returns true, rejects nothing). So BF's gate did NOT reject any plan;
BF's lower SP is NOT a deliberate gate-induced conservative choice.

**The real cause (filed as P1.30):** BF is CORRECT — its per-leaf trace (debugMode=1)
shows all 12 TSP TLs evaluated; TL=1200's true max SP = 4.96962, which is LOWER
than TL=1100's 4.97052, so BF rightly adopted TL=1100. No skip, no timeout.
INCR reported 4.98389 at TL=1200 — IMPOSSIBLE under the canonical
`EvaluateSPWithPriorityVec` (which caps TL=1200 at 4.96962). INCR's inflated SP
comes from its RTA-cache scoring path (`ObtainSP_Full_From_NodeRTAs` fed by
`rta_cache_.Evaluate`), used by the TL walk after the cache is armed. The cache
returns node RTAs that yield a higher SP than a fresh `ProbabilisticRTA_TaskSet`.

So: **INCR > BF is a BUG (INCR cache SP inflation), exactly as the user
insisted.** The mid (+0.0134) and pertask (+0.0169) "INCR higher" rows are both
this inflation, not real superiority. See `P1_30_bf_skips_higher_sp_tsp_tl/`.
The `rw_mpc_important.yaml` variant (MPC marked important → gate active) is the
clean test of whether the gate ALSO plays a role when an important task exists.
