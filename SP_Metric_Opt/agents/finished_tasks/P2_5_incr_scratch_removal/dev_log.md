# P2.5 — Remove the `INCR_SCRATCH` scheduler arm — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-11

- Task **created** per the user's directive ("add a related task, and remove it as
  planned") — continuing the P2.4 session after a context-limit break. P2.4 had shipped
  its behavior-preserving rename (`INCR_P<n>` → `INCR_Reopt_X`, Slices A+B + the 1-arg
  overload *test-site* migration) and **filed the `INCR_SCRATCH` removal as Step 5**, a
  separate result-changing sub-task. P2.5 is that sub-task, promoted to its own folder
  so P2.4's staged unit stays clean.
- **Mapped the full removal surface** via `grep -rn "INCR_SCRATCH"` (excl.
  `et_repro_result.json` run artifacts + `/build/`). Key finding: the surface is
  **broader than P2.4 Step 5's note stated.** P2.4 said "the E2 gate + Q2/Q3/E1 tuples";
  the verified reality is that `INCR_SCRATCH` is a **co-equal subject in FIVE of the six
  north-star gates** (Q1, Q2, Q3, E1, E2) — every gate that reads "INCR & SCRATCH...".
  So the removal collapses Q1/Q2/Q3/E1 to INCR-only AND deletes E2 outright (its only
  subject pair is INCR-vs-SCRATCH). The north-star drops 6 gates → 5.
- **Found a P2.4 Step-3 miss:** `tests/testIncreOpt_w_TL.cpp:428`
  (`opt_scratch.ReOptimizePeriodic(2)`) is still a **1-arg call**. P2.4's migration
  summary claimed "×5 sites in testIncreOpt_w_TL migrated" but `:428` (inside
  `OptimizeWithOptimizationSpace`) was missed. This must migrate to 2-arg
  (`ReOptimizePeriodic(dag_tasks, 2)`) BEFORE the 1-arg overload can be deleted — else
  the build breaks. Logged in `goal.md` § "C++ test gap P2.4 Step 3 missed".
- **Confirmed the interval-0 fallback survives the removal.** The
  `INCR_SCRATCH`-as-amnesiac mechanism is `IfInitialized()`-gated: a fresh optimizer
  has no incumbent → `ReconstructTimeLimitVecFromResOpt` would return all -1 → the guard
  falls back to the Gaussian-mean TL. This is exercised by ANY fresh-optimizer reopt,
  not just the INCR_SCRATCH dispatch — specifically by the
  `ReOptimizePeriodic_Interval0FallsBackToGaussianMean` test
  (`testIncreOpt_w_TL.cpp:1258`), which uses a fresh local `StartTLStub` (NOT the
  INCR_SCRATCH dispatch path). So the fallback behavior is tested independently and
  survives; only the `INCR_SCRATCH` *mode string* + its dispatch branch go away. The
  `:1206` / `:1253` comments that name `INCR_SCRATCH` as the canonical exerciser are
  reworded to name the mechanism (`IfInitialized()` / fresh optimizer).
- **Confirmed the 1-arg overload's only production caller** is the INCR_SCRATCH branch
  (`SimulationOrchestrator.cpp:346`, `scratch_opt.ReOptimizePeriodic(K)`). Every other
  caller is already 2-arg (P2.4 Step 3 migrated the test/example sites — except the
  `:428` miss). With the branch gone, the overload is dead → deleted here.
- **Wrote `goal.md`** (the full removal surface C++/configs/python-gate/python-non-gate/
  docs, the why-remove reasoning, the design decisions, Done-when, Out-of-scope) +
  **`tasks.md`** (Step 0 mapping DONE through Step 6 build+stage+index, with the TDD
  red→green on the python gate fixtures as the test surface).
- **Design decisions settled:** (1) full removal, no alias shim (stale config fails
  loudly via dispatch fall-through to RM, the P1.3 trap pattern); (2) E2 deleted, not
  repurposed (substituting Reopt_1 would re-state E3's first edge); (3) Q1/Q2/Q3/E1
  collapse to INCR-only; (4) the 1-arg overload deleted with the branch; (5)
  `INCR_Reopt_1` NOT relabeled to absorb SCRATCH's role (the amnesia axis is dropped,
  P1.6 isolates warm-start value a cleaner way); (6) history docs not rewritten.
- **No source code touched yet.** Next: Step 1 (TDD red — python fixtures drop SCRATCH).

- **Resumed after a context-limit break — CLOSEOUT 2026-07-11.** Re-established state by
  mapping the actual working tree (the dev log's "No source code touched yet" was stale:
  the prior session HAD done the C++ removal + the gate code + most of the python/config
  surface; only the test-file residue + the doc/index pass remained). Verified each step's
  real state, then finished the remainder:
  - **Step 1/2 (gate code + fixtures):** `evaluation_suite.py` was already collapsed to
    INCR-only (Q1/Q2/Q3/E1 check `INCR` alone; `evaluate_e2` deleted; 5 gates). The one
    residue was `test_evaluation_suite.py` — 2 stale section comments (`:255`/`:280`
    "INCR/SCRATCH") + 2 dead `INCR_SCRATCH` fixture rows in `test_main_pass_exit_zero`
    (`:547`/`:563`). The suite was already green (the dead rows are written into the
    synthetic run but no gate reads them — pure residue). Removed all four.
  - **Step 3 (C++):** verified DONE — the `INCR_SCRATCH` dispatch branch + the
    construction-condition string are gone from `SimulationOrchestrator.cpp`; the 1-arg
    `ReOptimizePeriodic(int K)` overload is gone from both `.h` and `.cpp` (only the 2-arg
    `ReOptimizePeriodic(const DAG_Model&, int)` remains); `:428` migrated to 2-arg;
    `RunOrchestrator.cpp` `--help` cleaned. 16/16 ctest green on the DEBUG build
    (`libSP_OPTDebug.so`).
  - **Step 4 (configs + python non-gate):** verified DONE — `evaluation_suite_config.json`
    + `p25_period_ab_config.json` carry only `_comment` prose mentions (already updated to
    say P2.5 removed SCRATCH); `compare_optimizers.py`, `repro_et_grows_with_period.py`,
    `aggregate_across_tasks.py`, both `debug_analysis` scripts have zero SCRATCH refs.
  - **Step 5 (docs):** north-star (`project_evaluation_northstar.md`) collapsed to the
    5-gate INCR-only form (deleted the "INCR cannot run slower than SCRATCH" E2 line +
    the SCRATCH mentions in the SP-quality bullets); `plan_publication_figures.md` dropped
    SCRATCH from Ab-A/Ab-B; P0.3 figure goal/tasks dropped the SCRATCH floor reference;
    P2.4 Step 5 marked DONE with a pointer here. **P1.6 goal reword DEFERRED to the user**
    — P1.6's premise (compare INCR vs SCRATCH, the user's stated purpose) is now obsolete;
    the surviving axis is vs `INCR_Reopt_1` (the always-reopt-with-memory arm), and a
    find-replace would mis-state its new purpose. P1.6 is PLANNING-ONLY; the user decides
    its re-framing. Flagged in `overall_tasks.md` + the top-level dev log + P2.4 Step 5.
  - **Step 6 (build + index):** 16/16 ctest + 287 python green; `grep` sweep clean (only
    intentional history mentions remain); `overall_tasks.md` got a P2.5 row + the P2.4 row
    + P1.6 suggested-order entry updated; top-level `agents/dev_log.md` appended the P2.5
    milestone.
- **Standing constraints honored:** no `git commit` (`git add` only — staged for user
  review); result-changing by design (its own review cycle); history docs NOT rewritten.
- **Final `grep -rn "INCR_SCRATCH"` code surface (excl. history + run artifacts):** zero
  outside the intentional `evaluation_suite.py` / `test_evaluation_suite.py` docstrings
  + the two config `_comment` strings (all three document that P2.5 removed it).

