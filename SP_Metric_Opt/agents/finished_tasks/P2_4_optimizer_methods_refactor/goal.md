# P2.4 — Optimizer Methods & Mode-String Refactor (Naming + Implementation Clarity)

**Priority:** P2 (should-do hygiene — naming/clarity, NOT a correctness fix. No bug in
the *computation*; the bugs are in *what things are called and how the names map to
behavior*, which is a readability + paper-credibility hazard.)
**Status:** IMPLEMENTATION COMPLETE 2026-07-11 (Slices A+B + the 1-arg overload test
migration: 16/16 ctest DEBUG green + 291 python tests green; `git add` staged, no commit —
see "Done when" below). The `INCR_SCRATCH` removal is filed as a separate sub-task Step 5,
NOT executed this pass. Previously IMPLEMENTATION GREENLIT 2026-07-11; FILED ONLY 2026-07-11.
**Trigger:** the `INCR_P1` inversion surfaced while answering "does INCR_P1 always call
incremental rather than re-optimization?" — the answer is the *opposite* of what the name
reads (`P1` = reopt period 1 = reopt **every** interval, **never** incremental). That
exposed a cluster of naming/implementation-clarity issues across the optimizer method
surface and the mode-string plumbing. This task collects them into one refactor rather
than fixing piecemeal.

## The issue catalogue (verified against current code 2026-07-11)

### I1 — `INCR_P1` reads as "incremental, period 1" but is the MAX-reopt arm (the trigger)

`Optimize_w_TL_ScratchOrIncre` (`sources/Optimization/OptimizeSP_TL_Incre.cpp:298-304`):
```cpp
int period = GlobalVariables::ReoptimizationPeriod;       // INCR_P1 → period=1
bool trigger_reopt = (reoptimization_interval_count_ % period == 0);
if (trigger_reopt)  ReOptimizePeriodic(...);   // from-scratch descent
else                OptimizeIncre_w_TL(...);   // warm-started incremental
```
`INCR_P1` parses to `ReoptimizationPeriod = 1` (`tests/RunOrchestrator.cpp:65`), so
`count % 1 == 0` is **always true** → reopt **every interval**, incremental **never**.
The `P<n>` knob runs the *wrong way* for a reader: larger `n` = **more** incremental,
**less** reopt. `INCR_P1` is the most-reopt extreme, not the least. This is a
paper-credibility hazard (a reviewer reading "INCR_P1" will assume the opposite of what
runs) and a contributor-trap (the author of P1.6 mis-stated it as "does periodic reopt"
before checking the code).

**Scope:** rename the family so the name reflects *reopt frequency* honestly, OR rename
to surface what the period counts. Candidates flagged in Design Decisions.

### I2 — Plain `INCR` vs `INCR_P1` are NOT reliably the same arm (silent YAML dependency)

`INCR` plain is dispatched identically to `INCR_P<n>`
(`SimulationOrchestrator.cpp:293, 313` — `scheduler_mode_ == "INCR" ||
IsINCRPeriodVariant(...)`), but it does **not** override `ReoptimizationPeriod`. So `INCR`
uses whatever `ReoptimizationPeriod` the YAML loaded (`sources/Utils/Parameters.cpp:20`
reads `loaded_doc["ReoptimizationPeriod"]`), a value the mode string does not surface. Two
configs both listing `INCR` can run different schedules if their YAMLs differ; `INCR` and
`INCR_P1` coincide only when the YAML happens to set `ReoptimizationPeriod=1`. The
`evaluation_suite_config.json` lists `INCR` and `INCR_P1` as **separate** arms
(`main_scheduler_list` line 12/31) — they are silently coupled to an invisible knob.

**Scope:** either (a) make `INCR` an alias that pins a documented default period, or
(b) drop bare `INCR` from the A/B configs in favor of explicit `INCR_P<n>`, or (c)
surface the YAML period into the mode string / a startup assert that fails if `INCR` is
used without an explicit period. Decision flagged.

### I3 — Stale `prev_optimizer_` references in comments (P0.5 removed the member)

P0.5 removed the `prev_optimizer_` member (the incumbent now lives solely in `res_opt_`,
P0.5 RESOLVED 2026-07-10, committed `7fa2e9d2`). But the *word* `prev_optimizer_`
survives in comments that describe the current logic:
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:329` — "persistent
  `incr_optimizer_`, so `prev_optimizer_` carries the prior interval's incumbent"
- `tests/testIncreOpt_w_TL.cpp:442, 1087` — describe the design in terms of `prev_optimizer_`

The logic these comments describe is correct (it now runs through `res_opt_`); the
*wording* is stale and will mislead anyone tracing the code. (Already noted in the P1.6
dev log + memory `p05-subsumes-tl-init-bug.md`; this task is where it gets *fixed*.)

**Scope:** rewrite those comments to name `res_opt_` / the P0.5 incumbent-state design.
Pure doc; no behavior change.

### I4 — `ReoptStartFromAdoptedTL` survives only in comments (P1.4 removed the flag)

P1.4 choice (b) removed the `ReoptStartFromAdoptedTL` knob + the `_ADOPTED` arms (the
adopted-TL seed is now the unconditional default, P1.4 implemented 2026-07-11). The flag
no longer exists in code, but the *name* appears in history comments:
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:28`
- `tests/RunOrchestrator.cpp:19`
- `tests/testIncreOpt_w_TL.cpp:1204`

These are *history* comments (they correctly describe what P1.4 removed), so they are not
wrong the way I3 is — but they should be checked for consistency and possibly condensed
so a reader doesn't go looking for a flag that isn't there.

**Scope:** review; tighten to a single "P1.4 removed this" pointer if the comment is
longer than the reader needs. Low priority.

### I5 — Method-name / role misalignment in `OptimizePA_Incre_with_TimeLimits`

The class surface (`sources/Optimization/OptimizeSP_TL_Incre.h`) has several methods whose
names understate or mislead about their role:
- `ReOptimizePeriodic` — at interval 0 this is also the **bootstrap** (it seeds the
  RM-fast incumbent, not just "re-optimizes"). The name hides the bootstrap role. The
  1-arg overload (`:84`) is the INCR_SCRATCH entry point; the 2-arg (`:88`) is the
  INCR/INCR_P<n> entry point — same name, different lifetimes, documented only in a
  long comment.
- `OptimizeIncre_w_TL` — "incremental" is accurate, but the `_w_TL` suffix is redundant
  (everything in this class is "with time limits"). Inconsistent with
  `Optimize_w_TL_ScratchOrIncre` which also carries the suffix.
- `Optimize_w_TL_ScratchOrIncre` — the dispatcher; the name buries that it is the
  *counter-driven entry point* and that `count==0` makes it the bootstrap.
- `ResetIncumbentBaseline(bool from_scratch)` — the bool controls whether this is the
  reopt path (re-eval + commit) or the incremental path (set gate to -1). The
  `from_scratch` name is inherited from the descent's vocabulary and conflates "reopt"
  with "from scratch"; a reader has to read the body to learn it really means
  "compare-and-keep baseline mode."
- `EvaluateTimeLimitConfig_ScratchOrIncre` — again `_ScratchOrIncre` overloaded onto an
  evaluator; the `from_scratch` bool selects warm-start vs fresh-build.

**Scope:** this is the largest, most opinionated part of the refactor and the most likely
to touch many call sites. Rename for role clarity + collapse redundant suffixes. This is
flagged as the part most likely to be DESCOPED if the user wants a small, safe pass — the
mode-string fixes (I1/I2) and the comment fixes (I3/I4) are independently shippable.

## Why this is P2 and not P1/P0

- **No computation bug.** Every issue here is a name or a comment lying about correct
  behavior. The schedules the code produces are right; the labels are wrong.
- **Paper credibility + contributor safety.** A reviewer or a new contributor reading
  `INCR_P1` will be misled (I1) — that is a real hazard for publication, but it is a
  *naming* hazard, not a result hazard. P0.3 (the prod figure run) is what makes results
  publishable; P2.4 makes them *legible*. It can run after P0.3 without invalidating any
  figure (renames are behavior-preserving).
- **Blast radius is the gate.** I1/I2 touch the mode-string plumbing, which appears in
  configs (`evaluation_suite_config.json`, `p25_period_ab_config.json`,
  `experiment_config.json`) AND in python output-dir-name parsing
  (`compare_optimizers.py`, `aggregate_across_tasks.py`, `evaluation_suite.py`, and their
  tests). A rename is mechanical but wide. The task's central risk is *not* the rename
  itself but keeping the config↔python↔C++ string surface in sync.

## Design decisions (SETTLED 2026-07-11 — implementation greenlit)

1. **Scope of this pass** (SETTLED). Ship **Slice A + Slice B + the 1-arg
   `ReOptimizePeriodic` overload removal**; defer Slice C (I5, conflicts with P1.6/P1.2
   in-flight edits); file the `INCR_SCRATCH` removal as a SEPARATE sub-task (Step 5), not
   this pass. Three coherent slices, independently shippable:
   - **Slice A (smallest, safest):** I3 + I4 — rewrite the stale `prev_optimizer_` /
     `ReoptStartFromAdoptedTL` comments. Pure doc, zero behavior risk, no call-site
     churn. Could ship today.
   - **Slice B (medium):** I1 + I2 — fix the mode-string naming (the `INCR_P1`
     inversion + the `INCR`-vs-`INCR_P1` silent-YAML coupling). Touches C++ parsing +
     configs + python. The trigger issue; the reason this task exists.
   - **Slice C (largest, opinionated):** I5 — rename the optimizer methods for role
     clarity. Widest blast radius; most likely to conflict with in-flight work (P1.6,
     P1.2 both touch this class). Most deferrable. **DEFERRED this pass** (see
     Out-of-scope).
   - **Settled:** ship A + B together + the 1-arg `ReOptimizePeriodic` overload removal;
   defer C; file `INCR_SCRATCH` removal as Step 5 (separate sub-task, not this pass).
2. **The `INCR_P<n>` rename direction** (SETTLED). `INCR_Reopt_X` where X in
   {1,5,10,30,60}; `Reopt` (not `RePeriod`) matches the codebase vocabulary
   (`ReOptimizePeriodic`, `ReoptimizationPeriod`). X=5 is a NEW arm (existing family is
   {1,10,30,60}) — ADDED, not renamed. Keeps the `INCR_` family prefix consistent with
   `INCR_NO_TL`/`INCR_WCET`/`INCR_SCRATCH`. Old `INCR_P<n>` names FAIL LOUDLY (mirror the
   P1.4 `_ADOPTED` hard-error pattern, NOT a silent alias). Candidates considered:
   - `REOPT_P<n>` — names what the period *counts* (reopt every n-th interval). `REOPT_P1`
     = reopt every interval (reads true). Inverts the prefix from "incremental" to "reopt".
   - `INCR_REOPT<n>` / `INCR_R<n>` — keep the `INCR` family, append the reopt period.
   - Keep `INCR_P<n>` but add a startup banner / `--help` line that prints the mapping
     (band-aid; does not fix the misread).
   - **Recommend `REOPT_P<n>`** — it makes the trigger arm read correctly and separates
   the reopt-period family from the pure-incremental arm (P1.6's `INCR_PURE`), so the
   baseline table reads as `REOPT_P1` (max reopt) … `REOPT_P60` (min reopt) vs
   `INCR_PURE` (zero reopt) vs `INCR_SCRATCH` (amnesiac reopt). But this is the user's
   call — it touches every config + the paper's baseline table.
   - **Settled:** `INCR_Reopt_X` (keeps the `INCR_` family prefix the user wants; `Reopt`
   matches codebase vocabulary). Rejected `REOPT_P<n>` (inverts the family prefix).
3. **`INCR` plain** (SETTLED). Stays canonical. Bare `INCR` = `INCR_Reopt_10` via
   `parameters.yaml:29 ReoptimizationPeriod: 10` (verified). I2 resolved by leaving bare
   INCR as the documented default-period alias and requiring explicit `INCR_Reopt_X` for
   any arm that sweeps the period — the A/B configs use explicit `INCR_Reopt_X`, never
   bare `INCR`, so no silent-YAML coupling in any published arm. See I2.
4. **1-arg `ReOptimizePeriodic(int K)` overload** (SETTLED): REMOVED. Only production
   caller was the INCR_SCRATCH branch (`SimulationOrchestrator.cpp:335`); the ~11
   test/example 1-arg call sites migrate to the 2-arg `ReOptimizePeriodic(dag_tasks, K)`
   form. `dag_tasks` is a fixture member or local `DAG_Model` in scope at every site
   (verified). Rationale (user): `dag_tasks` should be an explicit parameter, not the
   implicit `dag_tasks_` member. Behavior-preserving (the 1-arg body was
   `return ReOptimizePeriodic(dag_tasks_, K);`).
5. **Method renames (Slice C / I5)** (DEFERRED). The wider method-name sweep
   (`ReOptimizePeriodic` split-bootstrap, `_w_TL` suffix collapse, `from_scratch` bool
   rename) is NOT in this pass — conflicts with P1.6/P1.2's in-flight edits to
   `OptimizeSP_TL_Incre`. The user's "rename methods as planned" refers to the overload
   removal + the comment rewrites, NOT the I5 sweep. See Out-of-scope.
6. **`INCR_SCRATCH` removal** (SETTLED as a SEPARATE sub-task, Step 5, NOT this pass).
   The user said "we can remove it in a separate step." Step 5 deletes the
   `INCR_SCRATCH` dispatch branch + construction string + the arm from the 3 configs +
   python iteration tuples + ~30 test-fixture keys; deletes the E2 gate (collapses to
   E3's first edge once SCRATCH -> Reopt_1 and bare INCR=Reopt_10) + north-star line 10;
   updates P1.6's `goal.md` (P1.6 named INCR_SCRATCH as a comparison arm — removing it
   narrows P1.6 to the descent-cost axis only, which the user accepted). Step 5 is
   RESULT-CHANGING — its own review cycle, blocked-behind this pass.

## Done when (implementation phase — COMPLETE 2026-07-11)

- [x] Design decisions 1–6 settled with the user.
- [x] Slice A: stale `prev_optimizer_` comments rewritten (`SimulationOrchestrator.cpp:329`,
      `testIncreOpt_w_TL.cpp:442`); `:1087` + `:1204` confirmed intentional history (left
      as-is). `ReoptStartFromAdoptedTL` history comments reviewed/tightened
      (`SimulationOrchestrator.cpp:28`, `RunOrchestrator.cpp:19`,
      `testIncreOpt_w_TL.cpp:1204` — carried into the renamed `INCR_Reopt_X` prose).
- [x] Slice B: mode-string rename implemented in `RunOrchestrator.cpp`
      (`MaybeOverrideReoptPeriod`) + `SimulationOrchestrator.cpp` (`IsINCRPeriodVariant`
      + construction + dispatch) + both configs + python (`evaluation_suite.py` +
      `test_evaluation_suite.py` + `repro_et_grows_with_period.py`); a stale old-name
      `INCR_P<n>` config fails LOUDLY (the P1.4 `_ADOPTED` hard-error pattern, not a
      silent alias); the new `INCR_Reopt_5` arm added to the p25 + eval-suite configs.
- [x] 1-arg `ReOptimizePeriodic(int K)` overload test/example call sites migrated to the
      2-arg form (~8 sites); the overload itself is KEPT (decision (a)) for Step 5 to
      remove with the INCR_SCRATCH branch.
- [x] `cmake --build build --target check.SP_OPT -j5` (DEBUG) green; existing
      `testIncreOpt_w_TL` + 16/16 ctest green (Slices A+B + the overload migration are
      behavior-preserving — no test *expectations* changed, only symbol names + the new
      `INCR_Reopt_5` arm + renamed fixture keys). Python E3 fixtures 6 RED → 30 GREEN;
      full `tests/python/` 291 passed.
- [x] `agents/overall_tasks.md` + top-level `agents/dev_log.md` updated.
- [x] `git add` staged; user reviews. (No `git commit` — standing constraint.)
- [ ] **Step 5 (SEPARATE sub-task, NOT this pass):** `INCR_SCRATCH` removal + E2/north-
      star/P1.6 cleanup. Result-changing; its own review cycle.

## Out of scope

- `git commit` — user's standing constraint (`git add` only).
- Changing any *behavior* (for Slices A+B + the overload removal). Every rename must be
  behavior-preserving. If a rename would change a result, that is a different task (and
  likely P1/P0, not P2). NOTE: Step 5 (`INCR_SCRATCH` removal) IS result-changing — that
  is why it is a separate sub-task, not part of this behavior-preserving pass.
- Re-litigating P1.4 (seed policy) or P0.5 (incumbent state) — this task fixes *names*
  that reference those designs, not the designs themselves.
- **Slice C / I5 method renames** (`ReOptimizePeriodic` split-bootstrap, `_w_TL` suffix
  collapse, `from_scratch` bool rename) — deferred; conflicts with P1.6/P1.2's in-flight
  edits to `OptimizeSP_TL_Incre`. The user's "rename methods as planned" = the overload
  removal + comment rewrites only.
- The P1.6 arm (`INCR_PURE` / pure-incremental) — P2.4's mode-string rename should leave
  room for it / be consistent with it, but P1.6 is its own task.
