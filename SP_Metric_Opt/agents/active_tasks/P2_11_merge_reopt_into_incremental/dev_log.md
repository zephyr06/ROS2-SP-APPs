# P2.11 — Dev Log

## 2026-07-25 — Task filed; design recorded + evaluated

### Why this task
The user rejected the prior (A)/(B) design framing for merging incremental +
reopt as "too complicated with many changes that cannot be easily interpreted"
and proposed their own simpler design: **re-opt = the standard incremental path
+ (1) RM-Fast interval-0 init + (2) one upfront re-opt step at each interval
start adopting last interval's TLs → champion, then standard incremental walk
with patience+1.** The user asked to "first evaluate this new design" before any
implementation; this task records that evaluation and the resulting design.

### Evaluation of the user's design (the verdict)
**Headline: the design is ~90% already implemented — it is essentially the P2.9
flag-on path.** Mapping the user's 3 points onto the live code:

1. **Interval 0 = RM-Fast bootstrap → ALREADY EXISTS.** The first call (count==0)
   goes to `OptimizeIntervalFromScratch` → `ResetIncumbentBaseline(true)` with no
   incumbent → falls back to `RateMonotonicPriorityVec()` + min/Gaussian TL. So
   interval-0 RM bootstrap is current behavior, not a new delta. (Minor open
   question: current seeds Gaussian-mean TL via `InitializeTimeLimitsFromETConfig`;
   "RM-Fast" may mean strict min-TL via `SmallestTimeLimitVec` — flagged in
   goal.md open questions.)
2. **"One upfront re-opt on new DAG with adopted TLs → champion, then incremental
   walk, patience+1" → ALREADY EXISTS (P2.9 flag-on).** `RunReoptTLDescent` already
   does, verbatim:
   - `ResetIncumbentBaseline(true)` — re-score carried {pa,tl} under new DAG, commit
     as compare-guard baseline. (= "adopt TLs from last interval")
   - `EvaluateTimeLimitConfig_PAReopt(from_scratch=true)` — one from-scratch PA
     re-opt on the adopted TL → first candidate. `UpdateRecords`' strict-SP guard
     keeps the carried champion if the re-opt is worse. (= "results from this
     single re-optimization will be the champion")
   - Then (flag ON) `OptimizeOneTaskTimeLimit`, patience 1. (= "incremental
     walk, patience+1")

   So the user's #2 is the P2.9 flag-on path, exactly.

3. **The ONE genuine delta: Type-E in the reopt queue.** The user's phrase "do the
   same as the standard incremental optimization" has two readings:
   - **(a) Full merge** — reopt adopts the Type-E + Type-L serialized queue
     (`BuildSerializedTaskQueue`), like incremental. Currently reopt walks Type-L
     only (`CollectTLFlexibleTaskIds`). This adds targeted 1D patches on env-changed
     tasks. Genuinely new.
   - **(b) Loose merge** — reopt keeps Type-L only, uses SubIncremental arm +
     patience 1. Exactly P2.9 flag-on — zero new code.

   Chosen: **(a)** (maximal merge; the more interesting experiment). Fallback (b)
   if Type-E plumbing breaks the cache contract.

### Behavior-change assessment (NOT bit-identical)
Both readings change SP vs current default reopt (flag OFF = FullBeam walk):
- (b) replaces FullBeam (re-opts PA per trial TL) with SubIncremental (patches 1
  task per trial TL). Loses PA exploration; patience-1 partially compensates. =
  the P2.9 A/B already staged.
- (a) = (b) + Type-E — strictly more targeted work. Should be ≥ (b) in SP at
  modest extra cost.

This needs its own A/B (user already plans this). It is NOT a P2.10 bit-identity
gate — it is a behavior change.

### The "frequent reopt performs worse than incremental" hypothesis
Three candidate causes:
1. **Incumbent degradation** — reopt's from-scratch PA re-opt commits a worse
   permutation. **Mitigated in this design**: carried {pa,tl} is the compare-guard
   baseline; `UpdateRecords` adopts only strictly-better. The upfront GlobalBeam
   cannot regress past the carried champion. ✓ safe.
2. **TL thrashing** — reopt re-searches TLs every period; patience-1 allows a
   non-improving step that doesn't pay off next interval. **Not eliminated**
   (patience still 1). Likely culprit. This design inherits it.
3. **Type-E blindness (current reopt only)** — current reopt ignores env-changed
   tasks. Reading (a) ADDS Type-E → strictly better targeted response. This
   dimension should make this design BETTER than current reopt, narrowing the gap
   to incremental.

So the user's hunch ("similar to current frequent reopt") is most likely right
for cause 2, but cause 3 may improve on current reopt. Net effect is empirical —
the A/B will tell. The design is sound to test.

### Simplicity win (the user's stated motivation) — confirmed
End state under this design:
- **1 descent body** (`RunIntervalDescent(mode)`) replaces `RunIncrementalTLDescent`
  + `RunReoptTLDescent`.
- **1 walk arm** (`OptimizeOneTaskTimeLimit`); `WalkOneTaskTimeLimit_FullBeam`
  deleted (the `GlobalBeam` *eval* stays for the one upfront baseline call only).
- **P2.9 flag dies** (reopt always uses SubIncremental walk).

Under reading (a), the reopt-vs-incremental delta shrinks to ~2 things: (i)
patience 0 vs 1, (ii) one upfront GlobalBeam PA re-opt (reopt) vs dedicated
re-score (incremental). That is the maximal merge the user wants.

### Relationship to P2.10 + P2.9
- **P2.10 is the prerequisite** — it merged the duplicated Type-L walk into the
  shared `OptimizeOneTaskTimeLimit` and renamed the confusable symbols.
  This task builds on that shared arm.
- **P2.9 is subsumed** — its flag-on path IS this design's walk. This task deletes
  the flag (reopt always uses the sub-incremental walk) and the legacy full-beam
  arm. P2.9's A/B (flag 0 vs 1) becomes this task's A/B (current-default-reopt vs
  new-merged-reopt).

### Status
Awaiting Phase 0 confirmations (reading (a) vs (b); RM-Fast TL semantics; Type-E
cache-contract verification against the `.cpp`) before implementation. No code
edits yet.

## 2026-07-25 — Phase 0 verified against the `.cpp`; Phase 0.5 rename landed

### Phase 0 closed (read `OptimizeSP_TL_Incre.cpp`)
- **0b (reading (a)/(b)) — RESOLVED.** User approved the updated call path
  (which includes Type-E on the reopt queue). Reading (a) confirmed.
- **0c (RM-Fast TL semantics) — RESOLVED.** `ResetIncumbentBaseline(true)`
  (line 723-754) at interval 0 already seeds `SmallestTimeLimitVec()` +
  `RateMonotonicPriorityVec()` = RM-Fast (strict smallest-TL). The Gaussian-
  mean `InitializeTimeLimitsFromETConfig` is ONLY the seed vector passed into
  `OptimizeIntervalFromScratch` (line 774); the reset overrides it with the
  RM-Fast champion. So interval-0 = RM-Fast champion with **no** upfront re-opt
  step (that step runs only when `IfInitialized()` is true — line 732-740).
- **0d (Type-E cache contract) — VERIFIED.** `EvaluateTimeLimitConfig_SingleTaskPatch`
  asserts `|diff|<=1` (line 208); the comment at line 206-207 states Type-E =
  `|diff|==0` ("the env move was absorbed into `dag_tasks_` before the champion
  was built, so it cancels") → the lighter FullReuse path. So Type-E on the
  reopt path holds the contract, PROVIDED `dag_tasks_prev_pre_tl` is captured
  before the absorb (the Phase 3a plumbing).

### Phase 0.5 — rename `EvaluateTimeLimitConfig_PAReopt` → `_PAReopt` (landed)
**Why:** the name `GlobalBeam` is wrong — the function does both
`OptimizeFromScratch` (from_scratch=true) AND `OptimizeIncre` warm-start
(from_scratch=false), confirmed at line 148-160. It evaluates one TL vector by
RE-OPTIMIZING the priority assignment; "GlobalBeam" describes only the
from_scratch half. The header's own docstring already says "RE-OPTIMIZING the
priority assignment" (line 103).

**Name chosen:** `EvaluateTimeLimitConfig_PAReopt` — parallels the existing
`_SingleTaskPatch` (PA = priority assignment, an established term; Reopt =
re-optimize the PA). Avoids inventing new terminology.

**Scope:** behavior-neutral symbol rename across 5 code files
(`OptimizeSP_TL_Incre.{h,cpp}`, `Parameters.h`, `tests/testIncreOpt_w_TL.cpp`,
`tests/testScheduleSimulate.cpp`) via `sed`. 0 old-name hits remain.

**Verify:** `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
→ 17/17 ctest green (incl. `testIncreOpt_w_TL` 4.35s, `testScheduleSimulate`
0.08s). Header change → used `--clean-first` (stale-`.o` lesson from P1.21).

**Status:** awaiting user review + commit as a standalone modular step BEFORE
the behavior-changing merge. Per agent_coding_rules.md "work by module, commit
by module." The merge (Phases 1-3) builds on this renamed symbol.
