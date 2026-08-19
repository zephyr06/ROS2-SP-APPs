# P1.30 — Tasks

- [x] File the bug; write `goal.md` with symptom + hypotheses
- [x] Capture BF per-leaf SP trace on mid config (all 12 TSP TLs × best PA)
- [x] Locate TL=1200 leaf: BF's true max SP at TL=1200 = 4.96962 (< 1100's 4.97052)
- [x] Conclusion: BF is CORRECT — no adoption bug, no skip, no timeout
- [x] Identify INCR inflation: cache path `ObtainSP_Full_From_NodeRTAs` reports
      4.98389 > canonical cap 4.96962 → INCR SP inflated
- [x] Eliminate H1 (baking): both paths bake identically (`GetUnitExecutionTimeDist`)
- [x] Empirically confirm cache returns wrong node RTAs (instrument
      `ObtainSP_Full_From_NodeRTAs` to diff vs fresh `ProbabilisticRTA_TaskSet`)
- [x] Localize: TSP's RTA (task id=0, pos=3) carries the divergence; all other
      tasks contrib_diff=0. cache UNDER-estimates TSP miss-prob (0.3618 vs
      0.4735) → inflates SP. Bug is in the `hp_tasks_et_conv` prefix the cache
      builds (RollPrefix) vs the oracle's rolling prefix, NOT in GetRTA_OneTask.
- [x] TDD RED test added: `TaskSetForTest_p130_rw_mid::BF_NotWorseThan_INCR`
      (tests/testBF_w_TL.cpp) — BF>=INCR on mid config; RED on HEAD.
- [x] Diff `RollPrefix` (RTA_Cache.cpp:24) vs oracle rolling prefix
      (RTA.cpp:~87,110-112) → BYTE-IDENTICAL. RollPrefix hypothesis OVERTURNED.
- [x] Add focused unit test: cache RTA == fresh RTA after small-relative TL
      change — `Evaluate_TLChange_SmallRelativeMagnitude_RecomputesChangedTask`
      (tests/testRTA.cpp). RED on HEAD, GREEN after fix.
- [x] Decide + apply fix: tight tolerance `!approx_equal(et, 1e-9)` in
      `FindTaskWithDifferentEt` (OptimizeSP_Incre.cpp:212), localized to the
      diff detector. Global `Value_Proba::operator==` 1e-1 left untouched.
- [x] Re-run P0.11 mid/pertask/mpc_important post-fix → INCR ≤ BF everywhere
      (mid: 4.97052==4.97052; pertask: 4.96487==4.96487; mpc: 4.97052==4.97052).
- [x] Full suite: 17/17 check.SP_OPT PASS; release RunSpeedTest PASS (0.056 s/int).
- [x] Cleanup: removed debug self-check from SP_Metric.cpp; restored debugMode=0.
- [ ] Verify whether prod/sim INCR SPs are inflated (PW impact) — sample a sim taskset
- [x] Update top-level `agents/dev_log.md` + MEMORY.md index entry
