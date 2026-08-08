# P1.27 — Tasks (working checklist)

> See `goal.md` for scope + hypotheses. Anomaly: BF (0.908552) < INCR_Reopt_10
> (0.922016) at N=4/dur600/interval10/seed1000 — violates `INCR ≤ BF`.

## Step 0 — Reproduce / confirm the numbers
- [x] Re-read `comparison_summary.csv`; record BF vs INCR_Reopt_10 gap.
- [x] Confirm `taskset_arm_status.csv`: all BF inst `OK` (no crash/silent fail).
      Ruled out 0-byte `run.log` (P1.15 fingerprint) — run.log is full.

## Step 1 — Localize WHERE BF loses (per-taskset)
- [x] Read BF and INCR_Reopt_10 `interval_sp_metrics.txt`; compute per-taskset.
- [x] Gap is ENTIRELY `taskset_2` (BF 0.717297 vs INCR 0.855923); other 9 BF ≥ INCR.
- [x] Losing intervals = the bimodal `0.527888` (RM-Fast fallback) BF values.

## Step 2 — Inspect the losing BF run.log
- [x] BF search completed (no `BFDLSharedBudget` cancel; exec 0.097 s ≪ 10 s).
- [x] `AdoptRmFastFallbackIfUnschedulable` (P0.10) fired: run.log shows BF FOUND
      `Optimal SP 0.793324→0.962717` (≥ INCR 0.954072) but reported `0.527888`.
- [x] BF's SP-max plan is gate-INFEASIBLE; a schedulable 0.954072 plan exists
      (INCR finds it). BF is dominated because it falls back, not because no
      better plan exists.

## Step 3 — Isolate the invariant on the losing taskset
- [x] RED test `TaskSetForTest_p127_taskset2_i0.BF_NotWorseThan_INCR_Reopt`
      reproduces in isolation: BF `sp_opt=1.976e-323` < INCR `0.954072`.

## Step 4 — Root cause + fix
- [x] Root cause: BF's important-task gate runs POST-enumeration only, not
      in-search. BF adopts the SP-max plan even when it fails the gate, then
      swaps down to RM-Fast.
- [x] Fix: gate each leaf in-search —
      `if (res_cur.sp_opt > res_opt.sp_opt && ImportantTasksMeetThresholds(
      dag_tasks, sp_parameters, res_cur.priority_vec, time_limit_for_task))`.
      Post-hoc `AdoptRmFastFallbackIfUnschedulable` kept as safety net.
- [x] GREEN: 17/17 ctest; release builds. Legacy bit-identical (gate vacuous
      w/o `is_important`).

## Step 5 — Close out
- [ ] Re-run the (single) losing taskset comparison to confirm the fix end-to-end
      (deferred — user's call; full sweep not re-run before root cause, now found).
- [x] Milestone to top-level `agents/dev_log.md`; cross-link P0.2.
- [x] `git add` only (no commit — user's standing constraint).

## Standing constraints
- No `git commit` (`git add` only).
- Do NOT re-run the full comparison sweep before the root cause is found.
