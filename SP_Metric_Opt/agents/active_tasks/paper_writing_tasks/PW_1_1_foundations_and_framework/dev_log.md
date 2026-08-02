# PW.1.1 foundations & framework — Dev Log

> On task completion, append a one-line milestone to the top-level `agents/dev_log.md`.

## 2026-08-02

- Task scaffolded (split from former PW.1 per the user: small sub-tasks, numeric
  sub-index notation matching `P1_2` etc.). Owns metric/prediction foundations +
  per-interval framework loop (former sketch items 1, 6, 7) **and** the Θ_i
  convention finding that gates PW.3/PW.4 Option A vs B.
- Code read done INLINE (the `Explore` subagent failed with
  `API Error: 400 Model not found` in this GLM-5.2 env, so read the 9 file groups
  directly): `SP_Metric.{h,cpp}`, `Probability.{h,cpp}`, `RTA.{h,cpp}`,
  `RTDA_Prob.h`, `RTA_Cache.h`, `OptimizeSP_Base.h`, `RegularTasks.{h,cpp}`,
  `execution_time_estimator.h`, `SimulationOrchestrator.cpp`; plus
  `§predict_ET_exp` (`section9_software_impl.tex:16-35`).
- Wrote `sketch_foundations.md` (4 items, each algorithm + motivation + file:line).
- **Θ_i convention RESOLVED → Option A** (`Pr(r_i>D_i) ≤ Θ_i`, max tolerable miss
  prob). `GetDDL_MissProbability` (`RTA.cpp:154-169`) returns `Pr(R>D)`;
  `ImportantTasksMeetThresholds` (`SP_Metric.cpp:234`) rejects on
  `ddl_miss_chance > threshold`; `SP_Func` (`SP_Metric.h:35`) rewards when
  `threshold >= violate_probability`. Code never computes `Pr(r_i≤D_i)` against Θ_i.
  Gates PW.3 §6 (state Option A) + PW.4 (experiment tables may need re-run check).
- Prediction method: sliding-window ET sampling (`ReadExtTimeData`, last-N lines,
  `execution_time_estimator.h:49`) + Gaussian-distribution fit (mean/var/min/max,
  `GaussianDist` `RegularTasks.cpp:75-79`) + CDF discretization to FiniteDist PMF
  (`Probability.cpp:18-44`). GP/GPR grep-confirmed absent.
- Deliverable DONE (`sketch_foundations.md`). No `.cpp`/`.py` changes; nothing committed.
