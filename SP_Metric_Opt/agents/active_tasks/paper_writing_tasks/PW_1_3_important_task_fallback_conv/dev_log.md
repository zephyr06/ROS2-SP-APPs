# PW.1.3 important-task fallback & convergence — Dev Log

> On task completion, append a one-line milestone to the top-level `agents/dev_log.md`.

## 2026-08-02

- Task scaffolded (split from former PW.1). Owns the new paper claim — important-task
  safety-performance guarantee via safe fallback — + the offline convergence loop
  (former sketch items 4 and 5). Grounds the new "important-task safety guarantee"
  subsection PW.3 must add.
- Code read done INLINE (Explore subagent skipped — failed on PW.1.1 in this env):
  `OptimizeFallback.{h,cpp}`, `OptimizeSP_TL_Incre.{h,cpp}` (gate `:234-258`,
  dispatch `:722-883`, `UntilConvergence` `:889-897`, `ComputeSafeFallback`
  `:1002-1076`, `AdoptFallbackIfUnschedulable` `:1106-1191`,
  `DeadlineMonotonicPriorityVec` `:920-936`, `DetectETJump` `:131-151`),
  `OptimizeSP_TL_BF.cpp:37-98` (BF gate inside `Optimize()`), `WorstCaseDAG.cpp`,
  `SP_Metric.cpp:200-255`, `RegularTasks.h:100-117` (`is_important`), + the Python
  generator (`taskset_generator.py:566-581`, `yaml_exporter.py:73`,
  `generation_config_parser.py:373`) + orchestrator call site (`SimulationOrchestrator.cpp:321-323`).
- Wrote `sketch_fallback.md` (§1 guarantee, §2 convergence loop, §3 draft drift, §4
  symbol cross-ref).
- **Key finding — guarantee is SELF-guaranteed, not conditional.** Draft §13.3
  (`section13_real_exp_analysis.tex:72-83`) frames it as "guarantee IF ET is
  upper-bounded and known… in conflict with our unknown-environment proposal." The
  code does NOT require the user to upper-bound ET: it builds the worst-case DAG
  from the per-interval ET dists it already collects
  (`BuildDAGForObtainSafeFallBAckAcrossIntervals`) → certifies `{PA,TL}` on it
  (`ComputeSafeFallback`) → enforces at runtime via 3 triggers (a `DetectETJump`
  ratio≥1.5 / b-i during-walk gate in `UpdateRecords` / b-ii
  `AdoptFallbackIfUnschedulable` backstop) + the BF gate
  (`AdoptRmFastFallbackIfUnschedulable`). No-safe-solution → throw → regenerate.
  §11 `\sen` note ("how to provide hard safety guarantee") is ANSWERED now. Gates
  PW.1.4 §13.3/§11 rows + PW.3/PW.4 rewrite.
- Convergence loop OFFLINE ONLY: `OptimizeIncre_w_TL_UntilConvergence` (`:889-897`)
  called ONLY by `ComputeSafeFallback` (`:1039`); online dispatchers call single-pass
  `OptimizeIncre_w_TL` / `ReOptimizePeriodic`. No cap; monotonic termination.
- Gate predicate = Option A (consistent w/ PW.1.1); important label = top-50% by
  `sp_weight` (`IMPORTANT_TASK_RATIO=0.5`, Python-side, persisted to YAML).
- Deliverable DONE (`sketch_fallback.md`). No `.cpp`/`.py` changes; nothing committed.
