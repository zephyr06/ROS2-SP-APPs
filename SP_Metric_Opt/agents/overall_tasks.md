# SP-Metric Optimization — Overall Task List

> **Archived Task Records**: Complete historical task details prior to 2026-07-25 are archived in [`agents/finished_tasks/overall_tasks_2026-07-25.md`](finished_tasks/overall_tasks_2026-07-25.md) and [`agents/finished_tasks/summary.md`](finished_tasks/summary.md). Canonical chronological narrative is in [`agents/dev_log.md`](dev_log.md).
> Priority Levels: **P0** = blocks publication / correctness; **P1** = active investigation; **P2** = should-do hygiene; **P3** = deferred / future.

---

## Active & Pending Tasks

### P0 — Blocks Publication / Correctness & Clean Baseline

| Task | Location / Folder | Status & Summary |
|------|-------------------|------------------|
| **P0.1** Persist adopted TL to YAML | — | ~~RESOLVED 2026-07-10~~ (subsumed by P0.5; inspectability write discarded). |
| **P0.2** Focused BF correctness audit | [`active_tasks/P0_2_bf_correctness_audit/`](active_tasks/P0_2_bf_correctness_audit/) | Verify `OptimizeSP_TL_BF` enumerates global optimum on small fixed taskset (`INCR ≤ BF`). |
| **P0.3** Run prod pipeline + generate core figures | [`active_tasks/P0_3_prod_figure_run/`](active_tasks/P0_3_prod_figure_run/) | THE publication deliverable. Core figures 1a/1c/1f/ab_a/ab_b/2/3 + NEW `fig_fallback_rejection_ratio` (gate-cost). `fig_p25_et_vs_period` DEFERRED → P3; P2.1 (fig2 sweep) + P2.6 (sim-RT SP) CLOSED 2026-08-02 (subsumed / deferred). |
| **P0.4** Project evaluation suite | [`active_tasks/P0_4_project_evaluation_suite/`](active_tasks/P0_4_project_evaluation_suite/) | Deterministic integration test on N=4/6/8 benchmarking avg SP + scheduler ET against north-star limits. |
| **P0.5** Redesign optimizer iteration process | [`finished_tasks/P0_5_optimizer_iteration_redesign/`](finished_tasks/P0_5_optimizer_iteration_redesign/) | ~~RESOLVED 2026-07-10~~ (committed `a8dba07f`→`7fa2e9d2`). Single-owned `res_opt_` state redesign. |
| **P0.6** Offline static solution (fall-back seed) | [`active_tasks/P0_6_static_solution/`](active_tasks/P0_6_static_solution/) | DESIGN locked 2026-07-27 (D1–D7). Offline seed DM-grouped PA + min-TL → WCET-mode TL walk (PA fixed) → skip-and-continue important-task filter → `static_solution_`. Depends on P0.8. **Superseded seed scheme by P0.9 (`AssignDMRespectingGroupOrder`).** |
| **P0.7** Online fall-back mechanism | [`active_tasks/P0_7_fallback_mechanism/`](active_tasks/P0_7_fallback_mechanism/) | DESIGN locked 2026-07-27 (D1–D7). Two online triggers: (a) ET-jump before opt → use `static_solution_` directly; (b) in-walk guard HALT on unsafe candidate → compare incumbent SP vs static SP → pick higher-SP winner. Depends on P0.6+P0.8. |
| **P0.8** Important-task schedulability (DM seed certification) | [`active_tasks/P0_8_important_task_schedulability/`](active_tasks/P0_8_important_task_schedulability/) | DESIGN locked 2026-07-27 (D1–D5). Step 1 + 2a + 2b LANDED + staged (NOT committed): `important_task_rta.py` (per-core RTA + WCET, 17 tests) + gate wrapper `run_full_generation_pipeline_with_important_task_gate` (`orchestrator.py`, seed-advancing retry loop, budget 20, loud raise). Split `run_full_generation_pipeline` → shell + `_run_pipeline_with_cfgs` (no default args). `pytest Gen_Taskset/tests/` = 47 green. DEFERRED: Step 2 prod wiring + Step 3 rejection-rate. **RTA sort switched period→deadline (DM) by P0.9.** |
| **P0.9** System-wide DM + important-first priority lock (seed PA) | [`active_tasks/P0_9_dm_and_important_first_priority/`](active_tasks/P0_9_dm_and_important_first_priority/) | ~~LANDED + FULLY COMMITTED 2026-07-28~~ (`0b9dae4a` Step1, `6a35080b` Step2 C++, `352f13d5` Step2 configs/py, `7dd1a7ac` Step4 records, `a6b06a22` Step3 py RTA). Seed PA plain-RM → Deadline-Monotonic + important-first group lock, system-wide. 17/17 ctest + 43+351/353 py green. D6 SP-shift = behavior change; prod A/B = user-go only. Blocks P0.6+P0.8. |

---

### P1 — Active Investigation & Core Optimizations

| Task | Location / Folder | Status & Summary |
|------|-------------------|------------------|
| **P1.1** P25 residual investigation | [`active_tasks/P1_1_p25_residual_investigation/`](active_tasks/P1_1_p25_residual_investigation/) | Reconcile 2-vs-8 changed-task-count, equal-radii A/B, confirm Fix D inertness. |
| **P1.3** `ReoptStartFromAdoptedTL` regression | [`finished_tasks/P1_3_adopted_tl_regression/`](finished_tasks/P1_3_adopted_tl_regression/) | ~~RESOLVED 2026-07-11~~ (Root cause = stale `release/` binary). |
| **P1.4** Reopt seed TL = carried incumbent | [`active_tasks/P1_4_reopt_seed_from_incumbent/`](active_tasks/P1_4_reopt_seed_from_incumbent/) | Algorithm-derived seed TL from carried `res_opt_` incumbent. Removed `_ADOPTED` arms. |
| **P1.5** Add INCR_Px_INIT baselines | — | ~~RETIRED 2026-07-11~~ (Invalidated by P1.4 choice b). |
| **P1.6** Pure-incremental baseline (RM-fast seed) | [`active_tasks/P3_6_incr_only_baseline/`](active_tasks/P3_6_incr_only_baseline/) | Pure incremental arm bootstrapped from RM-fast without periodic full re-optimization. |
| **P1.7** Simulator CPU partitioning mismatch | [`finished_tasks/P1_7_cpu_partition_mismatch/`](finished_tasks/P1_7_cpu_partition_mismatch/) | ~~RESOLVED 2026-07-12~~ (Partitioned `RunQueue` per `processorId`). |
| **P1.8** INCR_WCET outperforms INCR investigation | [`active_tasks/P1_8_incr_wcet_outperforms_incr/`](active_tasks/P1_8_incr_wcet_outperforms_incr/) | Feasibility clamp implemented (`clamp_avg_et_to_period` @ 0.95); awaiting user A/B re-run. |
| **P1.10** Serialized single-task incremental opt | [`active_tasks/P1_10_serialized_incremental_optimization/`](active_tasks/P1_10_serialized_incremental_optimization/) | Phase 1 complete & committed (`3d2f9b28`). `FindEnvTaskWithDifferentEt` filter landed. |
| **P1.11** Incremental RTA Patching (cache surface) | [`active_tasks/P1_11_incremental_rta_patching/`](active_tasks/P1_11_incremental_rta_patching/) | `RTACache` single-champion cache class design and leaf header completed. |
| **P1.12** Integrate RTA cache into incremental opt | [`active_tasks/P1_12_integrate_rta_cache/`](active_tasks/P1_12_integrate_rta_cache/) | Wire `RTACache` into incremental optimizer evaluation loop. Increment 2a done. |
| **P1.13** RTA cache for priority optimization | [`active_tasks/P1_13_rta_cache_priority_opt/`](active_tasks/P1_13_rta_cache_priority_opt/) | Base-class RTA cache integration for 1D priority variations in `OptimizePA_Incre`. |
| **P1.14** BF execution time violates TIME_LIMIT cap | [`active_tasks/P1_14_bf_time_limit_violation/`](active_tasks/P1_14_bf_time_limit_violation/) | Committed (`bfbec7e5`). Whole-call `BFDLSharedBudget` timer guard implemented. |
| **P1.15** Silent sim failure inflates aggregated A/B | [`active_tasks/P1_15_silent_sim_failure_inflates_agg/`](active_tasks/P1_15_silent_sim_failure_inflates_agg/) | Python harness crash detection, loud failure reporting, and non-zero exit on unequal N. |
| **P1.16** RTA Cache Desync on Rejected Walks | [`finished_tasks/P1_16_rta_cache_desync_fix/`](finished_tasks/P1_16_rta_cache_desync_fix/) | ~~RESOLVED 2026-07-19~~ (Revert cache champion state on rejected queue walk step). |
| **P1.17** Remove redundant ops in RTA cache/opt | [`active_tasks/P1_17_rta_cache_and_opt_redundant_ops/`](active_tasks/P1_17_rta_cache_and_opt_redundant_ops/) | Refactor unnecessary copies, re-bake operations, and redundant partitions in hot path. |
| **P1.18** ClassifyReusePerTask fine-grained reuse | [`finished_tasks/P1_18_classify_reuse_per_task_more_types/`](finished_tasks/P1_18_classify_reuse_per_task_more_types/) | ~~RESOLVED 2026-07-24~~ (Committed `09d1fca9`). Rule A & Rule B fine-grained per-task reuse. |
| **P1.19** Pipeline `--rerun_mode` flag | [`finished_tasks/P1_19_e2e_rerun_mode/`](finished_tasks/P1_19_e2e_rerun_mode/) | ~~RESOLVED 2026-07-24~~ (Committed `8279ae0d` / `f0858eef`). `clear_all` & `clear_results` modes. |
| **P1.20** Processor map vectorization | [`finished_tasks/P1_20_processor_partition_map_to_vector/`](finished_tasks/P1_20_processor_partition_map_to_vector/) | ~~RESOLVED 2026-07-23~~ (Converted processor-to-task-set maps to O(1) flat vectors). |
| **P1.21** Abstract RTA cache transactions (RAII) | [`finished_tasks/P1_21_rta_cache_transaction_abstraction/`](finished_tasks/P1_21_rta_cache_transaction_abstraction/) | ~~RESOLVED / REVERTED~~ (Superseded & reverted by P1.25). |
| **P1.22** RTA cache transaction slowdown | [`finished_tasks/P1_22_investigate_rta_cache_transaction_slowdown/`](finished_tasks/P1_22_investigate_rta_cache_transaction_slowdown/) | ~~CLOSED 2026-07-23~~ (Superseded by P1.25). |
| **P1.25** Remove RTA cache transaction layer | [`finished_tasks/P1_25_remove_rta_cache_transaction/`](finished_tasks/P1_25_remove_rta_cache_transaction/) | ~~RESOLVED 2026-07-23~~ (Committed `0448db9c`). Eager backup/restore on reject branch. |

---

### P2 — Should Do (Figure Safety & Refactoring Hygiene)

| Task | Location / Folder | Status & Summary |
|------|-------------------|------------------|
| **P2.1** Confirm Fig 2 sweep runs in prod | [`finished_tasks/P2_1_fig2_sweep_confirmation/`](finished_tasks/P2_1_fig2_sweep_confirmation/) | ~~CLOSED 2026-08-02~~ (superseded by P0.3; not worked). Verification task for already-built `fig2_sp_vs_interval`; done-when subsumed by the P0.3 prod run. Stale-flags crash already fixed in code; memory `interval-sweep-stale-flags-bug` → RESOLVED. |
| **P2.2** Doc & memory hygiene | [`active_tasks/P2_2_doc_memory_hygiene/`](active_tasks/P2_2_doc_memory_hygiene/) | Update stale memory files and investigation notes. |
| **P2.3** `FiniteDist::approx_equal` cleanup | [`active_tasks/P2_3_finite_dist_dead_code/`](active_tasks/P2_3_finite_dist_dead_code/) | Clean up / wire up `approx_equal` delegation in `Probability.cpp`. |
| **P2.4** Optimizer methods & mode-string refactor | [`active_tasks/P2_4_optimizer_methods_refactor/`](active_tasks/P2_4_optimizer_methods_refactor/) | Refactored mode strings (`INCR_Reopt_X`), updated tests and comments. |
| **P2.5** Remove `INCR_SCRATCH` scheduler arm | [`finished_tasks/P2_5_incr_scratch_removal/`](finished_tasks/P2_5_incr_scratch_removal/) | ~~RESOLVED 2026-07-11~~ (Removed `INCR_SCRATCH` branch & 1-arg overload). |
| **P2.6** Sim-RT-based SP metric | [`finished_tasks/P2_6_sim_rt_based_sp_metric/`](finished_tasks/P2_6_sim_rt_based_sp_metric/) | ~~CLOSED 2026-08-02~~ (deferred → P3 `optional_figures`; not worked). Report analytical SP AND true SP from `job_history_` RT samples. Big lift (D1–D4 open), zero implementation since 2026-07-12, no gate reads it. Spec preserved for revival. |
| **P2.7** Code comment cleanup | [`active_tasks/P2_7_code_comment_cleanup/`](active_tasks/P2_7_code_comment_cleanup/) | Simplify and clean code comments across optimizer and RTA headers. |
| **P2.8** Scripts & configs refactor | [`active_tasks/P2_8_scripts_and_configs_refactor/`](active_tasks/P2_8_scripts_and_configs_refactor/) | In progress. Consolidated JSON configs (`paper_simulation_config.json`) & entry scripts. |
| **P2.9** Speed up reoptimization | [`finished_tasks/P2_9_speed_up_reoptimization/`](finished_tasks/P2_9_speed_up_reoptimization/) | ~~CLOSED 2026-07-25~~ (superseded/moot; flag `ReoptimizationUseSubIncrementalWalk` deleted by P2.11 Phase 2b). Lever A walk switch committed `5dfd146e`/`52e29e90`/`a6922ff5` (prod bit-identical, flag stayed OFF). |
| **P2.10** Unify inc/reopt TL walk | [`active_tasks/P2_10_unify_inc_reopt_walk/`](active_tasks/P2_10_unify_inc_reopt_walk/) | CORE LANDED `6f7ed838` (merged duplicated Type-L walk block into shared `OptimizeOneTaskTimeLimit`; 17/17 ctest). 13-fn self-describing rename DROPPED (out of scope); code keeps old names. Gate = bit-identical SP @ N=16, P2.9 flag OFF. |
| **P2.11** Merge reopt into incremental | [`active_tasks/P2_11_merge_reopt_into_incremental/`](active_tasks/P2_11_merge_reopt_into_incremental/) | A/B ACCEPTED 2026-07-25 (N=10 gate). Phase 2 (behavior change: legacy reopt arm + P2.9 flag deleted; merged sub-incremental walk + Type-E queue + patience+1 = unconditional reopt) committed `76c45114`. Phase 1b-1e (behavior-neutral: two descent bodies → `RunIntervalDescent`+`SeedBaselineAndArmCache`+`IntervalDescentMode` enum; old bodies = 1-line delegating test seams) committed `bd9f5912`. REMAIN: Phase 5b (vs pure incremental, deferred). |
| **P2.12** Reopt speed benchmark | [`finished_tasks/P2_12_reopt_speed_test/`](finished_tasks/P2_12_reopt_speed_test/) | ~~CLOSED 2026-07-25~~ (committed `6b5065c3`). `RunSpeedTest.cpp` (release) `INCR_Reopt_1`/`_10` @ N=8: 0.042/0.013 s/act vs 0.1 threshold → PASS. Resolves "optimization got slower" (was DEBUG-build artifact). |
| **P2.13** Important-task DDL vs SP metric study | [`active_tasks/P2_13_important_task_ddl_vs_sp_metric/`](active_tasks/P2_13_important_task_ddl_vs_sp_metric/) | Analysis study on `Important_Miss_Rate` vs `Mean_SP_Metric` sensitivity. |
| **P2.14** Remove `SP_THRESHOLDS_SET` | [`finished_tasks/P2_14_sp_threshold_set_removal/`](finished_tasks/P2_14_sp_threshold_set_removal/) | ~~CLOSED 2026-07-25~~ (code-complete, committed `5c782bad`; Step 5 empirical analysis deferred to P2.13). Standardized on `SP_THRESHOLD_RANGE: [0.001, 0.9]`. |
| **P2.15** Randomize `sp_weight` per task | [`finished_tasks/P2_15_sp_weight_randomization/`](finished_tasks/P2_15_sp_weight_randomization/) | ~~CLOSED 2026-07-25~~ (committed `771be079`; `SP_WEIGHT_RANGE: [0.1, 1.0]` sampling). 372/372 tests green. |

---

### P3 — Deferred / Future (Features or Non-Blocker Optimizations)

| Task | Location / Folder | Why Deferred |
|------|-------------------|--------------|
| **P1.2** Reopt incumbent degradation | [`active_tasks/P3_2_reopt_incumbent_degradation/`](active_tasks/P3_2_reopt_incumbent_degradation/) | Known theoretical hazard; out of immediate scope. |
| **P3.1** Efficiency optimizations bucket | [`finished_tasks/P1_1_efficiency_optimizations/`](finished_tasks/P1_1_efficiency_optimizations/) | Closed 2026-07-24 (speed adequate for paper needs). |
| **P3.11** Partial task-subset optimization | [`active_tasks/P3_11_partial_task_subset_optimization/`](active_tasks/P3_11_partial_task_subset_optimization/) | Future algorithm feature; walks subset of queue per interval. |
