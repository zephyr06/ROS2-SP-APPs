# Sketch — Foundations & Framework (PW.1.1)

> Code-side truth for metric/prediction foundations + per-interval loop. Consumed by
> PW.1.4 (revision plan) + PW.3 (methodology). Paths relative to `SP_Metric_Opt/`.
> Code read 2026-08-02. Compressed 2026-08-02.

## TL;DR
- **Prediction = sliding-window ET sampling + Gaussian-distribution fit (mean, variance, min, max)**, NOT Gaussian-process regression. GP/GPR absent from `sources/` (grep empty); `GaussianDist` (`Probability.h:22-43`) is an ET distribution type, not a GP predictor.
- **Θ_i = Option A** — max tolerable miss prob, `Pr(r_i > D_i) ≤ Θ_i` (§5).

## Symbol cross-reference (locators for PW.3/PW.4)

| Concept | Symbol / file:line |
|---|---|
| Outer loop | `RunSimulation` `SimulationOrchestrator.cpp:277`; per-interval `SimulateInterval` :332,:554 |
| Persistent optimizer (warm-start) | `incr_optimizer_` :307; `res_opt_` carried :296-308 |
| Offline safe-fallback | `ComputeSafeFallback` :323 (PW.1.3) |
| Per-interval yaml load | `LoadIntervalConfigs` :59 → `dag_tasks_vecs_` :81 |
| PA+TL dispatch | `DeterminePrioritiesAndBudgets` :339,:560 (`INCR*`/`BF`/`DM*`) |
| ET sliding window | `ReadExtTimeData` `execution_time_estimator.h:49` (`T^W` = period, section9:32) |
| Gaussian fit | `GaussianDist(mu,sigma)` `RegularTasks.cpp:75-79`; `_min`/`_max` support bounds |
| PMF discretization | `FiniteDist(gauss,min,max,gran)` `Probability.cpp:18-44` (samples ctor :235) |
| pRTA | `ProbabilisticRTA_TaskSet` `RTA.cpp:129` → `_SingleCore` :60-115; `GetRTA_OneTask` :46 |
| HP-conv checkpoint | `hp_tasks_et_conv_vec[i]` :86,:103 (init `{0,1.0}` :87) |
| Tail compression | `CompressDeadlineMissProbability` `RTA.cpp:27,:194`; `ResolvePreemptionsAndCompress` :9 |
| Convolution | `Convolve` `Probability.cpp:113`; single-point `ConvolveSinglePoint` :94 |
| RTA cache | `RTACache` `RTA_Cache.h:61`; `ChampionState` :47-55; `Evaluate` :90; `Initialize` :70 |
| Reuse classifier | `ClassifyReusePerTask` :136 (Rule A=ET-chg, Rule B=priority-move) |
| Cheap commit | `AdoptChampion` :78; diff query `ComputeTaskSetDifference` :109/`IsSingleTaskChange` :119 |
| `|diff|>1` throw | `RTA_Cache.cpp:358`; opt-out `RTACacheOpt = optional<reference_wrapper<RTACache>>` |
| SP metric | `ObtainSP_DAG` `SP_Metric.cpp:95` = `ObtainSP_TaskSet` :54 + chains :113-129 |
| Miss prob | `GetDDL_MissProbability` `RTA.cpp:154-169` (mass strictly `> ddl`) |
| Safety gate | `ImportantTasksMeetThresholds` `SP_Metric.cpp:208-239` (reject `miss>Θ` :234) |
| `SP_Func` | `SP_Metric.h:31-41`; `PenaltyFunc` :24 (`-0.01*exp(10*|Θ−1|)`); `RewardFunc` :27 (`log(Θ+1)`) |
| Normalize | `interpolate(val,min_val,0,max_val,1)` :12,:40 |
| TL→ET bake | `ApplyTimeLimitsToTasksExecutionTime` :76 (`GetUnitExecutionTimeDist`, −1=no limit) |

## 1. Outer loop / dynamic environment / ET prediction

`RunSimulation` builds the persistent optimizer once, computes the offline safe-fallback (PW.1.3), then loops `SimulateInterval` per interval — the **collect→predict→optimize→update→simulate→score** cycle. `LoadIntervalConfigs` pre-loads one `taskset_characteristics_interval_<i>.yaml` per interval (each its **own** ET dist per task); `DeterminePrioritiesAndBudgets` dispatches `INCR*`/`BF`/`DM*`; `incr_optimizer_` carries `res_opt_` across intervals → past-0 **warm-starts**; `ApplyTaskConfigurations` writes priority + sets ET to dist average; FP schedule per core, SP scored. **Dynamic env:** ET non-stationary (distinct yaml per interval) → PA/TL **re-derived per interval** (warm-started); E drifts → ET dist drifts → re-optimize each interval. **ET prediction** (cite `§predict_ET_exp`, section9:16-35): sliding-window sampling (last `last_data_count` ET-log lines, window `T^W` = scheduler period) → Gaussian fit (mean/var/min/max → `GaussianDist(mu,sigma)`) → PMF (`FiniteDist(gauss,min,max,gran)` discretizes the Gaussian CDF — what pRTA convolves). **Why:** re-derive per interval (ET non-stationary — the dynamic-env content change); Gaussian fit not GP because the scheduler runs under "limited computation resources" → "linear complexity" (section9:18,32), assuming smooth within-window drift (section9:20-26).

## 2. pRTA — probabilistic response-time analysis

Fixed-point response-time **convolution of ET distributions**. `ProbabilisticRTA_TaskSet` partitions by `processorId` → `_SingleCore`: priority-sorted ascending (highest first); rolling `hp_tasks_et_conv` (convolution of all HP tasks' ET dists, init `{0,1.0}`) snapshotted per task into `hp_tasks_et_conv_vec[i]` (cache-reuse checkpoint). Per task, `GetRTA_OneTask`: `rta_cur = own ET`; `Convolve(hp_tasks_et_conv)` (HP interference); `ResolvePreemptionsAndCompress` (iteratively add HP job releases while `rta_cur.max_time` extends past next HP period and ≤ deadline, compressing mass beyond deadline into one point — tails accumulated, not discarded) → a `FiniteDist` RT PMF. `ConvolveSinglePoint` = TL'd point-mass fast path. **Why:** probabilistic ET demands **probabilistic schedulability** — a hard WCET wastes capacity or breaks on one overrun; convolving ET dists yields an RT *distribution* → a miss probability (not a binary verdict), the quantity the SP safety term scores.

## 3. RTA cache — incremental reuse across moves

`RTACache` memoizes RTA of **one champion** `{dag,pa,tl}` + per-core HP-prefix checkpoints. The serialized optimizer's single-change invariant (every candidate differs from the champion by ≤ one task's ET and/or priority position, `|diff|≤1`) lets `Evaluate` answer cheaply:
- `Initialize` — full N-task RTA + store champion. Once per interval/champion.
- `Evaluate` — `ClassifyReusePerTask` → seed candidate RTA with champion's, recompute only `NoReuse` tasks (suffix from changed task onward) via `GetRTA_OneTask`; does **not** mutate champion. **Rule A** (ET-chg): at/above changed task's min pos → `NoReuse`, above → `FullReuse`. **Rule B** (priority-move): only shift window `p_min≤pos≤p_max` → `NoReuse`, outside → `FullReuse`.
- `AdoptChampion` — cheap commit: promote a candidate with **no** full RTA; rebuild `hp_prefix_per_core` by re-rolling per-core ET-convolution.
- `ComputeTaskSetDifference`/`IsSingleTaskChange` — pure query; `|diff|>1` **throws**.

`RTACacheOpt` opts a caller out. **Why:** incremental moves share almost all the HP set with the champion — only one task's ET/pos changed → only a suffix of the per-task RTA chain differs. ~36% faster at N=10, **bit-identical SP** (cache avoids recomputation, never approximates).

## 4. SP metric + safety definition + Normalize

SP fuses **safety** + **performance** per task into one maximizable scalar. `ObtainSP_DAG` = node terms (`ObtainSP_TaskSet`) + chain/path terms (RTDA over `dag_tasks.chains_`). Per-task: `sp_overall += SP_Func(miss_prob_i, Θ_i) * w_i * perf_coefficient_i`; `miss_prob_i = GetDDL_MissProbability(rtas[i], deadline)` (mass **strictly above** deadline = `Pr(r_i>D_i)`); `Θ_i=thresholds_node[task_id]`, `w_i=weights_node[task_id]`; `perf_coefficient_i = GetPerfCoefficient()` (looked up from `timePerformancePairs` by avg ET, 1.0 if none — the **performance** term: better ET band → more contribution). `SP_Func(violate_prob, threshold)`: `min_val=PenaltyFunc(1,Θ)=-0.01*exp(10*|Θ−1|)`, `max_val=RewardFunc(0,Θ)=log(Θ+1)`; `Θ≥violate_prob` (safe)→`RewardFunc`, else `PenaltyFunc`. **Normalize:** `interpolate(val,min_val,0,max_val,1)` maps `[Penalty,Reward]`→`[0,1]` (certain miss→0, zero miss→1). `ApplyTimeLimitsToTasksExecutionTime` bakes a candidate TL into ET as a point mass (−1=no limit) — how TL optimization enters SP. **Why:** fuses safety (`Pr(r_i>D_i)≤Θ_i`) + performance into one maximizable scalar with smooth saturation rewarding under-threshold and penalizing over-threshold.

## 5. Θ_i convention — **Option A**

**Finding: Θ_i = Option A — max tolerable miss prob, `Pr(r_i>D_i)≤Θ_i`.** `GetDDL_MissProbability` returns the *miss* prob (`Pr(R>D)`); the gate `ImportantTasksMeetThresholds` rejects when `ddl_miss_chance > threshold` (enforced constraint = `Pr(r_i>D_i)≤Θ_i`); `SP_Func` rewards when `miss≤Θ`, penalizes when `miss>Θ` (Θ_i = crossover — max tolerable miss prob). **Not Option B:** code never computes `Pr(r_i≤D_i)` nor compares it against Θ_i. **PW.3/PW.4:** state safety as `Pr(r_i>D_i)≤Θ_i`; make Definition / safety function / Example 2 / experiment tables mutually consistent. **PW.4 flag:** Option A picked → experiment tables (§12–15) must be checked against the Option-A reading; re-run may be needed if authored under Option-B.
