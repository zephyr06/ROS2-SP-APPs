# Session Summary — clean_simulation branch (OS Context COMPACTED)

**Date:** 2026-06-21
**Branch:** `clean_simulation`
**Commit before session:** `fb51b619` (fix sp update bug in UpdateSP)

---

## 1. Incremental vs. Brute-Force Optimization Analysis

### Context
`AnalyzeIncrVsBr.cpp` loads a single taskset, runs both the **INCR** (`OptimizePA_Incre_with_TimeLimits`) and **BF** (`EnumeratePA_with_TimeLimits`) optimizers, and prints a detailed side-by-side comparison:
- SP values (optimizer-reported and exact re-evaluated)
- Priority assignments per task
- Time-limit assignments per task
- Gap analysis (absolute and percentage)

### Key Terminology Update
All references to the old name **BR** (brute-force) were renamed to **BF** across `.cpp` and `.py` files to avoid ambiguity with "brute-force" vs "best-response."

### Moved / Removed
- `tests/AnalyzeIncrVsBr.cpp` → **deleted** (redundant with `analyze_incr_vs_br.py`)
- `tests/analyze_incr_vs_br.py` → moved to `tests/debug_analysis/`
- `tests/analyze_incr_vs_br_vary_k.py` → moved to `tests/debug_analysis/`

---

## 2. Simulation Orchestrator Baselines & Comparison Scripts

### C++ Side (`sources/RTDA/ImplicitCommunication/`)

#### `RunOrchestrator.cpp`
- Updated usage string to list **BF** instead of **BR**.
- Removed **CFS** from CLI-facing modes (CFS code remains in the C++ codebase but was removed from comparison scripts).
- Added **INCR_SCRATCH** mode which calls `OptimizePA_Incre_with_TimeLimits::OptimizeFromScratch_w_TL` directly for reference.

#### `SimulationOrchestrator.cpp / .h`
- Renamed mode string from `"BR"` to `"BF"` in `DeterminePrioritiesAndBudgets`.
- Added `INCR_SCRATCH` handling:
  ```cpp
  } else if (scheduler_mode_ == "INCR_SCRATCH") {
      OptimizePA_Incre_with_TimeLimits scratch_opt(dag_tasks, sp_parameters);
      scratch_opt.OptimizeFromScratch_w_TL(
          GlobalVariables::Layer_Node_During_Incremental_Optimization);
      res = scratch_opt.CollectResults();
  ```
- Updated all relevant `if`/`else if` initializer conditions to include `INCR_SCRATCH`.

### Python Side (root-level scripts)

#### `run_sim_experiments.py` — **CORE / kept in root**
- Full paper-evaluation pipeline aligned with `agents/tasks.md` lines 18-25.
- Runs multi-core `CSPSimulation_2`, computes miss rates, per-interval SP via `AnalyzeSP_Metric`, generates adaptation-over-time plots.
- Updated default schedulers from `["INCR", "INCR_SWAP", "BR", "RM_FAST", "RM_SLOW", "CFS"]` to `["INCR", "INCR_SWAP", "BF", "RM_FAST", "RM_SLOW"]`.

#### `run_e2e_eval.py` — **debug / moved**
- Lightweight wrapper around `RunOrchestrator`.
- Moved to `tests/debug_analysis/`; paths updated to use `_REPO_ROOT`.

#### `run_comparison.py` — **removed**
- Nearly identical to `run_e2e_eval.py` but simpler (3 runs, 30s traces, no exec-time tracking). Redundant.

#### `run_radius_comparison.py` — **debug / moved**
- Specialized ablation study for `TimeLimitSearchRadiusIncr` (R = 2, 3, 4, 5, 6).
- Moves to `tests/debug_analysis/`; paths updated to use `_REPO_ROOT`.

### Results
- Radius comparison summary CSV already generated (`tests/radius_comparison/radius_comparison_summary.csv`) before file moves.
- BF avg SP ≈ 1.91; INCR varies from 1.43 (R=2) up to 1.79 (INCR_SCRATCH).
- Execution times: BF ~5.0s, RM baselines ~1.3-1.4s, INCR variants ~1.3-2.6s depending on mode/radius.

---

## 3. Radius Comparison Findings

### Observed Data
| Mode | Radius | Avg_SP | Std_SP | Min_SP | Max_SP | Avg_Exec_Time_ms |
|------|--------|--------|--------|--------|--------|------------------|
| BF | — | 1.913912 | 0.946832 | 0.0 | 5.0 | 5069.89 |
| RM | — | 0.917430 | 1.146875 | 0.0 | 5.0 | 1420.17 |
| INCR_R2 | 2 | 1.434369 | 1.107435 | 0.0 | 5.0 | 1625.79 |
| INCR_R3 | 3 | 1.491495 | 1.086783 | 0.0 | 5.0 | 1799.99 |
| INCR_R4 | 4 | 1.523547 | 1.076445 | 0.0 | 5.0 | 1906.21 |
| INCR_R5 | 5 | 1.653499 | 1.028022 | 0.0 | 5.0 | 1970.22 |
| INCR_R6 | 6 | 1.667446 | 1.026090 | 0.0 | 5.0 | 2164.81 |
| INCR_NO_TL_R* | 2–6 | 1.160976 | 1.099115 | 0.0 | 5.0 | ~1770–1852 |
| INCR_WCET_R* | 2–6 | 1.105–1.255 | ~1.04–1.10 | ~0.0 | 5.0 | ~1329–1419 |
| INCR_SCRATCH_R* | 2–6 | 1.790941 | 1.028316 | 0.0 | 5.0 | ~2528–2613 |

### Why INCR_R6 Execution Time ≈ BF Execution Time
At R=6, the time-limit search window is already saturated (`MAX_TIME_LIMIT_OPTIONS = 10` in `yaml_exporter.py` clamps TL options). On a small n=6 taskset where only 2 tasks have TL configs:
- **BF** searches `10 × 10 = 100` TL combinations × full priority permutation enumeration.
- **INCR** searches roughly `10 + 10 = 20` TL coordinate-descent steps × local 1D priority search.

The overall execution times become comparable because INCR still performs many RTA calls during its priority search, and BF's exhaustive enumeration is efficient at this scale. The SP gap remains because BF explores all priority permutations exactly, while INCR only does local search.

### User Confirmed Insight
User acknowledged: "2 tasks have TL config, so BR searched for 10×10=100 combinations of TL, but INCR only searched for 10+10=20."

---

## 4. File Cleanup Summary

### Kept in Root
| File | Purpose |
|------|---------|
| `run_sim_experiments.py` | Core paper-evaluation pipeline (multi-core simulation + SP evaluation + plots) |

### Moved to `tests/debug_analysis/`
| File | Purpose |
|------|---------|
| `analyze_incr_vs_br.py` | Single-taskset INCR vs. BF comparison (debug) |
| `analyze_incr_vs_br_vary_k.py` | Vary-k analysis for INCR vs. BF (debug) |
| `run_e2e_eval.py` | Lightweight orchestrator runner (debug) |
| `run_radius_comparison.py` | Radius ablation study (debug) |

### Removed
| File | Reason |
|------|--------|
| `tests/AnalyzeIncrVsBr.cpp` | Redundant with `analyze_incr_vs_br.py` |
| `run_comparison.py` | Redundant with `run_e2e_eval.py` |

### `.gitignore` Updates
- `tests/*analysis*` still ignores artifacts.
- Added `!tests/debug_analysis/` and `!tests/debug_analysis/**` so scripts inside remain trackable.

---

## 5. CMakeLists.txt State
- `AnalyzeIncrVsBr` target removed.
- Remaining test executables:
  - `RunOrchestrator` ← main orchestrator binary
  - `AnalyzeSP_Metric` ← SP evaluation from traces
  - `EditYamlFile`, `AnalyzePriorityAssignment`, `AnalyzePriorityAssignmentIncrementalExample`
  - `test*.cpp` unit tests (gtest suite in DEBUG mode)

---

## 6. Critical Bug Fix: BF Was Optimizing Only Processor 0 Tasks

### Discovery (2026-06-22)
During follow-up evaluation, BF was still underperforming INCR in simulation. Root cause traced to `LoadIntervalConfigs()` in `SimulationOrchestrator.cpp`.

### Root Cause
`LoadIntervalConfigs()` only loaded `_p0.yaml` files:
```cpp
// OLD CODE
file_path = input_folder_ + "/taskset_characteristics_i" +
            std::to_string(interval_idx) + "_p0.yaml";
```
This meant BF was optimizing only the tasks assigned to **Processor 0**, completely ignoring Processor 1 tasks.

### Fix
Modified `LoadIntervalConfigs()` to prefer **merged** taskset files that contain ALL tasks (both p0 & p1):
```cpp
// NEW CODE
std::string file_path = input_folder_ + "/taskset_characteristics_" +
                        std::to_string(interval_idx) + ".yaml";
if (!std::filesystem::exists(file_path)) {
    // Fallback to legacy processor-specific file (p0 only)
    file_path = input_folder_ + "/taskset_characteristics_i" +
                std::to_string(interval_idx) + "_p0.yaml";
    if (!std::filesystem::exists(file_path)) {
        break;
    }
}
```
The merged `taskset_characteristics_N.yaml` files already exist in generated directories — they combine all tasks from both processors.

### Verification Results (after fix)
| Taskset | INCR SP | BF SP | Winner |
|---------|---------|-------|--------|
| experiment_4_tasks/taskset_0 | 1.86952 | 2.36674 | BF ✓ |
| experiment_4_tasks/taskset_1 | 2.10615 | 2.44645 | BF ✓ |

BF now correctly outperforms INCR as expected.

---

## 7. TIME_LIMIT Hard Limit Set to 10 Seconds

### Change
- **`sources/parameters.yaml`**: `TIME_LIMIT: 2` → `TIME_LIMIT: 10`
- **`run_sim_experiments.py`**: Removed the block that temporarily overrode `TIME_LIMIT` via regex (was setting 1s for 8-task, 2s otherwise). The persistent 10s value in `parameters.yaml` is now the single source of truth.

### Rationale
User requested BF to have a hard time limit of 10s. The previous setup had:
1. A default of 2s in `parameters.yaml`
2. A script-level override that further reduced it to 1-2s
This was insufficient for BF to complete meaningful enumeration on larger tasksets.

---

## 8. Files Created / Modified in This Session

| File | Action | Notes |
|------|--------|-------|
| `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp` | **EDITED** | `LoadIntervalConfigs()` now prefers merged taskset files |
| `sources/parameters.yaml` | **EDITED** | `TIME_LIMIT: 10` |
| `run_sim_experiments.py` | **EDITED** | Removed temporary TIME_LIMIT override block |
| `verify_bf_fix.py` | **CREATED** | Quick eval script to compare BF vs INCR on existing tasksets |

---

## 6. Constraints & Decisions from Session
- **Do not run or implement tasks from `agents/tasks.md` directly** — user explicitly prohibited this; only used the file to evaluate which scripts are core vs. debug.
- **CFS code stays in the C++ codebase** — removed only from comparison scripts and default scheduler lists.
- **BF naming** — enforced across C++ (`SimulationOrchestrator`, `RunOrchestrator`) and all Python scripts to maintain consistency.
