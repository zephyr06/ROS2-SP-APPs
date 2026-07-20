# P1.16 — RTA Cache Desynchronization on Rejected Walks

## The Goal
Fix the C++ `SIGABRT` crash (exit 134) occurring in `Reopt_X > 1` configurations (e.g., `INCR_Reopt_5/10/30/60`). 

During the serialized queue walk, speculative cache champion updates made within `OptimizeIncre_SingleTask` persist even if the overall candidate configuration is rejected by `UpdateRecords`. This desynchronizes the cache's champion from the global incumbent `res_opt_`. On the next task walk, the difference between the candidate (built from the old incumbent) and the cache champion (carrying the rejected changes) violates the $|diff| \le 1$ single-change invariant, causing a hard exception and crash.

The goal is to backup `rta_cache_` at the beginning of each queue walk step (`EvaluateTimeLimitConfig_SubIncremental`) and revert to the backup if the candidate configuration is not adopted by `UpdateRecords`.

---

## Proposed Changes

### C++ Optimizers

#### [MODIFY] [OptimizeSP_TL_Incre.h](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.h)
- Update `UpdateRecords` member function signature to return `bool` instead of `void`.

#### [MODIFY] [OptimizeSP_TL_Incre.cpp](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.cpp)
- Change the implementation of `UpdateRecords` to return `true` if `should_update` is true, and `false` otherwise.
- In `EvaluateTimeLimitConfig_SubIncremental`, create a backup copy `RTACache cache_backup = rta_cache_;` at the entry of the method.
- Check the return value of `UpdateRecords(challenger, time_limits)`. If it returns `false`, restore the cache: `rta_cache_ = cache_backup;`.

---

## Verification Plan

### Automated Tests
- Rebuild the binary (`make` in the build/release directories).
- Run the existing test suite:
  ```bash
  ./release/tests/testRTA_run
  ./release/tests/testOptimizeIncrePA_run
  ./release/tests/testSP_run
  ```
- Run the python test suite:
  ```bash
  pytest tests/python/
  ```

### Manual Verification
- Re-run the taskset simulation where the crash previously occurred (e.g., `taskset_3` under `INCR_Reopt_5`) and verify that it completes successfully without any crashes.
