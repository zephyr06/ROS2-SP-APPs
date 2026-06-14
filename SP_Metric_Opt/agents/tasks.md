# Simulation Experiment & Code Maintenance Tasks

## Code Maintenance & Compilation
- `[x]` Fix Google Test dependency header pollution by replacing global include paths with target-based includes in `CMakeLists.txt`
- `[x]` Resolve the ambiguous `IsXDigit` and `GTEST_FLAG_SET` compile errors in `gtest-all.cc`
- `[x]` Adjust `testIncreOpt_w_TL` debug timing threshold check (increase or conditionalize for Debug builds)
- `[x]` Fix incremental optimizer `opt_sp_` reset bug in `OptimizeIncre_w_TL`

## Code & Paper Alignment
- `[x]` Verify that `Gen_Taskset/gen_taskset.py` performs the GMM polar/Cartesian dataset generation

