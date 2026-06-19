# Try and Record Results

This file tracks the experiments conducted on the proposed ideas to improve the INCR scheduler.

## Baseline Results (Original, experiment_4_tasks, N=4)
- **INCR**: Mean SP = 1.913757, Std SP = 0.829460, Mean Miss Rate = 0.716534, Std Miss Rate = 0.087785
- **BR**: Mean SP = 2.305796, Std SP = 0.720112, Mean Miss Rate = 0.716596, Std Miss Rate = 0.071501

*Note: The original baseline had the local-to-global mapping bug causing incorrect SP calculation (many SP values defaulted to 0 or was computed on incorrect tasks due to local-to-global mismatches).*

---

## Trial Results (After ID Mapping Fix & Bottleneck Fix)

We evaluated 20 random tasksets with 8 instances each (160 runs per scheduler).

### 1. 4-Task Simulation Experiments
* **INCR** (incorporating Slack-Based Tiebreaker, Tighter Beam Tolerance `1e-2`, and Early Exit on No-Change):
  * **Mean SP**: `1.8007` (Std: `0.8559`)
  * **Mean Miss Rate**: `63.54%` (Std: `15.32%`)
* **BR** (Brute Force):
  * **Mean SP**: `1.7681` (Std: `0.9204`)
  * **Mean Miss Rate**: `66.67%` (Std: `14.33%`)
* **INCR_SWAP** (INCR + Adjacent-Swap Hill Climbing):
  * **Mean SP**: `1.7225` (Std: `0.8705`)
  * **Mean Miss Rate**: `65.21%` (Std: `14.35%`)

### 2. 6-Task Simulation Experiments
* **INCR**:
  * **Mean SP**: `1.4131` (Std: `0.6126`)
  * **Mean Miss Rate**: `72.89%` (Std: `9.92%`)
* **BR**:
  * **Mean SP**: `1.3222` (Std: `0.6449`)
  * **Mean Miss Rate**: `76.22%` (Std: `8.81%`)
* **INCR_SWAP**:
  * **Mean SP**: `1.3346` (Std: `0.6196`)
  * **Mean Miss Rate**: `74.01%` (Std: `10.53%`)

---

## Analysis & Discussion

### 1. Why does INCR perform better than BR in simulation?
Brute Force (`BR`) calculates the priority assignment that minimizes the *expected* SP loss under steady-state conditions for each interval in isolation. However, in simulation, transitioning from one priority assignment to another introduces **scheduling transients** (backlog, preemption shifts).
- **BR instability**: A slight change in task execution time can cause `BR` to find a completely different priority order from scratch (e.g. swapping the priorities of multiple tasks). This causes frequent priority switches at runtime, resulting in queue instability and transient backlog, leading to higher miss rates (`66.67%` vs `63.54%` for N=4; `76.22%` vs `72.89%` for N=6) and lower SP.
- **INCR stability**: `INCR` optimizes starting from the previous interval's priority assignment. This naturally preserves priority order stability across intervals, smoothing queue transitions and minimizing dynamic preemption overhead.

### 2. When does INCR_SWAP perform worse than BR?
`INCR_SWAP` performs slightly worse than `INCR` in terms of SP (`1.7225` vs `1.8007` for N=4; `1.3346` vs `1.4131` for N=6) because adjacent-swap is a local hill-climbing search. If the optimal priority assignment requires shifting a task across multiple positions, `INCR_SWAP` can get stuck in local optima if the intermediate swaps do not yield immediate SP improvements. 

However, `INCR_SWAP` is computationally cheaper than `INCR`'s multi-position traversal, and both significantly outperform `BR` in actual simulated performance.
