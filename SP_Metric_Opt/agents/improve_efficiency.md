# Efficiency Improvements for SP Metric Optimizer

This document outlines memory and algorithmic optimizations to speed up the incremental priority assignment (`INCR`) scheduler and the Response Time Analysis (`RTA`) calculations.

---

## 1. Approved Memory and Algorithmic Optimizations

These optimizations are approved for implementation to eliminate memory copy overhead and hash map lookup bottlenecks.

### **A. Convert `PriorityPartialPath` to Use Pointers**
*   **Location:** `sources/Optimization/OptimizeSP_Incre.h` (struct `PriorityPartialPath`)
*   **Bottleneck:** Currently, `PriorityPartialPath` stores `DAG_Model dag_tasks` and `SP_Parameters sp_parameters` by value. Audsley's search loop (`OptimizeFromScratch`) creates and copies thousands of partial paths:
    ```cpp
    PriorityPartialPath new_path = path; // Triggers full deep copy of task graph and parameters
    ```
    This wastes CPU cycles on heap allocations and copying unchanged constants.
*   **Optimization:** Convert the fields to `const DAG_Model*` and `const SP_Parameters*` pointers. This reduces the copy footprint to a few bytes and makes copy construction virtually free.

### **B. Cache Shared Pointers in Cache Map**
*   **Location:** `sources/Optimization/OptimizeSP_TL_Incre.h` (member `timelimit2optimizer_`)
*   **Bottleneck:** The map `timelimit2optimizer_` caches optimizer states `OptimizePA_Incre` by value. Inserting or retrieving from the map triggers deep-copies of the optimizer, including the nested task graphs.
*   **Optimization:** Change the map definition to store `std::shared_ptr<OptimizePA_Incre>`:
    ```cpp
    std::unordered_map<std::vector<double>, std::shared_ptr<OptimizePA_Incre>, HashKey4Vector> timelimit2optimizer_;
    ```
    This eliminates copying during map insertions and lookups.

### **C. Flat Vector Sort-Coalesce for Convolution**
*   **Location:** `sources/Safety_Performance_Metric/Probability.cpp` (method `FiniteDist::Convolve`)
*   **Bottleneck:** The nested loop of size $O(N \cdot M)$ performs dynamic hash map lookups on `std::unordered_map<double, double> m_v2p` (`count` and `operator[]`). This causes significant CPU overhead due to hashing, bucket lookups, pointer chasing, and dynamic node allocations on the heap.
*   **Optimization:** Convolve directly into a flat pre-allocated `std::vector<Value_Proba>`, sort it, and then coalesce adjacent elements using a single-pass scan. 
*   **Benefits:** 
    1.  **Cache Locality:** contiguous vector memory access guarantees high cache hits.
    2.  **No Malloc Overhead:** One contiguous pre-allocation via `reserve` instead of allocations for each hash node.
    3.  **Floating-point Safety:** Handles double equality cleanly via epsilon tolerance (`std::abs(a - b) < 1e-9`) instead of strict binary match.
*   **Code Implementation:**
    ```cpp
    void FiniteDist::Convolve(const FiniteDist& other) {
        if (distribution.empty() || other.distribution.empty()) {
            distribution.clear();
            UpdateMinMaxValues();
            return;
        }

        std::vector<Value_Proba> convolved_vec;
        convolved_vec.reserve(distribution.size() * other.distribution.size());

        for (const auto& element_this : distribution) {
            for (const auto& element_other : other.distribution) {
                double value = element_this.value + element_other.value;
                double prob = element_this.probability * element_other.probability;
                convolved_vec.emplace_back(value, prob);
            }
        }

        std::sort(convolved_vec.begin(), convolved_vec.end(), 
                  [](const Value_Proba& a, const Value_Proba& b) {
                      return a.value < b.value;
                  });

        std::vector<Value_Proba> coalesced_vec;
        coalesced_vec.reserve(convolved_vec.size());

        for (const auto& item : convolved_vec) {
            if (!coalesced_vec.empty() && std::abs(coalesced_vec.back().value - item.value) < 1e-9) {
                coalesced_vec.back().probability += item.probability;
            } else {
                coalesced_vec.push_back(item);
            }
        }

        distribution = std::move(coalesced_vec);
        UpdateMinMaxValues();
    }
    ```

---

## 2. On Hold / For Further Evaluation (Algorithmic Optimizations)

The following proposed algorithmic optimization is placed on hold for correctness evaluation, as the dynamic preemption iteration in response time analysis requires careful verification.

### **A. Incremental High-Priority Task Convolution ($O(N^2) \to O(N)$ Convolutions)**
*   **Location:** `sources/Safety_Performance_Metric/RTA.cpp` (method `ProbabilisticRTA_TaskSet_SingleCore`)
*   **Bottleneck:** For task `i`, the algorithm calls `GetRTA_OneTask(tasks[i], hp_tasks)`, which performs $i$ convolutions of all higher-priority task execution times from scratch.
*   **Optimization Proposal:** Maintain a running convolved distribution `hp_conv` of all higher priority tasks as we iterate through the taskset, and convolve task `i` with `hp_conv` once. 
*   **Status:** **ON HOLD** (Needs verification to ensure dynamic preemptions and deadlines are evaluated correctly under merged convolutions).
