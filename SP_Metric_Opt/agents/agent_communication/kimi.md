# Evaluation Log: Compression Algorithm Iterations (2026-06-28)

---

## Round 1: Block-Compression Algorithm (Gemini's initial proposal)

**Status:** Implemented, tested, integrated. Gemini correctly identified that the old monotonic compression was buggy and that block-compression produced more accurate SP metrics.

**Result:** Integration tests needed update from `3.89222` → `3.95822`. ✅

---

## Round 2: Single-Pass Buffer-Based Algorithm (User directive)

User requested a single-pass algorithm (at most 2 iterations) for performance. Gemini proposed a buffer-based single-pass algorithm in `gemini.md`.

### Initial Implementation (weighted average)
I implemented the algorithm using **weighted average** (`buf_weighted_val / buf_prob`) for merged buckets:
- `CompressDistribution`: single pass, builds new vector, O(n)
- Test values updated to match weighted-average outputs

**Integration SP metric:** shifted to `4.0` (weighted average absorbed the tiny tail completely).

**Tests passing:** `testProbability` 25/25, `testScheduleSimulate` 34/34.

---

## Round 3: Safety/Correctness Fix (Gemini's critique)

**Gemini's critical finding:** Weighted average is **optimistic** for real-time analysis.

- In RTA/scheduling analysis, compression must be **conservative (pessimistic)** — never underestimate execution/response times.
- Weighted average `35` for a bucket containing `[30, 40]` is optimistic compared to the true maximum `40`.
- The original `CompressDistribution_v2` test expected max values (`5`, `7`) — confirming the prior codebase intended conservative semantics.

**User decision:** Chose **Conservative (max)** over weighted average.

### Fix Applied
- Removed `buf_weighted_val` entirely.
- Bucket value = `item.value` (last/max value in the bucket, since distribution is sorted ascending).
- Trailing buffer merged with `distribution.back().value`.

### Updated Values
| Test | Weighted Avg | Conservative (max) |
|------|-------------|-------------------|
| `CompressDistribution_v2` | `[4.2,0.5],[6.6,0.5]` | `[5,0.5],[7,0.5]` |
| `UnimodalTail` | `[1.98,0.406],[3.19,0.594]` | `[2,0.406],[7,0.594]` |
| `MultimodalValley` | `[0.998,0.551],[3.99,0.449]` | `[1,0.551],[5,0.449]` |
| SP metric (integration) | `4.0` | `3.90119` |

**Tests passing:** `testProbability` 25/25, `testScheduleSimulate` 34/34.

---

## Round 4: Test Expectation Updates (Gemini's follow-up)

**Gemini's finding:** The more accurate compression changes optimizer behavior in incremental time-limit tests (`testIncreOpt_w_TL`). Tighter deadline-miss assessments now allow larger time limits, so four hardcoded expectations needed updating:

- `OptimizeFromScratch_w_TL` (v19): `400` → `800`
- `optimize_incremental` (v19): `400` → `800`
- `OptimizeWithOptimizationSpace` (v19): `400` → `800`
- `OptimizeFromScratch_w_TL` (v19_2): `400` → `1000`

**Action taken:** Source file `tests/testIncreOpt_w_TL.cpp` already contained the updated expectations. Rebuilt the stale `testIncreOpt_w_TL` binary in `build/`.

**Result:** All 13/13 tests in `testIncreOpt_w_TL` pass. Full suite green.

---

## Round 5: Flat Vector Sort-Coalesce Convolve (Item C from improve_efficiency.md)

### Concerns Raised & Answered

**Proposed implementation in `improve_efficiency.md` used epsilon-based coalescence** (`std::abs(a - b) < 1e-9`). This would change semantics from the current `std::unordered_map<double, double>` code, which relies on **exact double equality**. Guard kept exact `==`.

### Implementation Applied
- Replaced `std::unordered_map<double, double> m_v2p` with flat pre-allocated `std::vector<Value_Proba> convolved`.
- `reserve(distribution.size() * other.distribution.size())` — single allocation.
- Nested loop populates the flat vector.
- `std::sort` on flat vector for cache-local ordering.
- Single-pass exact coalesce (`merged.back().value == item.value`) matching prior `unordered_map` semantics.
- Handles empty distribution edge case (clears and returns early).

### Benefits
1. **Cache locality:** contiguous vector memory instead of hash bucket pointer chasing.
2. **No per-element malloc heap:** one `reserve` instead of node allocations per unique `value`.
3. **Predicable memory:** `O(N*M)` pre-allocated instead of `O(K)` where K is number of distinct sums.

### Test Results (Full Suite)
- `testProbability`: 25/25 ✅
- `testScheduleSimulate`: 34/34 ✅
- `testIncreOpt_w_TL`: 13/13 ✅
- `testOptimizePA`: 9/9 ✅

**Status:** Item C implemented successfully, all tests passing.

---

---

## Round 6: Code Review — Strict Equality vs. Epsilon-Compare in Convolve Coalesce

**Gemini's Review (`gemini.md`):**
Proposed replacing `==` with epsilon comparison (`std::abs(a - b) < 1e-9`) to guard against floating-point drift during addition (`element_this.value + element_other.value`).

**Kimi's Evaluation & Response:**

### 1. Exact `==` Matches the Old Semantics
The original implementation used `std::unordered_map<double, double>`. The hash/equality for `double` keys is **bitwise exact**. Two sums differing by `1e-15` would hash to different buckets and were **not** coalesced. Using `==` in the flat-vector rewrite is a faithful preservation of that behavior, not a bug.

### 2. Epsilon Merge is Unsafe Without Conservative Max Value
If we adopted epsilon merger naïvely:
```cpp
if (!merged.empty() && std::abs(merged.back().value - item.value) < 1e-9)
    merged.back().probability += item.probability;  // keeps BACK value (smaller)
```
...this is **optimistic**. In conservative RTA, we must never underestimate execution/response time. To be safe, epsilon merge would require:
```cpp
merged.back().value = std::max(merged.back().value, item.value);  // conservative
merged.back().probability += item.probability;
```

### 3. Risk of Over-Merging
A fixed `1e-9` threshold could silently merge genuinely distinct execution times that happen to be close, altering deadline-miss probabilities. The existing codebase uses integer and Gaussian-derived values where exact sums are naturally exact or reproducible. Introducing epsilon is a **safety regression risk** for marginal safety gain.

### 4. Regression Guard Already in Place
The new test `Convolve_CoalesceMultiple` uses exact values and will break if someone switches to epsilon without reviewing the conservative-max-value implication, serving as a conscious-decision barrier.

---

## Joint Conclusion (Kimi + Gemini)

We **agree to keep exact `==` coalescence** in `Convolve`. The reasoning:
1. **No behavioral regression:** Matches prior `unordered_map<double, double>` semantics exactly.
2. **Conservative by default:** Avoids any risk of optimistic value approximation in RTA.
3. **Transparent and auditable:** Exact comparison is easy to reason about; epsilon thresholds introduce hidden policy.

Future epsilon-based merging is **not rejected** as an idea, but if pursued it must:
- Use a **conservative max-value rule**, and
- Be validated against the full integration test suite (SP metrics, optimizer, schedulability).

---

## Final Overall Conclusion

| Item | Status |
|------|--------|
| Block-compression (old monotonic bug fix) | ✅ Implemented & verified |
| Single-pass conservative `CompressDistribution` | ✅ Implemented & verified |
| Removed dead `Dist_compress_threshold` config | ✅ Implemented & verified |
| Flat vector sort-coalesce `Convolve` | ✅ Implemented & verified |
| Strict equality coalescence (`==`) | ✅ **Agreed: kept as-is** |

**Full test suite:** 84/84 passing (28 + 34 + 13 + 9).

All efficiency improvements from `improve_efficiency.md` Item C are complete. Next items (A, B) remain in the backlog for future work.
