# §10 — Complexity Analysis

> Draft: `section10_complexity.tex` (41 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions. Label: `section_complexity_analysis`.

## High-level change

§10 is the complexity section the §8 `\Sen` note defers to. Its numbers are
broadly correct vs code but need three reconciliations: (1) confirm BF is
exponential (omitted, correct); (2) the modified-Audsley complexity is stated as
`O(m × N²)` RTA calls (`:20`) — verify `m` = beam width `K` (=2) and that this
matches `OptimizeFromScratch`; (3) the incremental complexity `O(N × O_RTA)`
(`:27`) **depends on the RTA cache's single-change invariant** — state that
dependency explicitly (the cache throws on `|diff|>1`, `RTA_Cache.cpp:358`), and
add the RTA-cache speedup (~36% faster, bit-identical SP, per `sketch_optimization.md`).
§8's 1-task-ET-diff property is stated (not proven) — §10 should cite the same
invariant, not a theorem.

## Action summary

- **VERIFY** BF exponential framing (`:6`) — correct (BF omitted from detailed
  analysis). Optional: note BF is the offline optimality reference (Ryan 2.2
  ablation) if PW.4 adds it.
- **VERIFY** modified-Audsley `O(m × N²)` (`:20`): confirm `m` = beam width
  `K=2` (`Layer_Node_During_Incremental_Optimization`, `parameters.yaml:7`) and
  matches `OptimizeFromScratch` (`OptimizeSP_Incre.cpp:100-164`). State `m`'s
  meaning explicitly (Ryan 1.2 rigor).
- **VERIFY + ADD dependency** incremental `O(N × O_RTA)` (`:27`): correct, BUT
  state it **depends on the RTA cache's single-change invariant** (every
  candidate differs from the champion by ≤1 task → only a suffix of the RTA chain
  is recomputed). Cite the invariant the cache enforces (`RTA_Cache.cpp:358`
  throws on `|diff|>1`), NOT a formal theorem (PW.1.2 recommendation, mirrors §8
  complexity row).
- **ADD** the RTA cache speedup: ~36% faster at N=10, SP bit-identical (cache
  avoids recomputation, never approximates). Sourced from `sketch_foundations.md`
  §3 / `sketch_optimization.md`.
- **VERIFY** the `O(1)` practical-RTA claim (`:27`) via resampling/PMF-bounding —
  keep if defensible, soften if it overclaims (Ryan 1.4 honest calibration).

## Ryan Cat-1

- (indirect) 1.2 rigor + 1.4 honest claim calibration — complexity claims must be
  stated as propositions with their preconditions, not bare results.

## Open notes

Row-by-row detailing deferred. The one non-trivial item: make the
`O(N × O_RTA)` claim's dependence on the single-change invariant explicit (ties
§10 to §8's complexity row + the §6.x guarantee). Flag for PW.3.
