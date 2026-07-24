# P1.22 — Dev Log

## 2026-07-22 — Task Creation & Initial Hypotheses

### Context
In commit `4d7d14b6bc0f70ed2123b20e1efad1b60369f287` (P1.21), an `RTACache::Transaction` mechanism using Lazy Copy-on-Write (CoW) was introduced to abstract speculative cache backups during sub-incremental optimization walks (`EvaluateTimeLimitConfig_SubIncremental`).

However, profiling measurements indicated that the transaction abstraction resulted in a slight performance slowdown compared to the prior eager stack-copy approach.

### Summary of Hypotheses

1. **High Frequency of Mid-Walk `AdoptChampion` Calls**:
   `OptimizeIncre_SingleTask` searches 1D priority variations within each sub-incremental step and calls `AdoptChampion` whenever a local priority variation beats `opt_sp_`. Because local 1D priority improvements are frequent, `AdoptChampion` is called in almost every step, causing the lazy snapshot to fire ~90–95% of the time anyway.
2. **Dynamic Heap Allocation Churn**:
   `std::make_unique<ChampionState>` allocates memory on the heap via `malloc`/`operator new` per snapshot, and deallocates via `free` on scope exit. Doing this thousands of times per second adds allocator latency compared to stack or reusable memory buffers.
3. **Redundant Mid-Walk Prefix Rebuilding**:
   Each intermediate `AdoptChampion` call inside `OptimizeIncre_SingleTask` re-convolves execution times (`RebuildPrefixes`). For outer steps that are ultimately rejected by `UpdateRecords`, these intermediate bakes are wasted work.
4. **Pointer Indirection & Branch Overhead**:
   Checking `in_transaction_` and pointer dereference via `snapshot_` in hot evaluation loops.

### Next Steps
- Implement zero-heap allocation buffer reuse for `ChampionState`.
- Measure execution time improvements against the baseline.

## 2026-07-23 — Phase 1a Measurement: Transaction/AdoptChampion frequencies

### Method
Temporary instrumentation (NOT committed — guarded for removal) added to
`RTA_Cache.cpp`: a file-scope `TransactionCounters` struct + `atexit` stderr
dump (`====== P1.22 TRANSACTION COUNTERS ======` block) emitted once per
RunOrchestrator process = once per (taskset × scheduler). Counters:
`transactions_opened` (BeginTransaction), `committed` (CommitTransaction =
accept), `rolled_back` (RollbackTransaction = reject), `snapshots_captured`
(lazy capture fired = the tx had ≥1 mutation → paid the deep `ChampionState`
copy), `snapshots_null_on_rollback` (zero-copy reject: no mutation fired),
`adopt_champion_total`/`adopt_champion_in_tx`, `initialize_total`/`in_tx`.

Config `simulation_experiments/configs/p1_22_counter_probe.json`: INCR_Reopt_10
ONLY (BF/RM/CFS open no tx → noise), N={10,16}, 3 tasksets each, 300s @ 10s
interval (~30 scheduling triggers/run), `parallel_worker_processes=1` (clean
1:1 process→run.log mapping), `RERUN_MODE=clear_all`, seed 1000. Built in
`build_test` (DEBUG; fine for a frequency count, NOT a timing bench).

### Raw per-process table
| N  | ts | opened | commit | rollback | snaps | null@rb | adoptT | adopt@tx | init |
|----|----|--------|--------|----------|-------|---------|--------|----------|------|
| 10 | 0  | 314    | 27     | 287      | 133   | 181     | 289    | 262      | 27   |
| 10 | 1  | 108    | 0      | 108      | 29    | 79      | 56     | 29       | 27   |
| 10 | 2  | 213    | 22     | 191      | 79    | 134     | 156    | 129      | 27   |
| 16 | 0  | 429    | 6      | 423      | 140   | 289     | 310    | 283      | 27   |
| 16 | 1  | 278    | 40     | 238      | 118   | 160     | 250    | 223      | 27   |
| 16 | 2  | 290    | 35     | 255      | 161   | 129     | 445    | 418      | 27   |

`Initialize` total=27 every run = 1 per interval (27 intervals in 300s@10s ≈
matches); `Initialize in_tx=0` confirms the Initialize path never fires inside
a tx (the tx body calls `OptimizeIncre_SingleTask` → Evaluate+AdoptChampion
only, NOT `OptimizeIncre` where Initialize lives; defensive coverage only —
matches the P1.21 Phase 4 trace).

### Aggregate (3 tasksets each)
| metric                          | N=10      | N=16      | GRAND (6 proc) |
|---------------------------------|-----------|-----------|----------------|
| transactions_opened             | 635       | 997       | 1632           |
| committed (accept)              | 49 (7.7%) | 81 (8.1%) | 130 (8.0%)     |
| rolled_back (reject)            | 586 (92.3%)| 916 (91.9%)| 1502 (92.0%) |
| snapshots_captured (paid copy)  | 241 (38.0%)| 419 (42.0%)| 660 (40.4%)  |
| zero-copy reject (no mutation)  | 394 (62.0%)| 578 (58.0%)| 972 (59.6%)  |
| AdoptChampion total             | 501       | 1005      | 1506           |
| AdoptChampion inside tx         | 420 (83.8%)| 924 (91.9%)| 1344 (89.2%)|
| mean in-tx adopts / opened tx   | 0.66      | 0.93      | 0.82           |

### Headline — direct answer to "how often does AdoptChampion fire"
A transaction is opened on **every** `EvaluateTimeLimitConfig_SubIncremental`
trial. Of opened tx: **92% reject / 8% accept**. **~60% of tx never mutate**
(zero in-walk AdoptChampion → zero-copy reject, the lazy-COW win); **~40%
mutate** (≥1 in-tx AdoptChampion → pay the deep `ChampionState` copy on
capture). Of all AdoptChampion calls, **89% fire inside a tx** (the rest are
accept-path `CommitIncumbent` adopts, outside tx scope); a mutating tx adopts
~once (mean 0.82/opened tx; N=16: 0.93). So hypothesis 1's "AdoptChampion
fires ~90–95% of the time" is **over-estimated — actual snapshot capture ≈40%**.

### Findings vs the 4 hypotheses
1. **High-frequency mid-walk AdoptChampion (premise: snapshots fire ~90–95% of
   the time)** — **PARTIALLY CONFIRMED, weaker than feared.** A mutation fires
   in **40%** of opened tx (N=10: 38%, N=16: 42%), NOT 90–95%. So the lazy
   snapshot captures on 40% of tx, and the zero-copy path holds on the
   remaining ~60% (all of them rejects: a reject-without-adopt is the dominant
   case at 60% of all tx). The lazy design IS buying a real win — 60% of tx pay
   zero copy vs the eager P1.16 approach which paid 100%. BUT: of the AdoptChampion
   calls, **89% fire inside a tx** (N=16: 92%), and a tx that adopts almost
   always captures (accept=8% + reject-with-adopt). mean in-tx adopts/opened tx
   = 0.82 (N=16: 0.93) → most tx that DO mutate adopt ~once. So the
   "AdoptChampion fires in almost every sub-incremental step" framing is too
   strong; the truth is "≈40% of tx mutate, and those that do adopt ~once".
2. **Heap churn (`make_unique<ChampionState>`)** — the snapshot capture fires
   **660 times** across 6 processes (≈110/run). Each is one `make_unique` +
   one `unique_ptr::reset` (free) + the `ChampionState` deep copy of 5 members
   (incl. `vector<FiniteDist> rta_` — the heavy one, N FiniteDists). 660
   heap allocs across a ~360s run is NOT high-frequency allocator churn — it is
   a non-issue at this rate. The deep `vector` copy on capture/restore is the
   real cost, not `malloc`. → **Hypothesis 2 (heap) is NOT the dominant cost.**
3. **Redundant mid-walk bakes (RebuildPrefixes)** — AdoptChampion in-tx = 1344.
   Each AdoptChampion → BakeChampionForms → RebuildPrefixes (re-rolls all
   per-core ET convolutions). For the 92% of tx that roll back, every in-tx
   adopt's RebuildPrefixes is discarded. This is the larger wasted-work axis,
   not the heap alloc. → **Hypothesis 3 (redundant bake) is the more live
   concern**, gated on whether the in-tx adopts' bakes are actually avoidable.
4. **Pointer indirection** — negligible (one branch + one null check per
   AdoptChampion/Initialize; the hot Evaluate path has none).

### Reconciling the 40% / 8% / 92% (2026-07-23, post-review)
A reader flagged that 8% commit vs 42% snapshot-capture (N=16) "don't match."
They DO — the mismatch is in *when* the deep copy fires. **Capture fires on
the first MUTATION (in-walk AdoptChampion), NOT on commit.** Verified in
`RTA_Cache.cpp:325` (`SnapshotPreMutationStateIfOpen`, first-capture-only,
called from `AdoptChampion:275` + `Initialize:257`); `CommitTransaction:360`
just `snapshot_.reset()` (no copy); `RollbackTransaction:370` restores if a
snapshot exists. So capture ∝ mutations, not ∝ commits.

Two distinct "better?" bars:
1. **In-walk AdoptChampion → capture (the 40%)**: a 1D priority *variation*
   beat the LOCAL running `opt_sp_` (`OptimizeSP_Incre.cpp:348`). Speculative,
   advances the champion WITHIN the tx. Weak bar, frequent.
2. **Commit (the 8%)**: `UpdateRecords` (`:308`) found the trial config beat
   the GLOBAL incumbent `res_opt_`. Strong bar, rare.

40% > 8% is expected. Arithmetic closes EXACTLY:
`snapshots_captured(660) = committed(130) + rolled_back_with_snapshot(530)`,
where `rolled_back_with_snapshot = rolled_back(1502) − zero_copy_reject(972)
= 530`. → 130+530=660. Every committed tx had captured a snapshot; every
snapshot either committed or was restored. The 89%/92% "AdoptChampion in-tx"
is a different (per-call, not per-tx) denominator — share of all AdoptChampion
calls that fired inside an open tx.

**Sharper cost finding:** of 660 captures, 530 (80.3%) were thrown away by
rollback (paid capture AND restore, discarded); only 130 (19.7%) were kept.
So the lazy-COW saves the copy on the 60% zero-copy path, but on the 40% that
mutate it pays capture on 100% and an extra restore on 80% — 80% of that work
serves no kept result. Points at capture+restore of `vector<FiniteDist> rta_`
on rolled-back tx (candidate (a)) and/or redundant `RebuildPrefixes` on the
1344 in-tx adopts (hypothesis 3) — NOT heap churn (hypothesis 2, ruled out)
and NOT "snapshots fire too often" (they fire at the designed ~40% rate).

### Verdict
- The lazy-COW Transaction's core premise **holds**: 60% of opened tx (the
  reject-without-adopt majority) pay ZERO copy — a real win over P1.16's eager
  100%-copy. The "90–95% snapshot fire" fear in hypothesis 1 was an over-
  estimate; actual capture rate is **~40%**.
- The slowdown P1.22 was filed to chase is therefore NOT explained by "lazy
  snapshots fire almost always" (they don't). Candidates ranked by likelihood:
  (a) the **deep `ChampionState` copy itself** on the 40% that capture
  (`vector<FiniteDist> rta_` of N dists, copied on capture AND move-restored on
  rollback) — 660 captures + 660 restores of an N-sized vector;
  (b) **redundant `RebuildPrefixes`** on the 1344 in-tx adopts that get rolled
  back (hypothesis 3);
  (c) heap alloc is the LEAST likely (660 allocs in 360s is trivial).

### Next
Before any optimization, need a TIMING measurement (release build) to confirm
WHERE the regression actually is — the frequency data above rules out heap churn
but does not by itself locate the cost. Phase 1b (timing) next, or re-examine
the original "+5.02%@N=10" P1.22 reading (which the P1.23 memory notes was a
timer-scoping artifact, NOT a real regression — see [[p123-ab-rta-cache-speedup]]:
the old timer bracketed all of RunSimulation, washing the optimizer delta in
RTDA+I/O noise). **Open question for user: is P1.22 still live given P1.23's
finding that the apparent transaction regression was a timer artifact?**

### Cleanup
- Instrumentation in `RTA_Cache.cpp` is TEMPORARY (uncommitted); revert before
  any commit. The probe config `p1_22_counter_probe.json` is throwaway.
- Run artifacts under
  `simulation_experiments/optimizer_comparison/runs/p1_22_counter_run_test_dur300_interval10_seed1000_tasks10x16/`.
