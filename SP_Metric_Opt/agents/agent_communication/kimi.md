## P1.22 — Capture vs commit, and the lazy-copy framing (2026-07-23)

**Question raised:** the snapshot captures on ~40% of opened transactions but only ~8% commit. Why copy on mutation rather than only before commit? Isn't that backwards?

**Correction first — the current design IS lazy copy, by definition.** Copy-on-write / lazy copy means: do NOT copy at the logical backup point; defer the copy until a write (mutation) actually happens, because only then do you need a private copy to keep the original intact. The P1.21 `SnapshotPreMutationStateIfOpen` does exactly this:
- `BeginTransaction` = the logical "I may want to undo this scope" point. The eager P1.16 design deep-copied the whole cache HERE → 100% of transactions paid.
- Lazy design: no copy at `BeginTransaction`. The copy fires only on the FIRST mutation (`AdoptChampion`/`Initialize`), first-capture-only.
- 60% of transactions never mutate → never copy → zero cost. That 60% skip is the laziness paying off.
- 40% mutate → copy fires, correctly, because the original is about to be overwritten and rollback needs it.

So the three positions, correctly labeled:
1. **Eager** (P1.16): copy at BeginTransaction, 100% of tx. Not lazy.
2. **Lazy COW** (P1.21, current): copy on first mutation, 40% of tx. ← this IS lazy copy.
3. **"Copy before commit"** (the original intuition): ill-defined for rollback. At commit time the pre-transaction original is already destroyed by the in-walk mutations, so a copy made at commit cannot restore the original on rollback. If the intuition is really "don't touch the shared cache speculatively; write only at commit" — that's deferring the WRITE (a scratch-buffer restructure), not deferring the copy. Different axis.

**Capture vs commit, plainly:**
- Capture = save the pre-mutation original into `snapshot_` so rollback can restore it. Must precede the overwrite.
- Commit = keep the mutated result; just `snapshot_.reset()`, no copy.
- Rollback = `champion_ = move(*snapshot_)`; needs the pre-mutation capture.

**The real tension the 80%-wasted finding exposes** is NOT that lazy copy is wrong (it correctly skips the 60%). It's that COW pays a copy on EVERY write regardless of whether the transaction ultimately commits: of the 40% of tx that write (and thus copy), ~80% roll back, so that copy+restore served no kept result. This is inherent to copy-on-write-with-rollback — the copy must precede the write because at write-time you cannot know whether the tx will commit, so speculative writes that later roll back always pay the copy. Avoiding THAT requires not mutating the shared cache speculatively at all (scratch champion, swap-in at commit) — which is exactly the hypothesis-3 / candidate-(a) restructure, gated on whether in-walk `AdoptChampion`→`BakeChampionForms`→`RebuildPrefixes` is load-bearing for the next `Evaluate` in the same walk.

**Net:** the current capture-on-first-mutation is the laziest CORRECT design for in-place mutation — not a bug, not a mis-timed copy. The P1.22 cost question reduces to "can the in-walk search run against scratch so commit is the only write," not "is the lazy copy wrong." Pending: A/B/C decision (is P1.22 still live given P1.23's finding that the apparent slowdown was a timer-scoping artifact?).

## P1.25 — Resolution: remove the transaction layer (2026-07-23)

**Outcome:** the capture-vs-commit / lazy-copy debate above is now MOOT.
User directive: *"i'm tired of arguing on the transaction. let's add a new
active task p1_25 to remove all the rta cache transaction,etc, only keep rta
cache."* New task [`active_tasks/P1_25_remove_rta_cache_transaction/`] filed.
The transaction layer (P1.21 lazy CoW) will be deleted; the `RTACache` itself
stays.

**Why removal is justified by the evidence already gathered:**
- P1.22 Phase 1a: of 1632 opened tx, only 8% commit; ~40% capture a snapshot;
  **80.3% of those captures (530/660) are thrown away by rollback** — paid
  capture AND restore, discarded. The lazy design wins on the 60% zero-copy
  path but, on the 40% that mutate, pays capture on 100% and an extra restore
  on 80%.
- P1.23: the original "+5.02%@N=10 transaction slowdown" P1.22 was filed to
  chase was likely a timer-scoping artifact (old timer bracketed all of
  `RunSimulation`, washing the optimizer delta in RTDA+I/O noise); under the
  corrected timer the cache arm was ~36% *faster*, not slower. So the
  transaction's *premise of a regression to fix* is in doubt.

**The one correctness constraint the discussion did NOT settle (and P1.25
must):** the transaction is the ONLY mechanism that reverts the cache
champion on a REJECTED sub-incremental walk step. The ACCEPT path is already
covered without it — `CommitIncumbent` (`OptimizeSP_TL_Incre.cpp:807-810`)
re-adopts the committed triple via `Evaluate`+`AdoptChampion` on every commit.
So deleting the transaction + adding nothing ⇒ a rejected walk leaves the
champion advanced to a trial PA that `res_opt_` never committed ⇒ the next
`Evaluate` sees `|diff|>1` ⇒ `ComputeTaskSetDifference` THROWS (the
P1.15/P1.16 crash class returns). P1.25 Decision D1 picks the replacement
revert: (a) re-seed champion from `res_opt_` at sub-incremental entry
[recommended], (b) eager save/restore on the reject branch, or (c) stop
speculative in-walk adopts entirely.

**D1 DECIDED = (b) eager save/restore scoped to the reject branch** (2026-07-23).
User: *"i didn't read your question carefully, just take references from the
previous commit, like ecbed89697016ee429ef8c9b72b52e46463cb0da, unless you have
better ideas to discuss."* `ecbed896` is the pre-transaction parent of `4d7d14b6`
("add rta cache transaction"); it had the cache but NO tx, and its reject path
was exactly `RTACache cache_backup = rta_cache_;` at entry + `if (!updated)
{ rta_cache_ = cache_backup; }` on reject — proven-correct by git history.
Overrides the earlier (a) re-seed-at-entry recommendation. Trade-off: (b)
re-introduces the per-step deep copy P1.21 removed, but the tx's perf premise
was a timer artifact (P1.23) and (b) is known-good vs (a)'s fresh correctness
check. (a) remains the deferred upgrade path.

**Task status:** P1.25 filed, D1=(b) decided, implementation TDD-first
(beginning Phase 0 cleanup). P1.22 to be closed as superseded-by-P1.25; P1.21
to move to `finished_tasks/` once P1.25 lands. The P1.22 temporary
`TransactionCounters` instrumentation in `RTA_Cache.cpp` + the throwaway
`p1_22_counter_probe.json` revert as P1.25 cleanup steps 0b/0c.
