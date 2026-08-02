# §12 — Real Experiment Setup

> Draft: `section12_real_exp.tex` (154 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions.

## High-level change

§12 is the real-experiment setup (tasks, deadlines, thresholds, baselines, ET
box-plots). The dominant revision is the **Θ-table inversion** (content change 5 /
Ryan 1.2): the table (`:29-32`) gives the most safety-critical task MPC the
*largest* Θ (0.99) and SLAM 0.9 — the *reverse* of Option A (`Pr(r_i>D_i)≤Θ_i`,
small Θ = stricter/more critical). This is the single experiment-side decision
that may force a re-run. Secondary: align the "deadline miss threshold reflects
importance" framing (`:13`) with Option A; PW.2 structural merge (§12+§13);
headline-number consolidation; and (optional) task-level ground-truth reporting
(Ryan 2.1).

## Action summary

- **FIX / OPEN DECISION (Θ table inversion, Ryan 1.2):** the table (`:29-32`)
  assigns MPC=0.99, SLAM=0.9, RRT=0.5 — but under Option A (code-verified,
  `GetDDL_MissProbability` returns `Pr(R>D)`; gate rejects on
  `miss > threshold`), Θ_i = *max tolerable miss prob*, so a more critical task
  gets a *smaller* Θ. MPC=0.99 would mean "tolerate MPC missing 99% of the time"
  — the least-protected task, opposite of stated intent (`:13` "higher thresholds
  reflect importance"). **Two paths:** (A) keep Option A everywhere and **invert
  the table** so critical tasks get small values (MPC≈0.01, SLAM≈0.1) — changes
  input numbers → **re-run required**; (B) redefine Θ_i = required success prob
  (`Pr(r_i≤D_i)≥Θ_i`, table values stay) — text-only IF the code already treats Θ
  that way (it does NOT — code is Option A). → **Path A is the code-consistent
  choice; it requires a re-run.** PW.4 owns the check + re-run. Flag at §6.3 too.
- **FIX** the "higher deadline-miss thresholds reflect importance" framing (`:13`):
  under Option A it's the reverse — lower Θ = more critical. Rewrite to match
  whichever convention is adopted.
- **MERGE** §12+§13 per PW.2's `reorg_plan.md` (structural — reference only, don't
  re-decide here).
- **CONSOLIDATE** the headline number (Ryan 1.3): currently quoted as 15–50% /
  20–50% / 20–40% across §12/§13/§15/§16. Pick one backed by a results table.
  (Touches §13, §15, and skipped §16 — note here so it's not dropped.)
- **ADD (optional, Ryan 2.1)** task-level ground truth already logged: per-task
  miss rates, RT CDFs, SLAM error, path length, MPC tracking error, scheduler
  overhead — answers the "evaluating only your own SP metric is circular"
  concern. Low effort if logs exist.

## Ryan Cat-1

- 1.2 (Θ-table contradiction — resolve via Option A; re-run if inverting).
- 1.3 (headline number consolidation — cross-section).
- 2.1 (optional task-level ground truth).

## Open notes

Row-by-row detailing deferred. The Θ-table inversion is the gating decision: it
determines whether a re-run is needed and ripples to §13.3's guarantee wording +
§6.3's Option-A consistency. Decide the convention first (PW.4), then detail rows.
