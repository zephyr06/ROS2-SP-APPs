# P0.4 Project Evaluation Suite — Dev Log
> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-06
- **Filed as an active P0 task (not started).** Per user request: add a
  project evaluation suite as a north-star integration test — runs on N=4/6/8
  simulated tasksets, measures optimization quality (avg SP) + scheduler ET
  overhead, checks against the red-flag lines in
  `agents/project_evaluation_northstar.md`. Intended as the single tuning
  target for any code/algorithm change: bounded ~20 min runtime is acceptable
  because it is an integration test, not a unit test. Scope deliberately
  captured in `goal.md` with the open questions (N-scope vs. E1, exact baseline
  set for Q3, seed granularity) flagged as **decide-first when the task is
  picked up**, not resolved now. Shares infrastructure with P0.3 (same e2e
  pipeline + metric collection) but adds the gate-checking layer. User
  instruction: "don't start working on it, just add this as an active task" —
  no implementation done.
