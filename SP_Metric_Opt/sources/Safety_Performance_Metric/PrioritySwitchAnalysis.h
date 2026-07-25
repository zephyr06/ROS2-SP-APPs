#pragma once
// Pure priority-analysis utilities for the single-champion RTA cache.
//
// These helpers factor the "is the candidate's per-core priority order a single
// task's relocation away from the champion's?" question out of
// RTACache::IsSingleTaskChange (RTA_Cache.cpp). They depend only on
// <vector>/<unordered_map>/int — no SP/RTA/DAG types — so this is a true leaf
// header (no header cycle, unlike RTA_Cache.h which documents one). Lives in a
// dedicated header so the test suite can target the two-pointer walk +
// remove-and-compare edge cases directly with hand-built vector<int> inputs,
// rather than only indirectly through the DAG/PA/TL pipeline.

#include <vector>

namespace SP_OPT_PA {

// "Remove one task from both vectors, compare the rest": erase `task_id` (which
// appears exactly once in each — task ids are unique per core) from both the
// candidate and champion per-core orders, preserving order, then return whether
// the two shortened vectors are equal. If they are, `task_id`'s relocation (or
// ET change, when it's the known ET-diff task) is the SINGLE change and
// everything else is identical; if not, a second task also moved (=> not
// single).
//
// Two-pointer walk: advance i/j over candidate/champion, skipping `task_id` in
// either vector, and compare the un-skipped entries pairwise. Equal iff the two
// orders match after removing `task_id` from both — without materializing the
// shortened vectors. Caller guarantees the two orders are the SAME size (a size
// mismatch is a core migration, rejected before this is called); `task_id`
// appears exactly once in each (the changed task), so the two un-skipped
// sequences have equal length.
bool RestEqualAfterRemoving(const std::vector<int>& candidate_order,
                            const std::vector<int>& champion_order,
                            int task_id);

// Locate which core `task_id` sits on in `per_core` (the processorId → task-id
// order vector, indexed by core), or -1 if absent. Used to place the ET-changed
// task's core. Takes a flat vector<vector<int>> (dense 0-based cores).
int FindCoreOfTask(const std::vector<std::vector<int>>& per_core,
                   int task_id);

// Outcome of comparing the candidate's per-core priority orders against the
// champion's: is the difference a single task's priority move, and if so where?
enum class PrioritySwitchStatus {
    NotSingle,  // size mismatch (core migration), >1 changed core, or >1 task
                // moved on the changed core — not a single priority change.
    AllIdentical,  // every core's order matches → no priority change at all.
    SingleChange,  // exactly one task relocated on one core; locators below
                   // filled.
};

struct PrioritySwitchAnalysis {
    PrioritySwitchStatus status = PrioritySwitchStatus::NotSingle;
    int changed_core = -1;   // valid iff SingleChange
    int moved_task_id = -1;  // valid iff SingleChange
    int old_pos = -1;        // valid iff SingleChange (champion's order)
    int new_pos = -1;        // valid iff SingleChange (candidate's order)
};

// Per-core priority analysis: compare ONE core's candidate order against its
// champion order (two task-id vectors in priority order). Returns:
//   • AllIdentical — the two orders match exactly (no priority move on this
//   core). • SingleChange — exactly one task relocated;
//   moved_task_id/old_pos/new_pos
//     filled (old_pos in the champion's order, new_pos in the candidate's).
//   • NotSingle   — the orders differ by more than one task's relocation.
// Caller guarantees the two vectors are the SAME size (a size mismatch is a
// core migration, a multi-change the per-core-call loop rejects before this).
// The "remove one task from both vectors, compare the rest" test: at the first
// mismatch i the moved task is champion_order[i] OR candidate_order[i] — try
// removing each; if either makes the rest match, that's the single move; if
// neither → NotSingle.
PrioritySwitchStatus AnalyzePrioritySwitchPerCore(
    const std::vector<int>& candidate_order,
    const std::vector<int>& champion_order, PrioritySwitchAnalysis& out);

// Analyze how the candidate's per-core priority orders differ from the
// champion's, using ONLY the two order maps (no ET info). Steps:
//   1. Per-core size check: a core whose task count changed ⇒ a task migrated
//      cores (a multi-change) ⇒ NotSingle.
//   2. Find the one core (if any) whose order differs: 0 → AllIdentical; 1 →
//      delegate the per-core remove-and-compare to
//      AnalyzePrioritySwitchPerCore; ≥2 → NotSingle.
// This handles the pure-priority-move case (0 ET diff). The ET-known case (the
// moved task is the ET-changed task) is handled in IsSingleTaskChange,
// which calls this for the size/changed-core check then does its own
// remove-and-compare with the known task id.
// An absent core and an empty core are indistinguishable here — both mean
// "zero tasks" — so the flat-vector size check collapses the missing-core and
// emptied-core cases into one.
PrioritySwitchAnalysis AnalyzePrioritySwitch(
    const std::vector<std::vector<int>>& candidate_per_core,
    const std::vector<std::vector<int>>& champion_per_core);

}  // namespace SP_OPT_PA
