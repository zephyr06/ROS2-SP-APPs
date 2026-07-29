"""Tests for the P0.8 important-task GATE (Step 2 — option A wiring).

The gate wraps ``run_full_generation_pipeline``: after the canonical pipeline
emits its characteristics YAMLs (and the feasibility clamp runs — D5:
clamp-first, then RTA), the gate derives each task's WCET per D2 and runs
``important_tasks_schedulable``. On failure it re-runs the pipeline with an
ADVANCED seed (``RANDOM_SEED + attempt``) so each retry actually re-samples
(``generate_taskset_parameters`` re-seeds every call — a naive retry would
produce a byte-identical taskset). Budget 20 (D4), then LOUD raise.

This file tests the (A)-specific pieces the pure-RTA module
(``test_important_task_rta.py``) does NOT cover:
  - ``compute_wcets_from_characteristics``: reads the emitted
    ``taskset_characteristics_interval_*.yaml`` files and returns per-task
    global-max WCET. Perf task = ``period * FINAL_Et_OVER_PERIOD_RANGE[1]``
    (TL-grid upper bound, deterministic across intervals); non-perf =
    MAX ``execution_time_max`` across all interval YAMLs (post-clamp — the
    clamp may pull ``execution_time_max`` DOWN, so the global max MUST be
    taken post-clamp).
  - ``run_full_generation_pipeline_with_important_task_gate``: the retry
    wrapper. Advances the seed per attempt; loud-raises on budget exhaustion
    (NEVER silently emits an unschedulable taskset — re-creates the P1.8
    substrate); returns a report (attempts used, culprits on the final pass).

These tests build synthetic characteristics YAMLs on disk (the gate's input)
so the WCET derivation is tested in isolation from the (expensive) full
pipeline. The end-to-end retry behavior is exercised against the real pipeline
in ``test_integration.py``.
"""
import os
import json
import shutil

import pytest
import yaml

from Gen_Taskset.lib.important_task_rta import compute_wcets_from_characteristics
from Gen_Taskset.lib.yaml_exporter import export_taskset_to_yaml


def _write_characteristics(dir_path: str, tasks: list, interval_k: int, n_cores: int = 1) -> None:
    """Write one ``taskset_characteristics_interval_{k}.yaml`` (and the
    per-processor splits the gate walks) to ``dir_path``.

    ``tasks`` is a list of dicts in the EMITTED (C++-format) representation:
    ``id``, ``gid``, ``period``, ``deadline``, ``processorId``,
    ``execution_time_max``, ``performance_records_time`` (present iff perf),
    etc. — exactly what ``convert_taskset_parameters_to_cpp_yaml`` emits and
    what the gate reads off disk.
    """
    payload = {"tasks": tasks}
    fname = os.path.join(dir_path, f"taskset_characteristics_interval_{interval_k}.yaml")
    export_taskset_to_yaml(payload, fname)
    # The global k=0 copy (the gate should treat it as redundant, not a 3rd
    # source — it's the same data as interval_0; include it to assert the gate
    # does NOT double-count via the global copy when intervals exist).
    if interval_k == 0:
        export_taskset_to_yaml(payload, os.path.join(dir_path, "taskset_characteristics.yaml"))
    # Per-processor splits (CSPSimulation_2 compatibility) — the gate walks
    # these too; assert it does not double-count a task that appears in both
    # the full interval file and its per-processor split.
    for pp in range(n_cores):
        sub = {"tasks": [t for t in tasks if int(t.get("processorId", 0)) == pp]}
        export_taskset_to_yaml(
            sub,
            os.path.join(dir_path, f"taskset_characteristics_i{interval_k}_p{pp}.yaml"),
        )


def _emitted_task(
    gid: int,
    period: int,
    deadline: int,
    execution_time_max: float,
    processor_id: int = 0,
    performance_records_time: str = None,
    is_important: bool = False,
) -> dict:
    """Build one task dict in the EMITTED C++-format representation."""
    t = {
        "id": gid,
        "gid": gid,
        "period": period,
        "deadline": deadline,
        "processorId": processor_id,
        "execution_time_max": execution_time_max,
        "important": is_important,
    }
    if performance_records_time is not None:
        t["performance_records_time"] = performance_records_time
    return t


# --------------------------------------------------------------------------
# compute_wcets_from_characteristics
# --------------------------------------------------------------------------

def test_wcet_non_perf_is_global_max_across_intervals(tmp_path):
    """Non-perf WCET = MAX ``execution_time_max`` across all interval YAMLs.

    A task whose ET support varies interval-to-interval (the trace loop emits
    a per-interval ``Et_max``) takes its GLOBAL max as the WCET — the worst
    interval bounds the response time. This is D2.
    """
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    task = _emitted_task(gid=0, period=1000, deadline=1000,
                         execution_time_max=200, is_important=True)
    _write_characteristics(str(tmp_path), [task], interval_k=0)
    # Interval 1: same task, larger ET support → the global max.
    task_k1 = dict(task)
    task_k1["execution_time_max"] = 350
    _write_characteristics(str(tmp_path), [task_k1], interval_k=1)

    wcets = compute_wcets_from_characteristics(str(tmp_path), cfgs)
    assert wcets == {0: 350.0}


def test_wcet_clamp_lowered_max_is_respected(tmp_path):
    """The gate reads WCET POST-clamp (D5: clamp first, then RTA).

    ``feasibility_clamp`` may pull a non-perf task's ``execution_time_max``
    DOWN (to ``min(max, cap*period)``). The gate reads the clamped (on-disk)
    value — it does NOT re-derive the pre-clamp max. So a task clamped from
    600 → 950-cap*period uses the clamped value as its WCET.
    """
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    # period=1000, cap=0.95 → clamp target = 950. Pre-clamp max 600 < 950, so
    # clamp leaves it at 600 (the clamp only pulls DOWN, never raises). The
    # gate must read 600, not some pre-clamp phantom.
    task = _emitted_task(gid=0, period=1000, deadline=1000,
                         execution_time_max=600, is_important=True)
    _write_characteristics(str(tmp_path), [task], interval_k=0)

    wcets = compute_wcets_from_characteristics(str(tmp_path), cfgs)
    assert wcets == {0: 600.0}


def test_wcet_perf_task_is_tl_grid_upper_bound(tmp_path):
    """Perf task WCET = ``period * FINAL_Et_OVER_PERIOD_RANGE[1]`` (D2).

    A perf task's ``execution_time_min/max`` are TL-grid BOUNDS, not the ET
    support — the exporter forces them to the config range
    (``yaml_exporter.py:79-80``). So the gate must NOT read its
    ``execution_time_max`` (a grid bound); it must compute the TL-grid upper
    bound from the period + config. This is the gap ``feasibility_clamp``
    leaves open (it SKIPS perf tasks) that P0.8 closes.
    """
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    # period=1000 → TL-grid upper bound = 1000 * 0.9 = 900. The on-disk
    # execution_time_max (450) is a grid bound, NOT the WCET — the gate must
    # ignore it and use 900.
    task = _emitted_task(gid=0, period=1000, deadline=1000,
                         execution_time_max=450,
                         performance_records_time="100 200 300 450",
                         is_important=True)
    _write_characteristics(str(tmp_path), [task], interval_k=0)

    wcets = compute_wcets_from_characteristics(str(tmp_path), cfgs)
    assert wcets == {0: 900.0}


def test_wcet_mixed_perf_and_non_perf(tmp_path):
    """Both task types in one taskset: perf → TL-grid bound, non-perf → global max."""
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    perf = _emitted_task(gid=0, period=500, deadline=500,
                         execution_time_max=400,
                         performance_records_time="50 100 200 400",
                         is_important=True)
    non_perf = _emitted_task(gid=1, period=1000, deadline=1000,
                             execution_time_max=200, is_important=True)
    _write_characteristics(str(tmp_path), [perf, non_perf], interval_k=0)
    non_perf_k1 = dict(non_perf)
    non_perf_k1["execution_time_max"] = 300
    _write_characteristics(str(tmp_path), [perf, non_perf_k1], interval_k=1)

    wcets = compute_wcets_from_characteristics(str(tmp_path), cfgs)
    # perf: 500 * 0.9 = 450 (TL-grid bound); non-perf: max(200, 300) = 300.
    assert wcets == {0: 450.0, 1: 300.0}


def test_wcet_does_not_double_count_global_or_per_processor_splits(tmp_path):
    """The gate must NOT inflate the max by reading redundant copies.

    The pipeline emits the SAME task in: the full interval file, the global
    k=0 copy, AND the per-processor split. All carry the same
    ``execution_time_max`` for that interval. A naive max-over-all-files would
    still be correct (max of identical values) — BUT if the global copy or a
    per-processor split were ever stale/different, double-counting would
    silently corrupt the WCET. The gate reads the per-interval full files
    ONLY (the canonical per-interval source) and ignores the global copy +
    per-processor splits.
    """
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    task = _emitted_task(gid=0, period=1000, deadline=1000,
                         execution_time_max=200, is_important=True)
    _write_characteristics(str(tmp_path), [task], interval_k=0, n_cores=2)
    # Sabotage the global copy + a per-processor split with a LARGER max —
    # the gate must NOT pick it up (it reads interval files only).
    sabotaged = dict(task)
    sabotaged["execution_time_max"] = 9999
    export_taskset_to_yaml(
        {"tasks": [sabotaged]},
        os.path.join(str(tmp_path), "taskset_characteristics.yaml"),
    )

    wcets = compute_wcets_from_characteristics(str(tmp_path), cfgs)
    assert wcets == {0: 200.0}


def test_wcet_missing_interval_files_raises(tmp_path):
    """No interval characteristics YAMLs → loud raise (NEVER silent).

    A generation dir with no ``taskset_characteristics_interval_*.yaml`` means
    the pipeline did not emit its canonical output — the gate cannot certify
    what isn't there. Loud raise (not an empty-dict return) so a misconfigured
    dir is caught, not papered over.
    """
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    with pytest.raises(ValueError, match="interval"):
        compute_wcets_from_characteristics(str(tmp_path), cfgs)


def test_wcet_keyed_by_gid_not_id(tmp_path):
    """WCETs are keyed by ``gid`` (the generator's stable task identity), not
    ``id`` (the per-processor-local id, which restarts at 0 in each split)."""
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    # Two tasks: gid 5 (id 0) and gid 7 (id 1) — distinct gids, local ids 0/1.
    t5 = _emitted_task(gid=5, period=1000, deadline=1000,
                       execution_time_max=200, is_important=True)
    t5["id"] = 0
    t7 = _emitted_task(gid=7, period=2000, deadline=2000,
                       execution_time_max=400, is_important=True)
    t7["id"] = 1
    _write_characteristics(str(tmp_path), [t5, t7], interval_k=0)

    wcets = compute_wcets_from_characteristics(str(tmp_path), cfgs)
    assert set(wcets.keys()) == {5, 7}
    assert wcets[5] == 200.0
    assert wcets[7] == 400.0
