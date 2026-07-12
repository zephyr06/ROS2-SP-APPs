"""Tests for the post-generation feasibility clamp (P1.8, fix F1).

The clamp is the user's simpler design for the INCR-vs-INCR_WCET anomaly's
*substrate* (WCET/mu > deadline/period tasksets generated with no feasibility
guard): a single deterministic pass over every emitted
``taskset_characteristics*.yaml`` that, for any non-perf task whose
``execution_time_mu`` exceeds ``0.95*period``, pulls ``mu`` (and the scored
support bound ``execution_time_max`` — see the ``FiniteDist``-truncates-at-``max``
trace in ``feasibility_clamp.py``) back down to ``0.95*period`` and relabels
``deadline`` as ``period``. Perf-record tasks (``performance_records_time``
present) are SKIPPED — their ``min``/``max`` are the optimizer's TL-option grid
bounds, semantically distinct from the ET-distribution support, and clamping
them would corrupt the grid.

These tests build synthetic characteristics YAMLs in the exact on-disk format
produced by ``yaml_exporter.export_taskset_to_yaml`` so the round-trip through
``yaml.safe_load`` / ``SpaceSeparatedListDumper`` is exercised for real.
"""
import os
import yaml

from Gen_Taskset.lib.yaml_exporter import export_taskset_to_yaml
from Gen_Taskset.lib.feasibility_clamp import clamp_avg_et_to_period


def _write_characteristics(yaml_dir: str, tasks: list, fname: str = "taskset_characteristics.yaml") -> str:
    """Write a characteristics YAML in the on-disk format the clamp must read."""
    data = {"tasks": tasks}
    path = os.path.join(yaml_dir, fname)
    export_taskset_to_yaml(data, path)
    return path


def _make_task(
    gid: int,
    mu: float,
    sigma: float = 1.0,
    et_min: float = None,
    et_max: float = None,
    period: int = 20,
    deadline: int = None,
    perf_time: str = None,
    perf_perf: str = None,
    processor_id: int = 0,
) -> dict:
    """Build one task dict in the C++-facing characteristics schema."""
    if et_min is None:
        et_min = mu
    if et_max is None:
        et_max = mu
    if deadline is None:
        deadline = int(round(period * 0.7))
    t = {
        "id": gid,
        "gid": gid,
        "execution_time_mu": mu,
        "execution_time_sigma": sigma,
        "execution_time_min": et_min,
        "execution_time_max": et_max,
        "period": period,
        "deadline": deadline,
        "processorId": processor_id,
        "name": f"task_{gid + 1}",
        "sp_threshold": 0.2,
        "sp_weight": 1.0,
        "total_running_time": 30000,
    }
    if perf_time is not None:
        t["performance_records_time"] = perf_time
        t["performance_records_perf"] = perf_perf
    return t


# ---------------------------------------------------------------------------
# Deterministic task (min==max==mu): the taskset_1 task-2 case.
# Clamping mu alone is a no-op on the scored SP here because min==max==mu,
# so the clamp must also pull min/max down. This is the case that motivated
# the max/min clamp (see feasibility_clamp.py docstring).
# ---------------------------------------------------------------------------
def test_clamp_fires_on_deterministic_over_period_task(tmp_path):
    det = _make_task(gid=0, mu=21.4, et_min=21.4, et_max=21.4, period=20, deadline=14)
    _write_characteristics(str(tmp_path), [det])

    report = clamp_avg_et_to_period(str(tmp_path), et_over_period_cap=0.95)

    with open(os.path.join(str(tmp_path), "taskset_characteristics.yaml")) as f:
        out = yaml.safe_load(f)
    t = out["tasks"][0]
    # mu pulled to cap*period = 0.95*20 = 19.0
    assert t["execution_time_mu"] == 19.0
    # max MUST move with mu, else the scored support (truncated at max) is unchanged
    assert t["execution_time_max"] == 19.0
    # min clamped to <= the clamped max
    assert t["execution_time_min"] == 19.0
    assert t["execution_time_min"] <= t["execution_time_max"]
    # deadline relabeled to period (the user's spec)
    assert t["deadline"] == 20
    # period is untouched
    assert t["period"] == 20
    # report is honest about what fired
    assert report["files_written"] == 1
    assert report["tasks_clamped"] == 1


# ---------------------------------------------------------------------------
# Perf-record task: mu=518 > 0.95*500=475, but min/max are TL-grid bounds.
# MUST be skipped — clamping would corrupt the optimizer's search grid.
# This is the taskset_7 task-2 case (verified non-hypothetical in the N=4 run).
# ---------------------------------------------------------------------------
def test_clamp_skips_perf_record_tasks(tmp_path):
    perf = _make_task(
        gid=0,
        mu=518.0,
        et_min=25.0,
        et_max=450.0,
        period=500,
        deadline=475,
        perf_time="25.000 100.000 450.000",
        perf_perf="0.1 0.5 1.0",
    )
    _write_characteristics(str(tmp_path), [perf])

    report = clamp_avg_et_to_period(str(tmp_path), et_over_period_cap=0.95)

    with open(os.path.join(str(tmp_path), "taskset_characteristics.yaml")) as f:
        out = yaml.safe_load(f)
    t = out["tasks"][0]
    assert t["execution_time_mu"] == 518.0
    assert t["execution_time_max"] == 450.0
    assert t["execution_time_min"] == 25.0
    assert t["deadline"] == 475
    assert report["tasks_clamped"] == 0


# ---------------------------------------------------------------------------
# Clean task: mu well below 0.95*period. Untouched (the common case — 38/40
# tasks in the N=4 run).
# ---------------------------------------------------------------------------
def test_clamp_leaves_clean_task_untouched(tmp_path):
    clean = _make_task(gid=0, mu=10.0, et_min=5.0, et_max=15.0, period=100, deadline=75)
    _write_characteristics(str(tmp_path), [clean])

    report = clamp_avg_et_to_period(str(tmp_path), et_over_period_cap=0.95)

    with open(os.path.join(str(tmp_path), "taskset_characteristics.yaml")) as f:
        out = yaml.safe_load(f)
    t = out["tasks"][0]
    assert t["execution_time_mu"] == 10.0
    assert t["execution_time_max"] == 15.0
    assert t["execution_time_min"] == 5.0
    assert t["deadline"] == 75
    assert report["tasks_clamped"] == 0


# ---------------------------------------------------------------------------
# Mixed taskset: one of each kind in one file. Confirms the per-task gate
# (perf skip + over-period fire + clean pass) composes correctly.
# ---------------------------------------------------------------------------
def test_clamp_mixed_taskset(tmp_path):
    det = _make_task(gid=0, mu=21.4, et_min=21.4, et_max=21.4, period=20, deadline=14)
    perf = _make_task(
        gid=1,
        mu=518.0,
        et_min=25.0,
        et_max=450.0,
        period=500,
        deadline=475,
        perf_time="25.000 100.000 450.000",
        perf_perf="0.1 0.5 1.0",
    )
    clean = _make_task(gid=2, mu=10.0, et_min=5.0, et_max=15.0, period=100, deadline=75)
    _write_characteristics(str(tmp_path), [det, perf, clean])

    report = clamp_avg_et_to_period(str(tmp_path), et_over_period_cap=0.95)

    with open(os.path.join(str(tmp_path), "taskset_characteristics.yaml")) as f:
        out = yaml.safe_load(f)
    by_gid = {t["gid"]: t for t in out["tasks"]}
    # deterministic -> clamped
    assert by_gid[0]["execution_time_mu"] == 19.0
    assert by_gid[0]["execution_time_max"] == 19.0
    assert by_gid[0]["deadline"] == 20
    # perf -> unchanged
    assert by_gid[1]["execution_time_mu"] == 518.0
    assert by_gid[1]["execution_time_max"] == 450.0
    # clean -> unchanged
    assert by_gid[2]["execution_time_mu"] == 10.0
    assert by_gid[2]["execution_time_max"] == 15.0
    assert report["tasks_clamped"] == 1


# ---------------------------------------------------------------------------
# max already below the clamp target but mu above it (mu > cap*period > max).
# This can't arise from the current generator (max >= mu for non-perf tasks
# via the mean±2σ band) but guards the min(max, cap*period) against regressions:
# we must never RAISE max.
# ---------------------------------------------------------------------------
def test_clamp_never_raises_max(tmp_path):
    t = _make_task(gid=0, mu=21.4, et_min=10.0, et_max=12.0, period=20, deadline=14)
    _write_characteristics(str(tmp_path), [t])

    clamp_avg_et_to_period(str(tmp_path), et_over_period_cap=0.95)

    with open(os.path.join(str(tmp_path), "taskset_characteristics.yaml")) as f:
        out = yaml.safe_load(f)
    got = out["tasks"][0]
    # max was 12.0 (< 19.0 target) -> must NOT be raised to 19.0
    assert got["execution_time_max"] == 12.0
    # mu still pulled down to the cap target
    assert got["execution_time_mu"] == 19.0
    # min clamped to <= max (was 10.0 <= 12.0, unchanged)
    assert got["execution_time_min"] == 10.0


# ---------------------------------------------------------------------------
# Walks EVERY characteristics file in the dir, not just the global one.
# The gen pipeline emits taskset_characteristics.yaml (k=0 compat copy),
# taskset_characteristics_interval_{k}.yaml, and per-processor
# taskset_characteristics_i{k}_p{p}.yaml — the clamped task appears in each
# that owns it, so all must be rewritten for the scored SP to actually move.
# Non-characteristics files (taskset_param.yaml) must be left alone.
# ---------------------------------------------------------------------------
def test_clamp_walks_all_characteristics_files_and_skips_param(tmp_path):
    det = _make_task(gid=0, mu=21.4, et_min=21.4, et_max=21.4, period=20, deadline=14)
    # global (k=0 compat)
    _write_characteristics(str(tmp_path), [det], "taskset_characteristics.yaml")
    # an interval file (k=5)
    _write_characteristics(str(tmp_path), [det], "taskset_characteristics_interval_5.yaml")
    # a per-processor file (i5_p0)
    _write_characteristics(str(tmp_path), [det], "taskset_characteristics_i5_p0.yaml")
    # taskset_param.yaml — generator's pre-trace artifact; C++ never reads it
    # for scoring. MUST be untouched.
    param_path = os.path.join(str(tmp_path), "taskset_param.yaml")
    with open(param_path, "w") as f:
        yaml.dump({"tasks": [{"Et_mean": 21.4, "period": 20}]}, f)

    report = clamp_avg_et_to_period(str(tmp_path), et_over_period_cap=0.95)

    # all three characteristics files rewritten, each with the clamp applied
    for fname in (
        "taskset_characteristics.yaml",
        "taskset_characteristics_interval_5.yaml",
        "taskset_characteristics_i5_p0.yaml",
    ):
        with open(os.path.join(str(tmp_path), fname)) as f:
            out = yaml.safe_load(f)
        assert out["tasks"][0]["execution_time_mu"] == 19.0, fname
        assert out["tasks"][0]["execution_time_max"] == 19.0, fname
        assert out["tasks"][0]["deadline"] == 20, fname
    # taskset_param.yaml untouched
    with open(param_path) as f:
        param = yaml.safe_load(f)
    assert param["tasks"][0]["Et_mean"] == 21.4
    assert report["files_written"] == 3
    assert report["tasks_clamped"] == 3


# ---------------------------------------------------------------------------
# Byte-compatibility: the rewrite must use the same SpaceSeparatedListDumper
# so ReadTaskSet parses it identically. A perf task's space-separated
# performance_records_time string must survive the round-trip as a string
# (not a YAML list), because RegularTasks.cpp reads it via .as<std::string>().
# ---------------------------------------------------------------------------
def test_clamp_preserves_perf_records_string_format(tmp_path):
    perf = _make_task(
        gid=0,
        mu=518.0,
        et_min=25.0,
        et_max=450.0,
        period=500,
        deadline=475,
        perf_time="25.000 100.000 450.000",
        perf_perf="0.1 0.5 1.0",
    )
    det = _make_task(gid=1, mu=21.4, et_min=21.4, et_max=21.4, period=20, deadline=14)
    _write_characteristics(str(tmp_path), [perf, det])

    clamp_avg_et_to_period(str(tmp_path), et_over_period_cap=0.95)

    with open(os.path.join(str(tmp_path), "taskset_characteristics.yaml")) as f:
        text = f.read()
    # space-separated string form, not YAML flow/block list
    assert "performance_records_time: 25.000 100.000 450.000" in text
    with open(os.path.join(str(tmp_path), "taskset_characteristics.yaml")) as f:
        out = yaml.safe_load(f)
    perf_out = [t for t in out["tasks"] if t["gid"] == 0][0]
    assert isinstance(perf_out["performance_records_time"], str)
    assert perf_out["performance_records_time"] == "25.000 100.000 450.000"
