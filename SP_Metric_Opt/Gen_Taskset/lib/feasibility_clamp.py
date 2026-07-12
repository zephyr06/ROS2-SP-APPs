"""Post-generation feasibility clamp (P1.8, fix F1).

A single deterministic pass over every ``taskset_characteristics*.yaml`` the
generation pipeline emitted. For each non-perf task whose
``execution_time_mu`` exceeds ``et_over_period_cap * period`` it pulls the
scored execution-time support back inside the period and relabels the deadline
to the period; perf-record tasks are skipped; everything else is untouched.

WHY THIS EXISTS
---------------
The taskset generator draws deadline and execution-time bounds independently
of each other (``taskset_generator.py``: ``deadline = period * uniform(0.5,1.0)``
with no ``deadline > et`` check; ``execution_time_max = et_mean + 2*sigma``
uncapped vs deadline/period). The result, verified by sweeping the N=4
eval-suite run, is systemic unschedulability: 9/10 generated tasksets contain
at least one task whose WCET (``execution_time_max``) exceeds its deadline,
and several whose *mean* ET exceeds the period. That substrate is what let
``INCR_WCET`` (a degraded ablation) beat every plain ``INCR`` arm in the
eval suite — on the one catastrophically-unschedulable taskset, INCR's
analytic SP collapsed to ~0 while the ablation survived. Clamping the
support inside the period removes the substrate directly. See
``agents/active_tasks/P1_8_incr_wcet_outperforms_incr/`` for the full
investigation.

WHY mu ALONE IS NOT ENOUGH (the max/min clamp)
----------------------------------------------
The C++ scorer (``RegularTasks.cpp:75-79``) builds the scored ``FiniteDist``
from ``GaussianDist(mu, sigma)`` truncated to ``[execution_time_min,
execution_time_max]``. ``FiniteDist`` (``Probability.cpp:18-43``) bins mass
from ``min`` to ``max`` and dumps ALL upper-tail mass onto the ``max`` bin —
so the scored support's upper bound is ``execution_time_max``, NOT ``mu``.
Clamping ``mu`` alone, leaving ``max > cap*period``, leaves the scored
support crossing the period and the unschedulability the clamp is meant to
remove would persist. Therefore when ``mu > cap*period`` the clamp MUST also
pull ``execution_time_max`` down to ``min(max, cap*period)`` (never raising
it), and ``execution_time_min`` down to ``min(min, execution_time_max)`` so
the band stays valid (``min <= max``). For the deterministic case
(``min == max == mu``) this is mandatory, not optional — moving only ``mu``
is a complete no-op on the scored SP because the point mass sits at ``max``.

WHY PERF-RECORD TASKS ARE SKIPPED
---------------------------------
A task with ``performance_records_time`` present is a time-limit-optimizable
("perf") task. Its ``execution_time_min``/``execution_time_max`` are NOT the
ET-distribution support — they are the bounds of the optimizer's TL-option
grid (``period * FINAL_Et_OVER_PERIOD_RANGE``), set in
``taskset_generator.py`` and re-asserted in ``yaml_exporter.py``. Clamping
them would corrupt the TL grid the optimizer searches. The gate is
``not bool(performance_records_time)``. Verified non-hypothetical: taskset_7
task 2 is a perf task with ``mu=518 > 0.95*500`` whose ``min=25, max=450``
are grid bounds.

SCOPE
-----
Walks every ``taskset_characteristics*.yaml`` in ``yaml_dir`` — the global
``taskset_characteristics.yaml`` (k=0 backward-compat copy), every
``taskset_characteristics_interval_{k}.yaml``, and every per-processor
``taskset_characteristics_i{k}_p{p}.yaml``. ``taskset_param.yaml`` is NOT
touched (it is the generator's pre-trace artifact; the C++ scorer never
reads its ``Et_mean`` — only the characteristics YAMLs). The rewrite uses
the same ``SpaceSeparatedListDumper`` the exporter uses, so the output is
byte-compatible with what ``ReadTaskSet`` parses (perf-record strings stay
space-separated strings, not YAML lists — ``RegularTasks.cpp`` reads them
via ``.as<std::string>()``).

No RNG, no regenerate, no retry — deterministic and seed-stable.
"""
import os
import glob
import yaml

from .yaml_exporter import export_taskset_to_yaml


def _is_perf_task(task: dict) -> bool:
    """A task is a perf (time-limit-optimizable) task iff it carries records.

    The generator emits ``performance_records_time`` as a space-separated
    string for perf tasks and omits it (or emits "") for normal/env tasks;
    bool("") and bool(None) are both False, so this one check covers both.
    """
    return bool(task.get("performance_records_time"))


def _clamp_task(task: dict, cap: float) -> bool:
    """Clamp one task's scored ET support inside ``cap * period``.

    Returns True if the task was modified, False if it was left untouched.
    Mutates ``task`` in place. See the module docstring for why mu/max/min
    are all clamped (not mu alone) and why perf tasks are skipped upstream.
    """
    if _is_perf_task(task):
        return False

    period = task.get("period")
    if period is None:
        return False

    target = cap * float(period)
    mu = float(task.get("execution_time_mu", 0.0))
    if mu <= target:
        return False

    # mu -> target (the user's spec: "clamp avg ET back to 0.95*period").
    task["execution_time_mu"] = target

    # max -> min(max, target). NEVER raise max: FiniteDist truncates the
    # Gaussian at max, so max is the scored support's upper bound; raising it
    # would re-introduce the very unschedulability we are removing. If max is
    # already below target, leaving it untouched keeps the support tighter
    # than the cap, which is fine (still <= cap*period).
    et_max = float(task.get("execution_time_max", target))
    new_max = min(et_max, target)
    task["execution_time_max"] = new_max

    # min -> min(min, new_max) so the band stays valid (min <= max). Like max,
    # never raise min. If min was already <= new_max it is unchanged.
    et_min = float(task.get("execution_time_min", new_max))
    task["execution_time_min"] = min(et_min, new_max)

    # deadline -> period (the user's spec: "we also re-generate deadline as
    # period"). A deadline equal to the period is the loosest feasible
    # deadline for a task whose support is now <= cap*period < period.
    task["deadline"] = int(period)

    return True


def clamp_avg_et_to_period(yaml_dir: str, et_over_period_cap: float = 0.95) -> dict:
    """Clamp every over-period non-perf task across all characteristics YAMLs.

    Walks every ``taskset_characteristics*.yaml`` in ``yaml_dir``, clamps
    in place, and rewrites each visited file byte-compatible with the
    exporter's ``SpaceSeparatedListDumper``. ``taskset_param.yaml`` and any
    non-characteristics file are skipped.

    Args:
        yaml_dir: directory containing the emitted ``taskset_characteristics*.yaml``
            files (the generation output dir).
        et_over_period_cap: fraction of the period above which a non-perf
            task's avg ET is clamped back down. Default 0.95.

    Returns:
        A report dict with ``files_written`` (how many characteristics files
        were rewritten) and ``tasks_clamped`` (how many tasks across all
        files had their support clamped). A task that appears in multiple
        files (e.g. the global copy + its per-processor file) counts once
        per file, so ``tasks_clamped`` is the total clamp operations applied,
        not the count of distinct tasks.
    """
    pattern = os.path.join(yaml_dir, "taskset_characteristics*.yaml")
    files = sorted(glob.glob(pattern))

    files_written = 0
    tasks_clamped = 0

    for fpath in files:
        with open(fpath, "r") as f:
            data = yaml.safe_load(f)
        if data is None or "tasks" not in data:
            continue

        modified = False
        for task in data["tasks"]:
            if _clamp_task(task, et_over_period_cap):
                tasks_clamped += 1
                modified = True

        if modified:
            # Re-export with the same dumper the generator uses so the output
            # is byte-compatible with what ReadTaskSet parses (perf-record
            # strings stay space-separated strings, not YAML lists).
            export_taskset_to_yaml(data, fpath)
            files_written += 1

    return {"files_written": files_written, "tasks_clamped": tasks_clamped}
