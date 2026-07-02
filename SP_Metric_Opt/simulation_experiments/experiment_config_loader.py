"""Configuration loader for the simulation experiment pipeline.

Every script that needs experiment parameters should call:

    cfg = load_experiment_config(mode="test")

where ``mode`` is either ``"test"`` or ``"prod"``. The function loads
`configs/experiment_config.json` relative to this module.

Example
-------
>>> from experiment_config_loader import load_experiment_config
>>> cfg = load_experiment_config("test")
>>> print(cfg["num_tasks_for_cross_task_comparison"])
[4, 6]
"""
import json
import os

# Absolute path to ``simulation_experiments/``
_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
# Public so callers (e.g. run_end_to_end_experiments --config_json default) can
# reference the shipped config path without hard-coding it.
DEFAULT_CONFIG_PATH = os.path.join(_MODULE_DIR, "configs", "experiment_config.json")


def load_experiment_config(mode="test", config_path=DEFAULT_CONFIG_PATH):
    """Load experiment parameters from the central JSON configuration file.

    Parameters
    ----------
    mode : {"test", "prod"}
        Which parameter set to return. ``"test"`` uses small/fast values for
        CI and quick smoke tests. ``"prod"`` uses paper-grade values.
    config_path : str
        Path to the JSON config file. Defaults to the shipped config at
        ``configs/experiment_config.json``.

    Returns
    -------
    dict
        Flattened dictionary merging the selected mode block with the global
        ``plotting`` and ``analysis`` sections. Keys are descriptive strings
        such as ``num_tasks_for_cross_task_comparison``.
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Experiment config not found: {config_path}")

    with open(config_path, "r") as f:
        raw = json.load(f)

    mode_key = f"{mode}_mode"
    if mode_key not in raw:
        raise KeyError(f"Config file missing '{mode_key}' section. Valid modes: test, prod.")

    # Merge mode-specific parameters with global plotting/analysis settings
    merged = {}
    merged.update(raw[mode_key])
    merged["plotting"] = raw.get("plotting", {})
    merged["analysis"] = raw.get("analysis", {})
    merged["_config_source_path"] = config_path
    merged["_active_mode"] = mode
    return merged


def build_run_id(config_dict):
    """Return a stable folder key identifying one end-to-end run.

    Two runs that differ in mode, duration, trigger interval, base seed, or
    task-count list get different run ids, so their figures land in separate
    folders and never clobber each other. ``num_tasksets`` (sampling depth) is
    intentionally excluded -- two runs differing only in taskset count share a
    folder, which is acceptable since ``mode`` is the primary axis.

    Parameters
    ----------
    config_dict : dict
        The dictionary returned by :func:`load_experiment_config`.

    Returns
    -------
    str
        e.g. ``"run_test_dur70_interval10_seed1000_tasks4x6"``.
    """
    mode = config_dict.get("_active_mode", "run")
    dur = config_dict.get("simulation_duration_seconds", 0)
    interv = config_dict.get("scheduler_trigger_interval_seconds", 0)
    seed = config_dict.get("base_random_seed", 0)
    tasks = config_dict.get("num_tasks_for_cross_task_comparison", [])
    tasks_tag = "x".join(str(t) for t in tasks) if tasks else "na"
    return f"run_{mode}_dur{dur}_interval{interv}_seed{seed}_tasks{tasks_tag}"


def resolve_config_value(config_dict, key, default=None):
    """Safely fetch a value from the loaded config with an optional default.

    Parameters
    ----------
    config_dict : dict
        The dictionary returned by :func:`load_experiment_config`.
    key : str
        Top-level key name.
    default
        Fallback value if the key is absent.

    Returns
    -------
    Any
        The config value or ``default``.
    """
    return config_dict.get(key, default)


if __name__ == "__main__":
    import pprint

    for m in ("test", "prod"):
        print(f"\n{'='*60}\nMode: {m.upper()}\n{'='*60}")
        cfg = load_experiment_config(m)
        pprint.pprint(cfg)
