#!/usr/bin/env python3
"""Patch sources/parameters.yaml's TIME_LIMIT from a config's time_limit_seconds.

run_simulation_and_plot_figures.sh delegates the TIME_LIMIT overwrite here so
the YAML edit lives in readable Python rather than an inline awk/JSON one-liner.

Why this file exists:
  The C++ binary reads TIME_LIMIT from sources/parameters.yaml at STATIC INIT
  (Parameters.cpp runs YAML::LoadFile before main()), so the file must carry
  the configured value before the binary process starts -- a runtime env-var or
  CLI flag would not work. This helper:

    1. reads `time_limit_seconds` (default 1) out of the active <mode>_mode
       block of an experiment config JSON;
    2. backs up sources/parameters.yaml to a temp file (so any exit path can
       restore the pristine original);
    3. rewrites ONLY the first `TIME_LIMIT:` line in place, preserving the rest
       of the file verbatim (including the trailing `# seconds` comment);
    4. restores the backup on demand (`restore`), so a crash / Ctrl-C never
       leaves a patched YAML in the working tree.

  `time_limit_seconds` bounds ONE optimizer call (the per-activation budget a
  single EnumeratePA_with_TimeLimits / OptimizeIncre call runs against), NOT
  the task time-limits themselves.

CLI:
    patch_time_limit.py patch   CONFIG_JSON MODE [PARAMS_YAML]
    patch_time_limit.py restore [PARAMS_YAML [BACKUP_PATH]]

  - `patch`   : read time_limit_seconds from CONFIG_JSON's <MODE>_mode block and
                patch PARAMS_YAML (defaults to sources/parameters.yaml). Backs
                up to a sibling temp file and prints its path as the final
                stdout line so the caller can restore later.
  - `restore` : restore PARAMS_YAML (defaults to sources/parameters.yaml) from
                BACKUP_PATH if given, else the most recent sibling backup; remove
                the backup after. No-op (exit 0) when no backup exists.

  Both subcommands exit non-zero (with a stderr message) on a contract break:
  missing config / missing <mode>_mode block / missing time_limit_seconds /
  non-integer value / missing parameters.yaml / no TIME_LIMIT line to patch.

This is a line-based editor, NOT a YAML dumper: parameters.yaml carries a
`%YAML:1.0` directive and inline comments that PyYAML's dumper would reorder or
drop (cf. Gen_Taskset/tests/test_feasibility_clamp.py's SpaceSeparatedListDumper
trouble). Editing the single TIME_LIMIT line preserves everything else byte-for-byte.
"""
import json
import os
import re
import shutil
import sys
import tempfile

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
DEFAULT_PARAMS_YAML = os.path.join(PROJECT_ROOT, "sources", "parameters.yaml")

DEFAULT_TIME_LIMIT = 1  # seconds; the project default when the key is absent.

# Matches the first 'TIME_LIMIT:' line. Anchored at line start (allowing leading
# whitespace), captures the key + colon + separating whitespace as group 1 and
# the integer value as group 2 -- but NOT any trailing comment, so the rewrite
# preserves ` # seconds`. Python's `re` lacks POSIX `[[:space:]]`, so use `\s`.
_TIME_LIMIT_LINE = re.compile(r"^(\s*TIME_LIMIT:\s*)([0-9]+)")


def read_time_limit_seconds(config_path, mode):
    """Read time_limit_seconds from <mode>_mode of config_path.

    Fails loudly (raises ValueError) if the file / mode block / key is missing
    or the value is not a non-negative integer -- a config missing the key is a
    contract break, not a silent default (cf. the P2.8 no-silent-alias rule).
    """
    if not os.path.isfile(config_path):
        raise ValueError(f"config not found: {config_path}")
    with open(config_path) as f:
        raw = json.load(f)
    mode_key = f"{mode}_mode"
    block = raw.get(mode_key)
    if not isinstance(block, dict):
        raise ValueError(
            f"{config_path} has no '{mode_key}' block (got {type(block).__name__})")
    if "time_limit_seconds" not in block:
        raise ValueError(
            f"{config_path} {mode_key} missing 'time_limit_seconds' -- "
            f"every shipped config must carry the key (contract break, not a "
            f"silent default)")
    value = block["time_limit_seconds"]
    # bool is a subclass of int -- reject True/False explicitly.
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(
            f"{config_path} {mode_key} time_limit_seconds must be a non-negative "
            f"integer, got: {value!r}")
    return value


def patch_time_limit_line(text, time_limit):
    """Rewrite the first TIME_LIMIT: line in `text` to the given value.

    Returns (new_text, changed). `changed` is False when the line already
    carried `time_limit` (so the caller can skip a spurious restore). Raises
    ValueError when no TIME_LIMIT: line is present.
    """
    rebuilt = []
    changed = False
    done = False
    for line in text.splitlines(keepends=True):
        if not done:
            match = _TIME_LIMIT_LINE.match(line)
            if match:
                prefix, current = match.group(1), match.group(2)
                if int(current) != time_limit:
                    line = f"{prefix}{time_limit}" + line[match.end():]
                    changed = True
                done = True
        rebuilt.append(line)
    if not done:
        raise ValueError("no 'TIME_LIMIT:' line found in parameters.yaml to patch")
    return "".join(rebuilt), changed


def backup(params_yaml):
    """Copy params_yaml to a temp file in the same dir; return the backup path.

    Same-dir so the restore is a cheap same-filesystem copy and the backup
    lives next to the file it protects. shutil.copy2 preserves mode + mtime so
    a no-op restore (value unchanged) leaves no trace.
    """
    if not os.path.isfile(params_yaml):
        raise ValueError(f"parameters.yaml not found: {params_yaml}")
    fd, backup_path = tempfile.mkstemp(
        prefix=".parameters.yaml.", suffix=".bak", dir=os.path.dirname(params_yaml))
    os.close(fd)
    shutil.copy2(params_yaml, backup_path)
    return backup_path


def patch(params_yaml, config_path, mode):
    """Back up params_yaml, then patch its first TIME_LIMIT: line from the config.

    Returns (time_limit, backup_path, changed). Always creates a backup (even
    when the value is unchanged) so the caller's restore trap is uniform.
    """
    time_limit = read_time_limit_seconds(config_path, mode)
    backup_path = backup(params_yaml)
    with open(params_yaml, "r") as f:
        text = f.read()
    new_text, changed = patch_time_limit_line(text, time_limit)
    with open(params_yaml, "w") as f:
        f.write(new_text)
    return time_limit, backup_path, changed


def most_recent_backup(params_yaml):
    """Find the newest .parameters.yaml.*.bak sibling of params_yaml (or None)."""
    directory = os.path.dirname(params_yaml)
    if not os.path.isdir(directory):
        return None
    candidates = [
        os.path.join(directory, name)
        for name in os.listdir(directory)
        if name.startswith(".parameters.yaml.") and name.endswith(".bak")
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]


def restore(params_yaml, backup_path=None):
    """Restore params_yaml from backup_path (or the most recent sibling backup).

    Removes the backup after restoring. No-op (returns False) when no backup
    exists. Returns True if a restore happened.
    """
    if backup_path is None:
        backup_path = most_recent_backup(params_yaml)
    if not backup_path or not os.path.isfile(backup_path):
        return False
    shutil.copy2(backup_path, params_yaml)
    os.remove(backup_path)
    return True


def main(argv):
    if len(argv) < 2 or argv[1] not in ("patch", "restore"):
        print(__doc__, file=sys.stderr)
        return 2
    subcommand = argv[1]
    params_yaml = DEFAULT_PARAMS_YAML
    try:
        if subcommand == "patch":
            if len(argv) < 4:
                print("usage: patch_time_limit.py patch CONFIG_JSON MODE [PARAMS_YAML]",
                      file=sys.stderr)
                return 2
            config_path, mode = argv[2], argv[3]
            if len(argv) >= 5:
                params_yaml = argv[4]
            time_limit, backup_path, changed = patch(params_yaml, config_path, mode)
            # Human-readable line(s) for the .sh header, then the machine-readable
            # backup path as the FINAL stdout line so the caller can capture it.
            note = "" if changed else " (already set -- no change)"
            print(f"  TIME_LIMIT:   {time_limit}s{note} "
                  f"(patched in {os.path.relpath(params_yaml, PROJECT_ROOT)}; "
                  f"restored on exit)")
            print(backup_path)
            return 0
        else:  # restore
            # restore [PARAMS_YAML] [BACKUP_PATH]: explicit args, no guessing.
            if len(argv) >= 3:
                params_yaml = argv[2]
            if len(argv) >= 4:
                explicit_backup = argv[3]
            else:
                explicit_backup = None
            restored = restore(params_yaml, backup_path=explicit_backup)
            if restored:
                print(f"  TIME_LIMIT: restored {os.path.relpath(params_yaml, PROJECT_ROOT)} "
                      f"from backup")
            return 0
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:  # pragma: no cover - defensive
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
