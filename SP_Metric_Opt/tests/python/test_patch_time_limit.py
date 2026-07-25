"""Tests for scripts/lib/patch_time_limit.py -- the parameters.yaml TIME_LIMIT patcher.

Pins the contract run_simulation_plot_eval_ns.sh depends on:
  - the first 'TIME_LIMIT:' line is rewritten in place; the rest of the file
    (incl. the trailing '# seconds' comment) is preserved byte-for-byte;
  - read fails loudly on a missing config / mode block / key / non-integer value
    (no silent default -- a config missing the key is a contract break);
  - patch + restore is a byte-identical round-trip;
  - patch fails loudly when parameters.yaml has no TIME_LIMIT: line;
  - restore is a no-op (returns False) when no backup exists.

The helper is a LINE editor, not a YAML dumper, so these tests guard against a
future "just use yaml.dump" regression that would reorder keys / drop the
`%YAML:1.0` directive / strip inline comments.
"""
import io
import os
import sys
import tempfile
import unittest

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

SCRIPTS_DIR = os.path.join(PROJECT_ROOT, "scripts", "lib")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import patch_time_limit as ptl  # noqa: E402

CONFIGS_DIR = os.path.join(PROJECT_ROOT, "simulation_experiments", "configs")

# A faithful miniature of sources/parameters.yaml: directive, comments, the
# TIME_LIMIT line with a trailing comment, and other keys that must survive
# untouched.
_SAMPLE_YAML = (
    "%YAML:1.0\n"
    "# the time limit to run optimization for one time\n"
    "TIME_LIMIT: 10 # seconds\n"
    "\n"
    "Granularity: 10\n"
    "ReoptimizationPeriod: 10\n"
    "EXPORT_DETAIL_LEVEL: 0\n"
)


def _fresh_dir():
    """An isolated temp dir so the helper's sibling-search for backups cannot
    collide with backups left by other tests (most_recent_backup scans the
    params file's directory). Each test gets its own dir."""
    return tempfile.mkdtemp(prefix="patchtl_test.")


def _write_params(text, directory=None):
    # Write into an isolated dir (default) so the sibling-backup search is scoped
    # to this test alone; otherwise leftover .parameters.yaml.*.bak files from
    # other tests in the shared temp dir get picked up by most_recent_backup.
    directory = directory or _fresh_dir()
    fd, path = tempfile.mkstemp(prefix="params.", suffix=".yaml", dir=directory)
    os.close(fd)
    with open(path, "w") as f:
        f.write(text)
    return path


def _write_config(mode, time_limit_seconds, extra=None):
    block = {"time_limit_seconds": time_limit_seconds}
    if extra:
        block.update(extra)
    cfg = {f"{mode}_mode": block}
    fd, path = tempfile.mkstemp(prefix="cfg.", suffix=".json")
    os.close(fd)
    import json
    with open(path, "w") as f:
        json.dump(cfg, f)
    return path


class TestReadTimeLimitSeconds(unittest.TestCase):

    def test_reads_value_from_mode_block(self):
        cfg = _write_config("test", 5)
        self.assertEqual(ptl.read_time_limit_seconds(cfg, "test"), 5)

    def test_missing_config_raises(self):
        with self.assertRaises(ValueError) as cm:
            ptl.read_time_limit_seconds("/nonexistent/cfg.json", "test")
        self.assertIn("config not found", str(cm.exception))

    def test_missing_mode_block_raises(self):
        cfg = _write_config("test", 1)
        with self.assertRaises(ValueError) as cm:
            ptl.read_time_limit_seconds(cfg, "prod")
        self.assertIn("no 'prod_mode' block", str(cm.exception))

    def test_missing_key_raises_no_silent_default(self):
        # A config WITHOUT time_limit_seconds must FAIL LOUDLY, not fall back to
        # the 1s default -- the contract is that every shipped config carries it.
        import json
        fd, path = tempfile.mkstemp(prefix="cfg.", suffix=".json")
        os.close(fd)
        with open(path, "w") as f:
            json.dump({"test_mode": {"num_tasks_for_cross_task_comparison": [4]}}, f)
        with self.assertRaises(ValueError) as cm:
            ptl.read_time_limit_seconds(path, "test")
        self.assertIn("missing 'time_limit_seconds'", str(cm.exception))

    def test_non_integer_value_raises(self):
        cfg = _write_config("test", "1")  # string, not int
        with self.assertRaises(ValueError) as cm:
            ptl.read_time_limit_seconds(cfg, "test")
        self.assertIn("non-negative integer", str(cm.exception))

    def test_negative_value_raises(self):
        cfg = _write_config("test", -1)
        with self.assertRaises(ValueError) as cm:
            ptl.read_time_limit_seconds(cfg, "test")
        self.assertIn("non-negative integer", str(cm.exception))

    def test_boolean_value_rejected(self):
        # bool is a subclass of int in Python; True must not be accepted as 1.
        cfg = _write_config("test", True)
        with self.assertRaises(ValueError) as cm:
            ptl.read_time_limit_seconds(cfg, "test")
        self.assertIn("non-negative integer", str(cm.exception))


class TestPatchTimeLimitLine(unittest.TestCase):

    def test_rewrites_only_first_time_limit_line(self):
        text = (
            "%YAML:1.0\n"
            "TIME_LIMIT: 10 # seconds\n"
            "TIME_LIMIT: 99 # not this one\n"
            "Granularity: 10\n"
        )
        new_text, changed = ptl.patch_time_limit_line(text, 1)
        self.assertTrue(changed)
        self.assertEqual(
            new_text,
            "%YAML:1.0\n"
            "TIME_LIMIT: 1 # seconds\n"      # first line patched
            "TIME_LIMIT: 99 # not this one\n"  # second untouched
            "Granularity: 10\n",
        )

    def test_preserves_trailing_comment(self):
        new_text, changed = ptl.patch_time_limit_line(_SAMPLE_YAML, 3)
        self.assertTrue(changed)
        self.assertIn("TIME_LIMIT: 3 # seconds\n", new_text)
        # The rest of the file is byte-identical.
        self.assertIn("Granularity: 10\n", new_text)
        self.assertIn("EXPORT_DETAIL_LEVEL: 0\n", new_text)
        self.assertTrue(new_text.startswith("%YAML:1.0\n"))

    def test_no_change_when_already_set(self):
        text = "TIME_LIMIT: 1 # seconds\nGranularity: 10\n"
        new_text, changed = ptl.patch_time_limit_line(text, 1)
        self.assertFalse(changed)
        self.assertEqual(new_text, text)

    def test_no_time_limit_line_raises(self):
        text = "%YAML:1.0\nGranularity: 10\n"
        with self.assertRaises(ValueError) as cm:
            ptl.patch_time_limit_line(text, 1)
        self.assertIn("no 'TIME_LIMIT:' line", str(cm.exception))

    def test_indented_time_limit_line_matched(self):
        # Leading whitespace before the key is allowed (defensive -- the shipped
        # file has none, but a nested future layout should still patch).
        text = "config:\n  TIME_LIMIT: 10 # seconds\n"
        new_text, changed = ptl.patch_time_limit_line(text, 2)
        self.assertTrue(changed)
        self.assertIn("  TIME_LIMIT: 2 # seconds\n", new_text)


class TestPatchAndRestoreRoundTrip(unittest.TestCase):

    def test_patch_then_restore_is_byte_identical(self):
        params = _write_params(_SAMPLE_YAML)
        cfg = _write_config("test", 7)
        original = open(params, "rb").read()
        try:
            time_limit, backup_path, changed = ptl.patch(params, cfg, "test")
            self.assertEqual(time_limit, 7)
            self.assertTrue(changed)
            self.assertTrue(os.path.isfile(backup_path))
            patched = open(params, "rb").read()
            self.assertNotEqual(original, patched)
            self.assertIn(b"TIME_LIMIT: 7 # seconds", patched)
            self.assertTrue(ptl.restore(params, backup_path=backup_path))
            self.assertEqual(open(params, "rb").read(), original)
            self.assertFalse(os.path.exists(backup_path))  # backup removed
        finally:
            os.path.exists(params) and os.remove(params)

    def test_patch_creates_backup_even_when_unchanged(self):
        # When the configured value already matches, no line changes -- but a
        # backup is still created so the caller's restore trap is uniform, and
        # restore then restores the identical bytes (a no-op overwrite).
        params = _write_params("TIME_LIMIT: 1 # seconds\nGranularity: 10\n")
        cfg = _write_config("test", 1)
        original = open(params, "rb").read()
        try:
            time_limit, backup_path, changed = ptl.patch(params, cfg, "test")
            self.assertEqual(time_limit, 1)
            self.assertFalse(changed)
            self.assertTrue(os.path.isfile(backup_path))
            self.assertEqual(open(params, "rb").read(), original)
            self.assertTrue(ptl.restore(params, backup_path=backup_path))
            self.assertEqual(open(params, "rb").read(), original)
        finally:
            os.path.exists(params) and os.remove(params)

    def test_restore_noop_when_no_backup(self):
        params = _write_params(_SAMPLE_YAML)
        try:
            self.assertFalse(ptl.restore(params))
        finally:
            os.remove(params)

    def test_patch_fails_loudly_when_no_time_limit_line(self):
        params = _write_params("%YAML:1.0\nGranularity: 10\n")
        cfg = _write_config("test", 1)
        try:
            with self.assertRaises(ValueError) as cm:
                ptl.patch(params, cfg, "test")
            self.assertIn("no 'TIME_LIMIT:' line", str(cm.exception))
        finally:
            os.remove(params)


class TestShippedConfigsAllCarryKey(unittest.TestCase):
    """Every shipped config the .sh might point at must carry time_limit_seconds
    in both modes, so the helper never falls into its fail-loud branch at run
    time. Mirrors test_experiment_config_loader.TestTimeLimitConfig from the
    consumer side; here it pins the helper's read path against the real configs.
    """

    CONFIGS = (
        "paper_simulation_config.json",
        "compare_against_bf.json",
        "incr_et_profiling.json",
    )

    def test_all_shipped_configs_readable_in_both_modes(self):
        import json
        for name in self.CONFIGS:
            path = os.path.join(CONFIGS_DIR, name)
            with open(path) as f:
                raw = json.load(f)
            for mode in ("test", "prod"):
                value = ptl.read_time_limit_seconds(path, mode)
                self.assertEqual(
                    value, 1,
                    f"{name} {mode}_mode time_limit_seconds must be the 1s default, "
                    f"got {value}")


class TestCli(unittest.TestCase):
    """The .sh invokes `patch` and `restore` subcommands; pin their exit codes
    and stdout contract (backup path is the final line of `patch`)."""

    def _run(self, *argv):
        from contextlib import redirect_stderr, redirect_stdout
        buf_out, buf_err = io.StringIO(), io.StringIO()
        with redirect_stdout(buf_out), redirect_stderr(buf_err):
            code = ptl.main(["patch_time_limit.py", *argv])
        return code, buf_out.getvalue(), buf_err.getvalue()

    def test_patch_cli_prints_backup_path_as_last_line(self):
        params = _write_params(_SAMPLE_YAML)
        cfg = _write_config("test", 4)
        try:
            code, out, err = self._run("patch", cfg, "test", params)
            self.assertEqual(code, 0, err)
            lines = out.strip().splitlines()
            backup_path = lines[-1]
            self.assertTrue(os.path.isfile(backup_path))
            self.assertIn(b"TIME_LIMIT: 4 # seconds", open(params, "rb").read())
            # restore via CLI: pass PARAMS_YAML then the captured backup path
            # (the positional form the .sh uses).
            code, out, err = self._run("restore", params, backup_path)
            self.assertEqual(code, 0, err)
            self.assertFalse(os.path.exists(backup_path))  # backup removed
        finally:
            os.path.exists(params) and os.remove(params)

    def test_patch_cli_missing_config_exits_nonzero(self):
        params = _write_params(_SAMPLE_YAML)
        try:
            code, out, err = self._run("patch", "/nonexistent/cfg.json", "test", params)
            self.assertNotEqual(code, 0)
            self.assertIn("ERROR", err)
        finally:
            os.remove(params)

    def test_patch_cli_missing_key_exits_nonzero(self):
        import json
        fd, cfg = tempfile.mkstemp(prefix="cfg.", suffix=".json")
        os.close(fd)
        with open(cfg, "w") as f:
            json.dump({"test_mode": {"num_tasks_for_cross_task_comparison": [4]}}, f)
        params = _write_params(_SAMPLE_YAML)
        try:
            code, out, err = self._run("patch", cfg, "test", params)
            self.assertNotEqual(code, 0)
            self.assertIn("missing 'time_limit_seconds'", err)
        finally:
            os.remove(cfg)
            os.remove(params)

    def test_restore_cli_noop_when_no_backup(self):
        params = _write_params(_SAMPLE_YAML)
        original = open(params, "rb").read()
        try:
            code, out, err = self._run("restore", params)
            self.assertEqual(code, 0)
            self.assertEqual(open(params, "rb").read(), original)
        finally:
            os.remove(params)

    def test_unknown_subcommand_exits_nonzero(self):
        code, out, err = self._run("frobnicate")
        self.assertEqual(code, 2)


if __name__ == "__main__":
    unittest.main()
