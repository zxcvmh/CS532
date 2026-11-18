import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import check_ticket as ct  # noqa: E402
import ticketlib  # noqa: E402


def git(cwd, *a):
    subprocess.check_call(["git", "-c", "user.email=t@t", "-c", "user.name=t", *a], cwd=cwd,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def diff_of(path, removed=(), added=()):
    return "+++ b/%s\n" % path + "".join("-%s\n" % r for r in removed) + "".join("+%s\n" % a for a in added)


class TestScope(unittest.TestCase):
    def test_in_scope_exact_dir_glob(self):
        allowed = ["a.py", "docs/measurements/**", "tests/"]
        self.assertTrue(ct.in_scope("a.py", allowed))
        self.assertTrue(ct.in_scope("docs/measurements/2026/x.npz", allowed))
        self.assertTrue(ct.in_scope("tests/harness/t.py", allowed))
        self.assertFalse(ct.in_scope("b.py", allowed))
        self.assertFalse(ct.in_scope("docs/other.md", allowed))

    def test_scope_flags_untracked_and_deleted(self):
        with tempfile.TemporaryDirectory() as d:
            git(d, "init", "-q")
            Path(d, "a.py").write_text("x\n")
            Path(d, "b.py").write_text("y\n")
            git(d, "add", "a.py", "b.py")
            git(d, "commit", "-qm", "i")
            Path(d, "b.py").unlink()
            Path(d, "new.py").write_text("z\n")
            ch = ct.changed_paths(Path(d))
            self.assertEqual(sorted(ch), ["b.py", "new.py"])
            f = ct.check_scope(ch, ["a.py"], [])
            self.assertEqual(len(f), 2)

    def test_locked_path_hit(self):
        f = ct.check_scope(["tests/acceptance/t.py"], ["tests/acceptance/t.py"], [r"^tests/acceptance/"])
        self.assertTrue(any("LOCKED" in x for x in f))


class TestHygiene(unittest.TestCase):
    def test_removed_assert(self):
        f = ct.check_diff_hygiene(diff_of("t.py", removed=["    self.assertEqual(a, b)"]))
        self.assertTrue(any("assert" in x for x in f))

    def test_assert_replaced_is_fine(self):
        d = diff_of("t.py", removed=["    assert x == 1"], added=["    assert x == 2"])
        self.assertEqual(ct.check_diff_hygiene(d), [])

    def test_added_skip(self):
        self.assertTrue(ct.check_diff_hygiene(diff_of("t.py", added=["@unittest.skip('later')"])))

    def test_except_pass(self):
        self.assertTrue(ct.check_diff_hygiene(diff_of("m.py", added=["    except Exception:", "        pass"])))

    def test_physics_literal(self):
        self.assertTrue(ct.check_diff_hygiene(diff_of("m.py", added=["CRUISE_SPEED = 0.19"])))
        self.assertEqual(ct.check_diff_hygiene(diff_of("tests/t.py", added=["speed = 0.19"])), [])

    def test_structural_constants_are_not_physics(self):
        d = diff_of("dashboard/b.py", added=["_fwd_pwm_mag = 0.0", "w = (math.pi / 2.0) / turn_time"])
        self.assertEqual(ct.check_diff_hygiene(d), [])

    def test_path_prefix_is_not_stripped_by_char(self):
        f = ct.check_diff_hygiene(diff_of("build/bridge.py", added=["CRUISE_SPEED = 0.19"]))
        self.assertTrue(f and "build/bridge.py" in f[0], f)

    def test_dead_folder_path(self):
        self.assertTrue(ct.check_diff_hygiene(diff_of("m.py", added=["p = '03-MinhHieu/run.py'"])))
        self.assertEqual(ct.check_diff_hygiene(diff_of("notes.md", added=["see archive/x"])), [])


class TestCommands(unittest.TestCase):
    def mk(self, acc):
        t = "### TK-X-01: t\n- Role: slam\n- Status: READY\n- Acceptance command: %s\n- Threshold: n\n" % acc
        return ticketlib.parse_tickets(t)[0]

    def test_no_acceptance_command_does_not_crash(self):
        t = ticketlib.parse_tickets("### TK-X-02: t\n- Role: slam\n- Status: DRAFT\n")[0]
        self.assertEqual(ct.run_commands(t, Path(ROOT)), ["no acceptance command"])

    def test_commands_report_nonzero_exit(self):
        f = ct.run_commands(self.mk('`.venv/bin/python -c "import sys; sys.exit(1)"`'), Path(ROOT))
        self.assertEqual(len(f), 1)
        self.assertIn("exit 1", f[0])

    def test_commands_pass(self):
        self.assertEqual(ct.run_commands(self.mk('`.venv/bin/python -c "print(1)"`'), Path(ROOT)), [])


if __name__ == "__main__":
    unittest.main()
