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


class TestReady(unittest.TestCase):
    def mk(self, assumptions=None, tid="TK-NEW-01"):
        t = "### %s: t\n- Role: il-policy\n- Status: READY\n" % tid
        if assumptions is not None:
            t += "- Assumptions: %s\n" % assumptions
        return ticketlib.parse_tickets(t + "- Acceptance command: `x`\n")[0]

    def test_ready_requires_human_approval(self):
        with tempfile.TemporaryDirectory() as d:
            f = ct.check_ready(self.mk("room is 5x5 m"), Path(d))
            self.assertEqual(f, ["assumptions not approved by the human"])

    def test_ready_passes_when_approved(self):
        with tempfile.TemporaryDirectory() as d:
            t = self.mk("room is 5x5 m\n  Human-approved: 2026-11-18")
            self.assertEqual(ct.check_ready(t, Path(d)), [])

    def test_new_ticket_without_assumptions_fails(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(ct.check_ready(self.mk(None), Path(d)), ["no Assumptions field"])

    def test_open_needs_human_blocks(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d, "TK-NEW-01.md").write_text("Question: which room?\nTried: x\nProposed default: y\n")
            t = self.mk("a\n  Human-approved: 2026-11-18")
            self.assertEqual(ct.check_ready(t, Path(d)), ["open needs-human question"])

    def test_resolved_needs_human_passes(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d, "TK-NEW-01.md").write_text("Question: q\nRESOLVED: 5x5\n")
            t = self.mk("a\n  Human-approved: 2026-11-18")
            self.assertEqual(ct.check_ready(t, Path(d)), [])

    def test_legacy_ticket_warns_not_fails(self):
        real = Path(ROOT, "agents-doc", "tickets", "TK-IL-04.md").read_text(encoding="utf-8")
        t = ticketlib.parse_tickets(real)[0]
        legacy = ct.legacy_ids(Path(ROOT))
        self.assertIn("TK-IL-04", legacy)
        with tempfile.TemporaryDirectory() as d:
            f = ct.check_ready(t, Path(d), legacy)
            self.assertEqual(len(f), 1)
            self.assertTrue(f[0].startswith("WARN"), f)


class TestReviewFixes(unittest.TestCase):
    def repo(self, d):
        git(d, "init", "-q")
        Path(d, "tests", "acceptance").mkdir(parents=True)
        Path(d, "tests/acceptance/t.py").write_text("x\n")
        Path(d, "a.py").write_text("a\n")
        git(d, "add", "-A")
        git(d, "commit", "-qm", "i")

    def test_regression_field_with_parenthetical_is_parsed(self):
        t = ticketlib.parse_tickets("### TK-X-03: t\n- Regression commands (all green): `.venv/bin/python a.py`\n- Depends on: x\n")[0]
        self.assertEqual(ticketlib.backticked(ticketlib.field(t, "Regression commands")), [".venv/bin/python a.py"])

    def test_unrunnable_commands_are_reported_not_dropped(self):
        t = ticketlib.parse_tickets("### TK-X-04: t\n- Acceptance command: `.venv/bin/python -c 1`\n- Regression commands: `test_foo.py`\n")[0]
        f = ct.run_commands(t, Path(ROOT))
        self.assertEqual(len(f), 1)
        self.assertTrue(f[0].startswith("WARN") and "test_foo.py" in f[0], f)

    def test_ran_zero_tests_fails(self):
        t = ticketlib.parse_tickets("### TK-X-05: t\n- Acceptance command: `.venv/bin/python -c \"print('Ran 0 tests')\"`\n")[0]
        f = ct.run_commands(t, Path(ROOT))
        self.assertTrue(any("Ran 0 tests" in x for x in f), f)

    def test_rename_out_of_locked_path_is_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            self.repo(d)
            git(d, "mv", "tests/acceptance/t.py", "moved.py")
            ch = ct.changed_paths(Path(d))
            self.assertIn("tests/acceptance/t.py", ch)
            self.assertIn("moved.py", ch)
            f = ct.check_scope(ch, ["moved.py"], [r"^tests/acceptance/"])
            self.assertTrue(any("LOCKED" in x for x in f), f)

    def test_committed_changes_are_seen_with_base(self):
        with tempfile.TemporaryDirectory() as d:
            self.repo(d)
            base = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=d).decode().strip()
            Path(d, "a.py").write_text("changed\n")
            git(d, "commit", "-qam", "coder committed")
            self.assertEqual(ct.changed_paths(Path(d)), [])
            self.assertEqual(ct.changed_paths(Path(d), base), ["a.py"])

    def test_more_fake_pass_shapes(self):
        for line in ["@pytest.mark.xfail", "raise unittest.SkipTest('x')", "    except Exception: pass"]:
            self.assertTrue(ct.check_diff_hygiene(diff_of("t.py", added=[line])), line)

    def test_assert_true_replacement_is_not_balanced(self):
        d = diff_of("t.py", removed=["    assert err < 0.05"], added=["    assert True"])
        self.assertTrue(ct.check_diff_hygiene(d))

    def test_unparseable_heading_exits_cleanly(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d, "TK-IL-09.md")
            f.write_text("### TK-IL-09 - no colon\n- Role: x\n")
            with self.assertRaises(SystemExit):
                ct.load_ticket(f)

    def test_area_code_with_digit_parses(self):
        self.assertEqual(ticketlib.parse_tickets("### TK-CV2-01: t\n- Status: READY\n")[0].id, "TK-CV2-01")

    def test_scope_star_does_not_cross_slash_and_bare_dir_matches(self):
        self.assertTrue(ct.in_scope("dashboard/a.py", ["dashboard/*.py"]))
        self.assertFalse(ct.in_scope("dashboard/sub/deep.py", ["dashboard/*.py"]))
        self.assertTrue(ct.in_scope("dashboard/sub/deep.py", ["dashboard/**"]))
        self.assertTrue(ct.in_scope("motion/planning/a.py", ["motion/planning"]))


if __name__ == "__main__":
    unittest.main()
