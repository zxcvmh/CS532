import os
import subprocess
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import ticketlib  # noqa: E402


def monolith_text():
    """TICKETS.md as committed before the split (Task 2 repoints this to the archive copy)."""
    arch = os.path.join(ROOT, "agents-doc", "archive", "TICKETS-monolith-2026-11-16.md")  # repointed by Task 2
    if os.path.exists(arch):
        return open(arch, encoding="utf-8").read()
    return subprocess.check_output(["git", "show", "HEAD:agents-doc/TICKETS.md"], cwd=ROOT).decode("utf-8")


SYN = """### TK-IL-09: Foo: bar
- Role: il-policy
- Status: DONE 2026-11-16 — auditor PASS
- Allowed files: `a.py`,
  `b/`
- Forbidden files: everything else
- Acceptance command: `.venv/bin/python t.py -v`
"""


class TestTicketlib(unittest.TestCase):
    def test_parse_counts_real_tickets(self):
        ts = ticketlib.parse_tickets(monolith_text())
        self.assertEqual(len(ts), 35)
        self.assertEqual(len({t.slug for t in ts}), 35)  # TK-CV-02 appears twice; slugs stay unique
        self.assertIn("TK-CV-02-rewritten", [t.slug for t in ts])
        self.assertEqual(ts[0].id, "TK-SLAM-01")
        self.assertNotIn("TK-<AREA>-<NN>", [t.id for t in ts])

    def test_status_is_first_word(self):
        self.assertEqual(ticketlib.parse_tickets(SYN)[0].status, "DONE")

    def test_title_with_colon(self):
        t = ticketlib.parse_tickets(SYN)[0]
        self.assertEqual((t.id, t.title), ("TK-IL-09", "Foo: bar"))

    def test_field_multiline(self):
        t = ticketlib.parse_tickets(SYN)[0]
        self.assertEqual(ticketlib.backticked(ticketlib.field(t, "Allowed files")), ["a.py", "b/"])

    def test_field_missing_is_empty(self):
        self.assertEqual(ticketlib.field(ticketlib.parse_tickets(SYN)[0], "Threshold"), "")

    def test_body_is_verbatim(self):
        self.assertEqual(ticketlib.parse_tickets(SYN)[0].body, SYN)
