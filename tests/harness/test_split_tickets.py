import os
import re
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import split_tickets  # noqa: E402
import ticketlib  # noqa: E402

ARCHIVE = Path(ROOT, "agents-doc", "archive", "TICKETS-monolith-2026-11-16.md")
JUNK = re.compile(r"^(---|## .*)$")


class TestSplit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = ARCHIVE.read_text(encoding="utf-8")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.out = Path(cls.tmp.name)
        mono = cls.out / "mono.md"
        mono.write_text(cls.text, encoding="utf-8")
        cls.tickets = split_tickets.split(mono, cls.out / "tickets")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_split_is_lossless(self):
        first = self.text.index("\n### TK-SLAM-01") + 1
        orig = [ln for ln in self.text[first:].splitlines() if ln.strip()]
        kept = []
        for f in (self.out / "tickets").glob("*.md"):
            kept += [ln for ln in f.read_text(encoding="utf-8").splitlines() if ln.strip()]
        self.assertFalse(Counter(kept) - Counter(orig), "a file has a line the monolith does not")
        lost = Counter(orig) - Counter(kept)
        self.assertTrue(all(JUNK.match(ln) for ln in lost), "lost non-separator lines: %r" % [k for k in lost if not JUNK.match(k)])

    def test_one_file_per_ticket(self):
        names = sorted(p.stem for p in (self.out / "tickets").glob("*.md"))
        self.assertEqual(names, sorted(t.slug for t in self.tickets))
        self.assertNotIn("TK-<AREA>-<NN>", names)

    def test_index_lists_every_ticket_with_status(self):
        idx = split_tickets.render_index(self.tickets, "# TICKETS\n")
        for t in self.tickets:
            row = [ln for ln in idx.splitlines() if "tickets/%s.md" % t.slug in ln]
            self.assertEqual(len(row), 1, t.slug)
            self.assertIn(t.status, row[0])
            self.assertIn(t.id, row[0])


if __name__ == "__main__":
    unittest.main()
