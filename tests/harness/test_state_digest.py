import os
import sys
import unittest
from pathlib import Path

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import context_cost  # noqa: E402

BASELINE_CODER_TOKENS = 16075  # measured 2026-11-18 before the context diet (Task 1 baseline)
DIGEST = Path(ROOT, "agents-doc", "STATE_DIGEST.md")


class TestDigest(unittest.TestCase):
    def test_digest_size_and_headings(self):
        self.assertTrue(DIGEST.is_file())
        text = DIGEST.read_text(encoding="utf-8")
        self.assertLessEqual(len(text.encode("utf-8")), 2048)
        heads = [ln[3:] for ln in text.splitlines() if ln.startswith("## ")]
        self.assertEqual(heads, ["Decisions in force", "Open risks", "Do not repeat"])

    def test_phase1_gate(self):
        r = {k: context_cost.tokens(v, Path(ROOT)) for k, v in context_cost.ROLES.items()}
        self.assertLessEqual(r["supervisor"], 6000, r)
        self.assertLessEqual(r["coder"], 0.40 * BASELINE_CODER_TOKENS, r)
        self.assertNotIn("agents-doc/STATE.md", context_cost.ROLES["supervisor"])


if __name__ == "__main__":
    unittest.main()
