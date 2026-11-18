import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import context_cost  # noqa: E402


class TestContextCost(unittest.TestCase):
    def test_tokens_is_bytes_div_4(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d, "a.md").write_bytes(b"x" * 400)
            self.assertEqual(context_cost.tokens(["a.md"], Path(d)), 100)

    def test_missing_file_counts_zero(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(context_cost.tokens(["nope.md"], Path(d)), 0)

    def test_roles_contains_supervisor_coder_auditor(self):
        for r in ("supervisor", "coder", "auditor"):
            self.assertIn(r, context_cost.ROLES)
