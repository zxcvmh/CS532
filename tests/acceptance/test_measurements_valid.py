"""
LOCKED acceptance test — written by the Claude Supervisor (TK-MEAS-01).
Antigravity agents must NOT edit this file (see agents-doc/LOCKED_PATHS.txt).

A measurement is only accepted when its numbers can be recomputed from the RAW file it names. This test finds
the newest docs/measurements/<YYYY-MM-DD>/ directory and checks results.json against the raw files there:

  ping        results.json["ping"] = {file, n_sent, n_lost, mean_ms, p95_ms, p99_ms}; the raw file is the unedited
              output of `ping -c N`. Recomputed here from the `time=` samples and the summary line (numpy
              percentile, linear interpolation). Tolerance 1 %.
  lidar       results.json["lidar"] = [{file, scenario, n_scans, duration_s}, ...] with >= 3 entries covering the
              scenarios static_person, crossing, head_on. Each raw .npz needs keys t and ranges (n, 360) like
              tests/acceptance/data/field_turns_2026-11-15.npz, duration >= 55 s, and the stored n_scans /
              duration_s must equal the file's.
              Scans must be mostly valid and not constant (a recorder that saved zeros fails).
  inventory   results.json["jetson_inventory"] = {file}; the raw text file must exist and be non-empty.

Run:  .venv/bin/python tests/acceptance/test_measurements_valid.py -v
"""
import json
import glob
import os
import re
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TOL = 0.01
MIN_DURATION_S = 55.0
MAX_GAP_S = 1.0   # no hole longer than this inside a recording (TK-MEAS-01 REJECT: 12.4 s hole passed)
SCENARIOS = {"static_person", "crossing", "head_on"}


def _latest_dir():
    dirs = sorted(d for d in glob.glob(os.path.join(ROOT, "docs", "measurements", "*"))
                  if re.search(r"\d{4}-\d{2}-\d{2}$", d) and os.path.isdir(d))
    return dirs[-1] if dirs else None


class TestMeasurementsValid(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = _latest_dir()
        if cls.dir is None:
            cls.res = None
            return
        path = os.path.join(cls.dir, "results.json")
        cls.res = json.load(open(path, encoding="utf-8")) if os.path.isfile(path) else None

    def _need(self, key):
        self.assertIsNotNone(self.dir, "no docs/measurements/<YYYY-MM-DD>/ directory")
        self.assertIsNotNone(self.res, "no results.json in %s" % self.dir)
        self.assertIn(key, self.res, "results.json has no %r" % key)
        return self.res[key]

    def _raw(self, name):
        path = os.path.join(self.dir, name)
        self.assertTrue(os.path.isfile(path), "raw file missing: %s" % path)
        return path

    def test_ping_recomputed_from_raw(self):
        p = self._need("ping")
        text = open(self._raw(p["file"]), encoding="utf-8", errors="replace").read()
        samples = [float(x) for x in re.findall(r"time[=<]([\d.]+)\s*ms", text)]
        m = re.search(r"(\d+) packets transmitted, (\d+) (?:packets )?received", text)
        self.assertIsNotNone(m, "raw file is not `ping -c N` output (no summary line)")
        sent, got = int(m.group(1)), int(m.group(2))
        self.assertGreaterEqual(sent, 200, "need >= 200 packets")
        self.assertEqual(len(samples), got, "time= samples != received count")
        self.assertEqual(p["n_sent"], sent)
        self.assertEqual(p["n_lost"], sent - got)
        for key, val in (("mean_ms", np.mean(samples)), ("p95_ms", np.percentile(samples, 95)),
                         ("p99_ms", np.percentile(samples, 99))):
            self.assertAlmostEqual(p[key], val, delta=max(TOL * val, 0.05), msg="%s does not match raw" % key)

    def test_lidar_recordings(self):
        entries = self._need("lidar")
        self.assertEqual({e["scenario"] for e in entries}, SCENARIOS, "need one recording per scenario")
        for e in entries:
            with self.subTest(scenario=e["scenario"]):
                d = np.load(self._raw(e["file"]))
                self.assertTrue({"t", "ranges"} <= set(d.files), "npz needs keys t and ranges")
                self.assertEqual(d["ranges"].ndim, 2)
                self.assertEqual(d["ranges"].shape[1], 360)
                dur = float(d["t"][-1] - d["t"][0])
                self.assertGreaterEqual(dur, MIN_DURATION_S, "recording shorter than %g s" % MIN_DURATION_S)
                self.assertLessEqual(float(np.diff(d["t"]).max()), MAX_GAP_S,
                                     "recording has a hole longer than %g s" % MAX_GAP_S)
                self.assertEqual(e["n_scans"], d["ranges"].shape[0])
                self.assertAlmostEqual(e["duration_s"], dur, delta=max(TOL * dur, 0.1))
                finite = d["ranges"][np.isfinite(d["ranges"]) & (d["ranges"] > 0)]
                self.assertGreater(finite.size, 0.5 * d["ranges"].size, "mostly empty scans: recorder saved zeros?")
                self.assertGreater(float(np.std(d["ranges"], axis=0).max()), 0.0, "constant scans")

    def test_inventory_has_raw_text(self):
        inv = self._need("jetson_inventory")
        self.assertGreater(os.path.getsize(self._raw(inv["file"])), 0)


if __name__ == "__main__":
    unittest.main()
