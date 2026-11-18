#!/usr/bin/env python3
"""Mechanical half of the audit: scope, locked paths, diff hygiene, acceptance commands.

Usage: check_ticket.py TK-ID [--only scope|hygiene|commands]. Exit 1 if any check FAILs.
The auditor reads this output and spends tokens only on judgment (units/frames, citations).
"""
import argparse
import fnmatch
import re
import shlex
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ticketlib  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
_PHYSICS = re.compile(r"(speed|pwm|turn|trim|omega|velocity)", re.I)
_NUMBER = re.compile(r"\b(?!(?:0|1|2)\.0\b)\d+\.\d+\b")
_SKIP = re.compile(r"@unittest\.skip|pytest\.mark\.skip|pytest\.skip\(|\.skipTest\(")
_DEAD = re.compile(r"\b0[1-7]-[A-Za-z]|archive/")
_METRIC = re.compile(r"\[METRIC\]|^Ran \d+ tests|^OK|^FAILED")


def changed_paths(root):
    out = subprocess.check_output(["git", "status", "--porcelain=v1", "-z", "--untracked-files=all"],
                                  cwd=root).decode("utf-8", "replace")
    parts = out.split("\0")
    paths, i = [], 0
    while i < len(parts):
        e = parts[i]
        i += 1
        if len(e) < 4:
            continue
        paths.append(e[3:])
        if e[0] in "RC":  # rename/copy: next entry is the source path
            i += 1
    return paths


def in_scope(path, allowed):
    for a in allowed:
        if a.endswith("/**"):
            a = a[:-2]
        if a.endswith("/"):
            if path.startswith(a):
                return True
        elif "*" in a:
            if fnmatch.fnmatch(path, a):
                return True
        elif path == a:
            return True
    return False


def check_scope(changed, allowed, locked_patterns):
    out = []
    for p in changed:
        if not in_scope(p, allowed):
            out.append("out of scope: %s" % p)
        for pat in locked_patterns:
            if re.search(pat, p):
                out.append("LOCKED path hit: %s" % p)
                break
    return out


def check_diff_hygiene(diff):
    out, path, removed, added, prev_added = [], "", 0, 0, ""
    def flush():
        if path and removed > added:
            out.append("assert removed without replacement in %s (%d removed, %d added)" % (path, removed, added))
    for ln in diff.splitlines():
        if ln.startswith("+++ "):
            flush()
            path, removed, added, prev_added = ln[4:].removeprefix("b/"), 0, 0, ""
            continue
        if ln.startswith("---") or ln.startswith("@@") or not path:
            continue
        body = ln[1:]
        if ln.startswith("-") and "assert" in body:
            removed += 1
        if not ln.startswith("+"):
            prev_added = ""
            continue
        if "assert" in body:
            added += 1
        code = not path.endswith((".md", ".txt", ".json"))
        if _SKIP.search(body):
            out.append("skip added in %s: %s" % (path, body.strip()))
        if prev_added.startswith("except") and body.strip() == "pass":
            out.append("except/pass added in %s" % path)
        if code and not path.startswith(("tests/", "docs/")) and path != "setup/robot_dynamics.json":
            if _PHYSICS.search(body) and _NUMBER.search(body) and not body.strip().startswith("#"):
                out.append("numeric literal near a physics word in %s: %s" % (path, body.strip()))
        if code and _DEAD.search(body):
            out.append("dead folder / archive path in %s: %s" % (path, body.strip()))
        prev_added = body.strip()
    flush()
    return out


def run_commands(ticket, root):
    acc = ticketlib.backticked(ticketlib.field(ticket, "Acceptance command"))
    if not acc:
        return ["no acceptance command"]
    cmds = acc + ticketlib.backticked(ticketlib.field(ticket, "Regression commands"))
    out = []
    for c in cmds:
        if not c.startswith(".venv/bin/python"):
            continue
        r = subprocess.run(shlex.split(c), cwd=root, capture_output=True, text=True, timeout=900)
        text = (r.stdout + r.stderr).splitlines()
        for ln in [x for x in text if _METRIC.search(x)][-6:]:
            print("    | " + ln)
        if r.returncode != 0:
            out.append("`%s` exit %d" % (c, r.returncode))
    return out


def _full_diff(root):
    d = subprocess.run(["git", "diff", "HEAD"], cwd=root, capture_output=True, text=True).stdout
    for p in subprocess.check_output(["git", "ls-files", "--others", "--exclude-standard"], cwd=root).decode().split("\n"):
        f = root / p
        if p and f.is_file() and f.stat().st_size < 200_000:
            d += subprocess.run(["git", "diff", "--no-index", "--", "/dev/null", p], cwd=root,
                                capture_output=True, text=True).stdout
    return d


def _ticket(tid):
    f = ROOT / "agents-doc" / "tickets" / (tid + ".md")
    if not f.is_file():
        sys.exit("no ticket file %s" % f)
    return ticketlib.parse_tickets(f.read_text(encoding="utf-8"))[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ticket")
    ap.add_argument("--only", choices=["scope", "hygiene", "commands"])
    a = ap.parse_args()
    t = _ticket(a.ticket)
    failed = False
    def report(name, findings):
        nonlocal failed
        if findings:
            failed = True
        for f in findings or ["ok"]:
            print("%s %s: %s" % ("FAIL" if findings else "PASS", name, f))
    if a.only in (None, "scope"):
        allowed = ticketlib.backticked(ticketlib.field(t, "Allowed files"))
        locked = [ln.strip() for ln in (ROOT / "agents-doc" / "LOCKED_PATHS.txt").read_text().splitlines() if ln.strip()]
        report("scope", check_scope(changed_paths(ROOT), allowed, locked))
    if a.only in (None, "hygiene"):
        report("hygiene", check_diff_hygiene(_full_diff(ROOT)))
    if a.only in (None, "commands"):
        report("commands", run_commands(t, ROOT))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
