#!/usr/bin/env python3
"""Mechanical half of the audit: scope, locked paths, diff hygiene, acceptance commands.

Usage: check_ticket.py TK-ID [--only scope|hygiene|commands] [--base SHA] [--ready]. Exit 1 if any check FAILs.
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
_SKIP = re.compile(r"@unittest\.skip|pytest\.mark\.(skip|xfail)|pytest\.skip\(|\.skipTest\(|SkipTest\(")
_EXCEPT_PASS = re.compile(r"\bexcept\b.*:\s*pass\b")
_TRIVIAL_ASSERT = re.compile(r"assert\w*\(?\s*True\b")
_DEAD = re.compile(r"\b0[1-7]-[A-Za-z]|archive/")
_METRIC = re.compile(r"\[METRIC\]|^Ran \d+ tests|^OK|^FAILED")


def changed_paths(root, base="HEAD"):
    """Every path that differs from `base` (rename = both paths, deletions, committed changes) plus untracked files."""
    def z(*cmd):
        return [x for x in subprocess.check_output(cmd, cwd=root).decode("utf-8", "replace").split("\0") if x]
    return sorted(set(z("git", "diff", "--no-renames", "--name-only", "-z", base)
                      + z("git", "ls-files", "--others", "--exclude-standard", "-z")))


def _glob(a):
    return re.compile("".join(".*" if t == "**" else "[^/]*" if t == "*" else "[^/]" if t == "?" else re.escape(t)
                              for t in re.split(r"(\*\*|\*|\?)", a)))


def in_scope(path, allowed):
    for a in allowed:
        if a.endswith("/**"):
            a = a[:-2]
        if a.endswith("/"):
            if path.startswith(a):
                return True
        elif re.search(r"[*?]", a):
            if _glob(a).fullmatch(path):
                return True
        elif path == a or path.startswith(a + "/"):
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
        if "assert" in body and not _TRIVIAL_ASSERT.search(body):
            added += 1
        code = not path.endswith((".md", ".txt", ".json"))
        if _SKIP.search(body):
            out.append("skip added in %s: %s" % (path, body.strip()))
        if _EXCEPT_PASS.search(body) or (prev_added.startswith("except") and body.strip() == "pass"):
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
            out.append("WARN not run (not a `.venv/bin/python` command): %s" % c)
            continue
        r = subprocess.run(shlex.split(c), cwd=root, capture_output=True, text=True, timeout=900)
        text = (r.stdout + r.stderr).splitlines()
        if any(re.match(r"Ran 0 tests", x) for x in text):
            out.append("`%s`: Ran 0 tests (nothing was tested)" % c)
        for ln in [x for x in text if _METRIC.search(x)][-6:]:
            print("    | " + ln)
        if r.returncode != 0:
            out.append("`%s` exit %d" % (c, r.returncode))
    return out


def legacy_ids(root):
    """Ticket ids that existed when the HITL gate was introduced (2026-11-16): warned, not failed."""
    arch = Path(root) / "agents-doc" / "archive" / "TICKETS-monolith-2026-11-16.md"
    return {t.id for t in ticketlib.parse_tickets(arch.read_text(encoding="utf-8"))}


def check_ready(ticket, needs_human_dir, legacy=frozenset()):
    """HITL gate: READY needs approved Assumptions and no open needs-human file. 'WARN ...' does not fail."""
    out = []
    assumptions = ticketlib.field(ticket, "Assumptions")
    if not assumptions:
        out.append("WARN legacy ticket: no Assumptions (not held to the gate)" if ticket.id in legacy
                   else "no Assumptions field")
    elif not re.search(r"Human-approved: \d{4}-\d{2}-\d{2}", assumptions):
        out.append("assumptions not approved by the human")
    nh = Path(needs_human_dir) / (ticket.slug + ".md")
    if nh.is_file() and not re.search(r"^RESOLVED:", nh.read_text(encoding="utf-8"), re.M):
        out.append("open needs-human question")
    return out


def _full_diff(root, base="HEAD"):
    d = subprocess.run(["git", "diff", base], cwd=root, capture_output=True, text=True).stdout
    for p in subprocess.check_output(["git", "ls-files", "--others", "--exclude-standard"], cwd=root).decode().split("\n"):
        f = root / p
        if p and f.is_file() and f.stat().st_size < 200_000:
            d += subprocess.run(["git", "diff", "--no-index", "--", "/dev/null", p], cwd=root,
                                capture_output=True, text=True).stdout
    return d


def load_ticket(path):
    ts = ticketlib.parse_tickets(Path(path).read_text(encoding="utf-8"))
    if not ts:
        sys.exit("cannot parse a `### TK-AREA-NN: title` heading in %s" % path)
    return ts[0]


def _ticket(tid):
    f = ROOT / "agents-doc" / "tickets" / (tid + ".md")
    if not f.is_file():
        sys.exit("no ticket file %s" % f)
    return load_ticket(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ticket")
    ap.add_argument("--only", choices=["scope", "hygiene", "commands"])
    ap.add_argument("--base", default="HEAD", help="compare against this commit (the SHA at hand-off); catches coder commits")
    ap.add_argument("--ready", action="store_true", help="HITL gate for marking the ticket READY")
    a = ap.parse_args()
    t = _ticket(a.ticket)
    failed = False
    def report(name, findings):
        nonlocal failed
        hard = [f for f in findings if not f.startswith("WARN")]
        failed = failed or bool(hard)
        for f in findings or ["ok"]:
            print("%s %s: %s" % ("WARN" if f.startswith("WARN") else ("FAIL" if findings else "PASS"), name, f[5:] if f.startswith("WARN ") else f))
    if a.ready:
        report("ready", check_ready(t, ROOT / "agents-doc" / "needs-human", legacy_ids(ROOT)))
        sys.exit(1 if failed else 0)
    if a.only in (None, "scope"):
        allowed = ticketlib.backticked(ticketlib.field(t, "Allowed files"))
        locked = [ln.strip() for ln in (ROOT / "agents-doc" / "LOCKED_PATHS.txt").read_text().splitlines() if ln.strip()]
        report("scope", check_scope(changed_paths(ROOT, a.base), allowed, locked))
    if a.only in (None, "hygiene"):
        report("hygiene", check_diff_hygiene(_full_diff(ROOT, a.base)))
    if a.only in (None, "commands"):
        report("commands", run_commands(t, ROOT))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
