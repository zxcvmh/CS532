#!/usr/bin/env python3
"""Rough token cost (bytes // 4) of what each agent role loads at session start."""
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# What each role loads at start (STATE.md is a ledger: read the digest; a coder reads one ticket file).
ROLES = {
    "supervisor": ["CLAUDE.md", ".claude/supervisor.md", "agents-doc/STATE_DIGEST.md", "agents-doc/TICKETS.md"],
    "coder": ["AGENTS.md", ".agents/rules/slam.md", "agents-doc/tickets/TK-SLAM-09.md"],
    "auditor": [".claude/agents/quality-auditor.md", "agents-doc/TICKETS.md", "agents-doc/LOCKED_PATHS.txt"],
}


def tokens(paths, root):
    total = 0
    for p in paths:
        f = Path(root) / p
        if f.is_file():
            total += f.stat().st_size // 4
    return total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=sorted(ROLES))
    a = ap.parse_args()
    for role in ([a.role] if a.role else sorted(ROLES)):
        print("%-11s %6d  %s" % (role, tokens(ROLES[role], ROOT), " ".join(ROLES[role])))


if __name__ == "__main__":
    main()
