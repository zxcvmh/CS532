#!/usr/bin/env python3
"""One-off: split agents-doc/TICKETS.md into agents-doc/tickets/<slug>.md and rewrite it as an index."""
import re
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ticketlib  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
_JUNK = re.compile(r"^(---|## .*)$")


def _clean(body):
    """Drop the trailing section separators/titles that belong to the monolith, not to the ticket."""
    lines = body.rstrip().splitlines()
    while lines and (not lines[-1].strip() or _JUNK.match(lines[-1])):
        lines.pop()
    return "\n".join(lines) + "\n"


def split(monolith, out_dir):
    tickets = ticketlib.parse_tickets(Path(monolith).read_text(encoding="utf-8"))
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for t in tickets:
        (out_dir / (t.slug + ".md")).write_text(_clean(t.body), encoding="utf-8")
    return tickets


def render_index(tickets, header):
    rows = ["| id | role | status | file |", "|---|---|---|---|"]
    for t in tickets:
        role = ticketlib.field(t, "Role").split()
        rows.append("| %s | %s | %s | agents-doc/tickets/%s.md |" % (t.id, role[0] if role else "", t.status, t.slug))
    return header.rstrip() + "\n\n## Index (read the ticket FILE, not this table)\n\n" + "\n".join(rows) + "\n"


def main():
    src = ROOT / "agents-doc" / "TICKETS.md"
    arch = ROOT / "agents-doc" / "archive" / "TICKETS-monolith-2026-11-16.md"
    if not arch.exists():
        shutil.copy(src, arch)
    text = arch.read_text(encoding="utf-8")
    header = text[: text.index("\n### TK-SLAM-01") + 1]
    header = re.sub(r"^## .*\n\n?---\n?\Z", "", header.rstrip() + "\n")  # no dangling section title
    tickets = split(arch, ROOT / "agents-doc" / "tickets")
    src.write_text(render_index(tickets, header), encoding="utf-8")
    print("wrote %d ticket files and the index" % len(tickets))


if __name__ == "__main__":
    main()
