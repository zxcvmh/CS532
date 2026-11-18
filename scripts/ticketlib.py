"""Ticket parser shared by the harness scripts (stdlib only)."""
import re
from dataclasses import dataclass

_HEAD = re.compile(r"^### (TK-[A-Z]+-\d+)(?: \(([^)]*)\))?: (.*)$")
_FIELD = re.compile(r"^- ([A-Z][A-Za-z -]*):")


@dataclass
class Ticket:
    id: str
    title: str
    status: str
    body: str
    slug: str = ""  # file stem: id, or id-<annotation> when the heading is annotated


def parse_tickets(text):
    """Split a tickets document into Ticket objects; the template heading TK-<AREA>-<NN> is skipped."""
    lines = text.splitlines(keepends=True)
    starts = [i for i, ln in enumerate(lines) if _HEAD.match(ln.rstrip("\n"))]
    out = []
    for n, i in enumerate(starts):
        end = starts[n + 1] if n + 1 < len(starts) else len(lines)
        m = _HEAD.match(lines[i].rstrip("\n"))
        body = "".join(lines[i:end])
        slug = m.group(1) + ("-" + m.group(2).replace(" ", "-") if m.group(2) else "")
        t = Ticket(m.group(1), m.group(3), "", body, slug)
        words = field(t, "Status").split()
        t.status = words[0] if words else ""
        out.append(t)
    return out


def field(ticket, name):
    """Text after '- <name>:' up to the next '- <Capitalised>:' line, '' if absent."""
    lines = ticket.body.splitlines()
    for i, ln in enumerate(lines):
        if ln.startswith("- %s:" % name):
            parts = [ln.split(":", 1)[1].strip()]
            for nxt in lines[i + 1:]:
                if _FIELD.match(nxt) or nxt.startswith("###"):
                    break
                parts.append(nxt.strip())
            return "\n".join(p for p in parts if p).strip()
    return ""


def backticked(text):
    return re.findall(r"`([^`]+)`", text)
