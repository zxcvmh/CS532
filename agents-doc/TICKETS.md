# TICKETS — CS532 JetBot (v2, from 2026-11-15)

Status flow: `DRAFT` (Supervisor still writing the locked test) → `READY` (Antigravity may start) →
`IN REVIEW` (report submitted, auditor running) → `DONE` (auditor PASS + committed) | `REJECTED`.
Old v1 tickets: `agents-doc/archive/agents-v1/TICKETS-v1.md` (paths there are dead).

## Template (copy for every new ticket)

```
### TK-<AREA>-<NN>: <title>
- Role: researcher | perception | slam | il-policy | integration | supervisor
- Status: DRAFT | READY | IN REVIEW | DONE | REJECTED
- Problem (with evidence, file:line or command output):
- Allowed files:
- Forbidden files: everything else + agents-doc/LOCKED_PATHS.txt
- Acceptance command:
- Threshold (numbers):
- Regression commands:
- Depends on:
- Assumptions: (what the Supervisor filled in that the human did not say; end with an indented line
  `Human-approved: YYYY-MM-DD` once the human OK'd them. READY needs it: `check_ticket.py <ID> --ready`)
- Needs-human: (open questions; a coder who needs an answer writes agents-doc/needs-human/<ID>.md and stops)
```

---

## Index (read the ticket FILE, not this table)

| id | role | status | file |
|---|---|---|---|
| TK-SLAM-01 | slam | DONE | agents-doc/tickets/TK-SLAM-01.md |
| TK-IL-01 | il-policy | PAUSED | agents-doc/tickets/TK-IL-01.md |
| TK-IL-02 | il-policy | SUPERSEDED | agents-doc/tickets/TK-IL-02.md |
| TK-IL-03 | il-policy | DONE | agents-doc/tickets/TK-IL-03.md |
| TK-CV-01 | perception | DEFERRED | agents-doc/tickets/TK-CV-01.md |
| TK-CV-02 | perception | DRAFT | agents-doc/tickets/TK-CV-02.md |
| TK-INT-01 | supervisor | DONE | agents-doc/tickets/TK-INT-01.md |
| TK-INT-03 | supervisor | DONE | agents-doc/tickets/TK-INT-03.md |
| TK-INT-02 | integration | DRAFT | agents-doc/tickets/TK-INT-02.md |
| TK-SLAM-02 | integration | DONE | agents-doc/tickets/TK-SLAM-02.md |
| TK-SLAM-03 | slam | CLOSED | agents-doc/tickets/TK-SLAM-03.md |
| TK-SLAM-04 | slam | DONE | agents-doc/tickets/TK-SLAM-04.md |
| TK-SLAM-07 | slam | SUPERSEDED | agents-doc/tickets/TK-SLAM-07.md |
| TK-SLAM-08 | slam | SUPERSEDED | agents-doc/tickets/TK-SLAM-08.md |
| TK-SLAM-09 | slam | DONE | agents-doc/tickets/TK-SLAM-09.md |
| TK-INT-09 | supervisor | DONE | agents-doc/tickets/TK-INT-09.md |
| TK-INT-08 | integration | DRAFT | agents-doc/tickets/TK-INT-08.md |
| TK-SLAM-05 | slam | DEFERRED | agents-doc/tickets/TK-SLAM-05.md |
| TK-INT-04 | integration | CANCELLED | agents-doc/tickets/TK-INT-04.md |
| TK-INT-07 | supervisor | IN | agents-doc/tickets/TK-INT-07.md |
| TK-INT-05 | supervisor | DONE | agents-doc/tickets/TK-INT-05.md |
| TK-INT-06 | supervisor | DEFERRED | agents-doc/tickets/TK-INT-06.md |
| TK-IL-04 | il-policy | DONE | agents-doc/tickets/TK-IL-04.md |
| TK-IL-05 | il-policy | READY | agents-doc/tickets/TK-IL-05.md |
| TK-IL-06 | il-policy | DRAFT | agents-doc/tickets/TK-IL-06.md |
| TK-IL-07 | il-policy | DRAFT | agents-doc/tickets/TK-IL-07.md |
| TK-IL-08 | il-policy | DRAFT | agents-doc/tickets/TK-IL-08.md |
| TK-INT-10 | integration | DONE | agents-doc/tickets/TK-INT-10.md |
| TK-INT-11 | integration | DRAFT | agents-doc/tickets/TK-INT-11.md |
| TK-CV-03 | perception | DRAFT | agents-doc/tickets/TK-CV-03.md |
| TK-CV-04 | perception | DRAFT | agents-doc/tickets/TK-CV-04.md |
| TK-CV-02 | perception | DRAFT | agents-doc/tickets/TK-CV-02-rewritten.md |
| TK-CV-05 | perception | DRAFT | agents-doc/tickets/TK-CV-05.md |
| TK-CV-06 | perception | DRAFT | agents-doc/tickets/TK-CV-06.md |
| TK-MEAS-01 | supervisor-assistant | READY | agents-doc/tickets/TK-MEAS-01.md |
| TK-INT-12 | supervisor | DONE | agents-doc/tickets/TK-INT-12.md |
| TK-SLAM-10 | slam | DONE | agents-doc/tickets/TK-SLAM-10.md |
