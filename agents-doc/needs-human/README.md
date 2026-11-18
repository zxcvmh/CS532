# needs-human — a coder's "I am blocked" notes

A coder who lacks context STOPS and writes `agents-doc/needs-human/<TICKET-SLUG>.md` (one file per ticket):

```
Question: <what exactly is missing>
Tried: <what you looked at, with file:line or command output>
Proposed default: <what you would do if the human says "your call">
```

The Supervisor answers by appending `RESOLVED: <answer>`. `scripts/check_ticket.py <ID> --ready` fails while a file
without a `RESOLVED:` line exists. Delete the file only after the ticket is DONE.
