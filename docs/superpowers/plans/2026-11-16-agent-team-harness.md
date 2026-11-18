# Agent Team Harness v3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut the tokens each agent role loads, put hard guards on the mistakes already made, add a mechanical ticket checker, and add a human-in-the-loop gate for incomplete context.

**Architecture:** Stdlib-only Python scripts under `scripts/` (laptop side, Python 3.11 from `.venv`) share one ticket parser (`scripts/ticketlib.py`). Claude-side guards are `permissions` rules in `.claude/settings.json`. Antigravity coders cannot be bound by those, so their guard is `scripts/check_ticket.py`, run by the Supervisor and the auditor. Tests live in the new unlocked `tests/harness/` and run with `unittest` (pytest is not installed in `.venv`).

**Tech Stack:** Python 3.11 stdlib (`re`, `json`, `subprocess`, `pathlib`, `argparse`, `unittest`), git, Claude Code `settings.json`.

**Spec:** `docs/superpowers/specs/2026-11-16-agent-team-harness-design.md` (approved 2026-11-16, approach A).

## Global Constraints

- Scripts: stdlib only, no new dependency; run with `.venv/bin/python`; nothing here touches robot code or runs on the Jetson.
- `agents-doc/STATE.md` is an append-only ledger: never split, trim or rewrite it (a digest is a NEW file).
- Locked tests and thresholds (`tests/acceptance/`, `setup/robot_dynamics.json`) are not touched.
- New tests go in `tests/harness/` (NOT `tests/acceptance/`, which is locked).
- No `git push`. One commit per task, `git add <explicit paths>` only (never `-u`/`-A`/`.`); the working tree holds unrelated uncommitted coder WIP. Commit messages end with the attribution line the harness supplies.
- Phase order: 1 context diet, 2 hard guards, 3 `check_ticket.py`, 4 HITL gate. Pause any phase that would delay a coder ticket (SLAM-09, IL-04/05); freeze is 2026-12-14.
- Human decisions: stale root `handoff-cs532-jetbot.md` is NOT moved or deleted without asking; deleting `scripts/verify_agent_integrity.py` needs the human's nod.

## Review Focus

- Ticket headings whose title contains a colon, or a Status line with trailing text (`DONE 2026-11-16 — ...`): the parser must still return id, title and the bare status word.
- `Allowed files` given as a directory or glob (`docs/measurements/**`, `tests/`): a changed file under it is in scope; one outside is not.
- Changed files that are untracked, deleted or renamed (the tree has many untracked files): all count as changes.
- A DRAFT ticket with no acceptance command: the checker reports "no acceptance command", it does not crash.
- Existing READY tickets (IL-04, INT-10, SLAM-09, MEAS-01) have no `Assumptions` field: they pass `--ready` with a legacy warning; only tickets created after 2026-11-16 are held to it.

---

### Task 1: Ticket parser and context-cost meter (phase 1 baseline)

**Files:**
- Create: `scripts/ticketlib.py`, `scripts/context_cost.py`, `tests/harness/__init__.py`, `tests/harness/test_ticketlib.py`, `tests/harness/test_context_cost.py`

**Interfaces:**
- Produces (`scripts/ticketlib.py`):
  - `@dataclass Ticket: id: str, title: str, status: str, body: str` (`body` = the heading line through the line before the next `### TK-`, unchanged).
  - `parse_tickets(text: str) -> list[Ticket]` (only headings matching `^### (TK-[A-Z]+-\d+): (.*)$`; the template heading `TK-<AREA>-<NN>` is skipped; `status` is the first word after `- Status:`).
  - `field(ticket: Ticket, name: str) -> str` (text after `- <name>:` up to the next `- <Capitalised>:` line, `""` if absent).
  - `backticked(text: str) -> list[str]` (all `` `x` `` tokens, in order).
- Produces (`scripts/context_cost.py`): `ROLES: dict[str, list[str]]` (role to repo-relative paths it loads at start), `tokens(paths: list[str], root: Path) -> int` (sum of bytes // 4, missing file counts 0), CLI `context_cost.py [--role R]` printing one line per role `role  tokens  paths`.

- [ ] **Step 1: Write failing tests.** `test_ticketlib.py`: `test_parse_counts_real_tickets` (parses `agents-doc/TICKETS.md`, expects 36 tickets, skips the template, `parse_tickets(...)[0].id == "TK-SLAM-01"`); `test_status_is_first_word` (a synthetic ticket with `- Status: DONE 2026-11-16 — auditor PASS` gives `"DONE"`); `test_title_with_colon` (`### TK-IL-09: Foo: bar` gives title `"Foo: bar"`); `test_field_multiline` (an `Allowed files` value that continues on an indented second line is returned whole); `test_field_missing_is_empty`. `test_context_cost.py`: `test_tokens_is_bytes_div_4` (a 400-byte temp file counts 100), `test_missing_file_counts_zero`, `test_roles_contains_supervisor_coder_auditor` (keys `supervisor`, `coder`, `auditor`).
- [ ] **Step 2: Run and confirm failure.** Run: `.venv/bin/python -m unittest discover tests/harness -v`. Expected: `ModuleNotFoundError` for `ticketlib` / `context_cost`.
- [ ] **Step 3: Implement `scripts/ticketlib.py`** per the Interfaces block (regex line scan, no markdown library).
- [ ] **Step 4: Implement `scripts/context_cost.py`.** Initial `ROLES` = today's loads: `supervisor` = `CLAUDE.md`, `.claude/supervisor.md`, `agents-doc/STATE.md`, `agents-doc/TICKETS.md`; `coder` = `AGENTS.md`, the role rule file `.agents/rules/slam.md` as the representative one, `agents-doc/TICKETS.md`; `auditor` = `.claude/agents/quality-auditor.md`, `agents-doc/TICKETS.md`, `agents-doc/LOCKED_PATHS.txt`.
- [ ] **Step 5: Run tests; expect all pass.** Then record the baseline: `.venv/bin/python scripts/context_cost.py > /tmp/claude-1000/-home-zxcvmh-Projects-CS532/3507666c-c58a-44b9-a86f-0b5fc539ea51/scratchpad/context_baseline.txt` and read it (supervisor should be about 20k tokens, coder about 17k).
- [ ] **Step 6: Commit.** `git add scripts/ticketlib.py scripts/context_cost.py tests/harness/` then `git commit -m "feat(harness): ticket parser and context-cost meter"`.

---

### Task 2: One file per ticket, `TICKETS.md` becomes an index (phase 1)

**Files:**
- Create: `scripts/split_tickets.py` (one-off), `agents-doc/tickets/TK-*.md` (36 files), `agents-doc/archive/TICKETS-monolith-2026-11-16.md`, `tests/harness/test_split_tickets.py`
- Modify: `agents-doc/TICKETS.md` (rewritten as the index), pointers in `AGENTS.md`, `CLAUDE.md`, `.claude/supervisor.md`, `.claude/agents/quality-auditor.md`, `.claude/commands/{ticket,close,triage}.md`, `.agents/workflows/{integration,perception,il-policy,slam}.md`, `agents-doc/PLAYBOOK.md`, `agents-doc/README.md`
- Consumes: `parse_tickets(text) -> list[Ticket]` from Task 1.

**Interfaces:**
- Produces: `split(monolith: Path, out_dir: Path) -> list[Ticket]` writing `<out_dir>/<id>.md` = `ticket.body`; `render_index(tickets: list[Ticket], header: str) -> str` (header = the existing preamble through the template block, then a table `| id | role | status | file |`, then the one-line status flow).
- New convention (also written into `AGENTS.md` rule 1 and the three commands): a coder reads `agents-doc/tickets/<ID>.md` only; the index is for the Supervisor.

- [ ] **Step 1: Write the failing test.** `test_split_is_lossless`: split a temp copy of the monolith; concatenating the 36 files (in order) equals the monolith text from the first ticket heading to the end, minus the `---` separators and section titles; `test_index_lists_every_ticket_with_status`: each id and its bare status appear in `render_index`; `test_template_not_split` (no file for `TK-<AREA>-<NN>`).
- [ ] **Step 2: Run, expect failure** (`split_tickets` missing).
- [ ] **Step 3: Implement** `split` and `render_index`, plus a `main()` that archives the monolith first (`shutil.copy` to `agents-doc/archive/TICKETS-monolith-2026-11-16.md`), then writes the files and the new `TICKETS.md`.
- [ ] **Step 4: Run tests; expect pass.** Run the script once: `.venv/bin/python scripts/split_tickets.py`. Check: `ls agents-doc/tickets | wc -l` prints 36 and `.venv/bin/python scripts/context_cost.py` still runs.
- [ ] **Step 5: Update pointers.** For each file listed under Modify, replace "read/append in `agents-doc/TICKETS.md`" with the per-ticket path (`agents-doc/tickets/<ID>.md`; a NEW ticket = create that file and add a row to the index). Verify with `grep -rn "TICKETS.md" AGENTS.md CLAUDE.md .claude .agents agents-doc/PLAYBOOK.md`: every remaining hit is about the index, not about reading a ticket.
- [ ] **Step 6: Commit** with explicit paths: the scripts, `tests/harness/test_split_tickets.py`, `agents-doc/tickets/`, `agents-doc/TICKETS.md`, `agents-doc/archive/TICKETS-monolith-2026-11-16.md`, and each pointer file edited in step 5 (check `git status` first; several of them already carry unrelated uncommitted coder edits, so stage with `git add -p` for `.agents/workflows/*` and `AGENTS.md` if they show foreign hunks). Message: `refactor(harness): one file per ticket, TICKETS.md is an index`.

---

### Task 3: State digest and phase-1 gate (phase 1)

**Files:**
- Create: `agents-doc/STATE_DIGEST.md`, `tests/harness/test_state_digest.py`
- Modify: `scripts/context_cost.py` (`ROLES`), `.claude/supervisor.md` §7 session ritual, `CLAUDE.md` (one line pointing at the digest)

**Interfaces:**
- Consumes: `tokens(paths, root)` and `ROLES` from Task 1.
- Produces: `agents-doc/STATE_DIGEST.md` with exactly these `##` headings in order: `Decisions in force`, `Open risks`, `Do not repeat`; size <= 2048 bytes. `Do not repeat` = one line per mistake already ledgered (git add -u; deploying the wrong `disparity_extender.py`; SSH/sudo beyond a read-only grant; validators that check a proxy such as `t[-1]-t[0]`; trusting a REPORT without the auditor; heading from key-press time).

- [ ] **Step 1: Write failing tests.** `test_digest_size_and_headings` (exists, <= 2048 bytes, the three headings in order); `test_phase1_gate`: with the new `ROLES`, `supervisor` <= 6000 tokens and `coder` <= 40 % of the baseline recorded in Task 1 (hardcode the baseline number read from `context_baseline.txt` as a named constant `BASELINE_CODER_TOKENS`).
- [ ] **Step 2: Run, expect failure.**
- [ ] **Step 3: Write the digest** from `supervisor.md` §1b/§6 and the ledger (current plan, IL as the only avoider, reflex = brake only, control-loop placement provisional, SLAM-09/IL-04/INT-10 awaiting audit). Keep it factual and under the size limit.
- [ ] **Step 4: Change `ROLES`:** `supervisor` loads `CLAUDE.md`, `.claude/supervisor.md`, `agents-doc/STATE_DIGEST.md`, `agents-doc/TICKETS.md` (the index); `coder` loads `AGENTS.md`, its role rule file, and one ticket file (use `agents-doc/tickets/TK-SLAM-09.md` as the representative). Update `supervisor.md` §7 start: read `STATE_DIGEST.md` instead of `tail -40 agents-doc/STATE.md`, and add "regenerate the digest at the session end" to the End step.
- [ ] **Step 5: Run `.venv/bin/python -m unittest discover tests/harness -v` and `.venv/bin/python scripts/context_cost.py`.** Expected: all pass; supervisor <= 6000, coder <= 40 % of baseline. If supervisor is over, shorten `supervisor.md` §6 snapshot (the digest supersedes it) and rerun.
- [ ] **Step 6: Commit** `agents-doc/STATE_DIGEST.md scripts/context_cost.py tests/harness/test_state_digest.py .claude/supervisor.md CLAUDE.md` with `feat(harness): state digest, phase-1 token gate`.

---

### Task 4: Hard guards in `.claude/settings.json` (phase 2)

**Files:**
- Modify: `.claude/settings.json`
- Create: `tests/harness/test_settings_rules.py`

**Interfaces:**
- Produces: `permissions.deny` and `permissions.ask` lists. The existing two `robot_dynamics.json` deny entries stay.

Plan-time correction to the spec (my call): the spec says "deny edits to LOCKED_PATHS entries", but the Supervisor (this Claude session) must edit `tests/acceptance/`, `CLAUDE.md` and `.claude/`. So: `deny` only for `setup/robot_dynamics.json` (already) and the destructive git commands; `ask` for edits to the other locked paths.

- [ ] **Step 1: Confirm the current rule syntax** for Bash prefix patterns and Edit path patterns (Claude Code docs via `mcp__7efe1c4f-c213-4894-8eb0-0a42b058cfae__query-docs`, or the `claude-code-guide` agent) and write the exact strings you will use at the top of the test file as constants.
- [ ] **Step 2: Write the failing test.** `test_deny_has_destructive_git` (patterns for `git add -u`, `git add -A`, `git add .`, `git push`, `git reset --hard` all present in `deny`); `test_ask_has_jetson_access` (patterns for `ssh`, `scp`, `rsync` to the Jetson host `jetbot@192.168.1.18` present in `ask`); `test_ask_has_locked_paths` (an Edit/Write pattern for each of `tests/acceptance/`, `AGENTS.md`, `.agents/`, `CLAUDE.md`, `.claude/`, `agents-doc/LOCKED_PATHS.txt`); `test_robot_dynamics_still_denied`; `test_json_valid`.
- [ ] **Step 3: Run, expect failure.**
- [ ] **Step 4: Edit `.claude/settings.json`** to add the entries.
- [ ] **Step 5: Run the test, expect pass. Then the live check (human present):** in this Claude session try each of: `git add -u --dry-run`, `git push --dry-run`, an `ssh jetbot@192.168.1.18 true`, an Edit of `CLAUDE.md` (revert it). Expected: the first two are denied outright, the last two prompt. Record the observed result per rule in the commit message.
- [ ] **Step 6: Commit** `.claude/settings.json tests/harness/test_settings_rules.py` with `feat(harness): deny destructive git, ask on Jetson and locked paths`.

---

### Task 5: `check_ticket.py` mechanical checks (phase 3)

**Files:**
- Create: `scripts/check_ticket.py`, `tests/harness/test_check_ticket.py`
- Modify: `.claude/agents/quality-auditor.md` (checks 1, 4, 5, 7 become "run `check_ticket.py` and read its output"), `.claude/commands/audit.md`, `agents-doc/LOCKED_PATHS.txt` (add `^scripts/check_ticket\.py$`)
- Delete (needs the human's nod): `scripts/verify_agent_integrity.py` and its line in `LOCKED_PATHS.txt`
- Consumes: `parse_tickets`, `field`, `backticked` from Task 1.

**Interfaces** (all in `scripts/check_ticket.py`; each check returns a list of finding strings, empty = pass):
- `changed_paths(root: Path) -> list[str]` (`git status --porcelain`: modified, deleted, renamed targets and untracked files).
- `in_scope(path: str, allowed: list[str]) -> bool` (an allowed token that ends with `/` or `**` matches by prefix; otherwise exact).
- `check_scope(changed: list[str], allowed: list[str], locked_patterns: list[str]) -> list[str]` (one finding per out-of-scope path, one per `LOCKED_PATHS.txt` regex hit).
- `check_diff_hygiene(diff: str) -> list[str]`: findings for a removed `assert`/`self.assert*` line without a replacement, an added `skip`/`@unittest.skip`/`pytest.mark.skip`, an added `except...:` followed by `pass`, a changed numeric literal on a line containing `speed|pwm|turn|trim|omega|velocity` outside `setup/robot_dynamics.json`, and any added path containing `0[1-7]-` or `archive/`.
- `run_commands(ticket: Ticket, root: Path) -> list[str]`: runs the backticked commands under `Acceptance command` and `Regression commands` via `subprocess` with `.venv/bin/python`; findings = non-zero exit; always echoes the last metric-looking lines (`[METRIC]`, `Ran N tests`, `OK`/`FAILED`) so the auditor sees numbers, not a banner. A ticket with no acceptance command returns `["no acceptance command"]`.
- CLI: `check_ticket.py TK-ID [--only scope|hygiene|commands] [--ready]`; prints one `PASS|FAIL <check>: <detail>` line per finding or check; exit 1 if any FAIL.

- [ ] **Step 1: Write failing tests** with synthetic inputs (no git needed except one temp repo): `test_in_scope_exact_dir_glob`; `test_scope_flags_untracked_and_deleted` (temp git repo, one untracked file outside allowed + one deleted file); `test_locked_path_hit`; `test_hygiene_removed_assert`, `test_hygiene_added_skip`, `test_hygiene_except_pass`, `test_hygiene_physics_literal`, `test_hygiene_dead_folder_path`; `test_no_acceptance_command_does_not_crash`; `test_commands_report_nonzero_exit` (ticket whose acceptance command is `.venv/bin/python -c "import sys; sys.exit(1)"`).
- [ ] **Step 2: Run, expect failure.**
- [ ] **Step 3: Implement** the functions above and the CLI.
- [ ] **Step 4: Known-result checks.** Run `.venv/bin/python scripts/check_ticket.py TK-SLAM-09 --only commands`. Expected: `Ran 13 tests ... OK`, exit 0. Run `... TK-INT-10 --only scope`. Expected: FAIL lines listing the unrelated uncommitted files (correct: the tree holds coder WIP); none of the three INT-10 allowed paths flagged. Plant one out-of-scope edit in a temp copy of the repo and confirm FAIL, then discard the copy.
- [ ] **Step 5: Wire the auditor and the command.** In `quality-auditor.md` replace the hand-run steps for scope, test integrity, physics literals and paths with "run `.venv/bin/python scripts/check_ticket.py <ID>` and quote its output; keep checks 2/3 (re-run, read metrics), 6 (units and frames), 8 (boot) and 9 (citations) as judgment". Add the script to `LOCKED_PATHS.txt`.
- [ ] **Step 6: With the human's nod,** `git rm scripts/verify_agent_integrity.py` and delete its `LOCKED_PATHS.txt` line. Without a nod, leave both and note it.
- [ ] **Step 7: Commit** the script, test, `quality-auditor.md`, `audit.md`, `LOCKED_PATHS.txt` (and the deletion if approved) with `feat(harness): check_ticket.py mechanical audit checks`.

---

### Task 6: HITL gate (phase 4)

**Files:**
- Modify: `scripts/check_ticket.py` (`--ready`), `agents-doc/TICKETS.md` (template), `AGENTS.md` (coder protocol), `.claude/commands/ticket.md`, `tests/harness/test_check_ticket.py`
- Create: `agents-doc/needs-human/README.md`

**Interfaces:**
- Template fields added to the ticket template: `- Assumptions:` (what the Supervisor filled in) and `- Needs-human:` (open questions); READY requires a line `Human-approved: YYYY-MM-DD` inside `Assumptions`.
- `check_ready(ticket: Ticket, needs_human_dir: Path) -> list[str]` in `check_ticket.py`: findings = `"no Assumptions field"` is NOT a failure for tickets created before 2026-11-16 (returns `["legacy ticket: no Assumptions (warning)"]` flagged WARN, exit 0); for a ticket with `Assumptions` but no `Human-approved:` line, `"assumptions not approved by the human"`; for any open file `<needs_human_dir>/<ID>.md` without a `RESOLVED:` line, `"open needs-human question"`.
- Needs-human file format (documented in the README): `Question:`, `Tried:`, `Proposed default:`, optional `RESOLVED: <answer>`.

- [ ] **Step 1: Write failing tests:** `test_ready_requires_human_approval`, `test_ready_passes_when_approved`, `test_open_needs_human_blocks`, `test_resolved_needs_human_passes`, `test_legacy_ticket_warns_not_fails` (use IL-04's real ticket text).
- [ ] **Step 2: Run, expect failure.**
- [ ] **Step 3: Implement `check_ready` and `--ready`.**
- [ ] **Step 4: Update docs.** Template gets the two fields. `AGENTS.md` gets one rule: "If the ticket does not give you what you need to continue, STOP and write `agents-doc/needs-human/<ID>.md`; do not guess, do not widen scope." `ticket.md` (the `/ticket` command) gets: before setting READY, list the Assumptions to the human and wait for approval, then write `Human-approved: <date>`.
- [ ] **Step 5: Run all harness tests.** Run: `.venv/bin/python -m unittest discover tests/harness -v`. Expected: all pass.
- [ ] **Step 6 (human-gated acceptance for phase 4):** the Supervisor writes a throwaway ticket `TK-TEST-01` with an Assumption the coder cannot verify, the human hands it to one Antigravity coder, and the coder must stop with a needs-human file instead of guessing. Record the result in `STATE.md`; delete the throwaway ticket.
- [ ] **Step 7: Commit** the touched files with `feat(harness): human-in-the-loop gate for incomplete context`.

---

## Self-review (run against the spec)

- Spec phase 1: parser + meter (Task 1), per-ticket split and index (Task 2), digest and the numeric gate (Task 3), root handoff left alone (Global Constraints). Phase 2: Task 4, with the deny-vs-ask correction stated. Phase 3: Task 5 including the auditor rewiring and the `verify_agent_integrity.py` replacement. Phase 4: Task 6 including the coder-stop acceptance.
- Spec's "reproduce MEAS-01 attempt 1" known-result is replaced by `TK-SLAM-09` commands plus a planted out-of-scope edit, because attempt 1's tree no longer exists. Flagged here rather than silently dropped.
- Names are consistent across tasks: `parse_tickets`, `field`, `backticked`, `tokens`, `ROLES`, `check_scope`, `check_diff_hygiene`, `run_commands`, `check_ready`.
- Proportion: six tasks, no function bodies except the signatures and rules the spec fixes.
