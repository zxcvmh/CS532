# CS532 JetBot — Agent Harness v2 (2026-11-15)

## Ai làm gì

| Role | Chạy trên | Định nghĩa | Được sửa |
|---|---|---|---|
| 👑 Supervisor | Claude (phiên chính) | [`CLAUDE.md`](../CLAUDE.md) | tickets, locked tests, commit, **phần cứng/SSH** |
| 🛡️ Quality Auditor | Claude subagent | [`.claude/agents/quality-auditor.md`](../.claude/agents/quality-auditor.md) | không sửa code — chỉ chạy lại & phán PASS/REJECT |
| 🔬 Research Assistant | Antigravity `/researcher` | [`.agents/workflows/researcher.md`](../.agents/workflows/researcher.md) | `docs/research/` |
| 👁️ Perception | Antigravity `/perception` | [`.agents/workflows/perception.md`](../.agents/workflows/perception.md) | `perception/`, `dashboard/floor_ipm.py`, `dashboard/backend/app/cv/` |
| 🗺️ SLAM & Mapping | Antigravity `/slam` | [`.agents/workflows/slam.md`](../.agents/workflows/slam.md) | `motion/planning/slam/`, `cartographer/`, `cartographer_adapter.py` |
| 🤖 IL Policy | Antigravity `/il-policy` | [`.agents/workflows/il-policy.md`](../.agents/workflows/il-policy.md) | `imitation_learning/`, `il_onnx_bridge.py` |
| 🔌 Integration & Edge | Antigravity `/integration` | [`.agents/workflows/integration.md`](../.agents/workflows/integration.md) | phần còn lại của `dashboard/backend/`, `jetbot_bridge.py`, motor/CBF |

Luật chung cho Antigravity: [`AGENTS.md`](../AGENTS.md) (luôn nạp) + [`.agents/rules/`](../.agents/rules/) (nạp theo thư mục).

## Vòng làm việc

```
Supervisor viết ticket + locked test (tests/acceptance/)  →  READY
   → bạn chạy /<role> trong Antigravity  →  Flash code + REPORT (output thô)
   → Supervisor gọi quality-auditor  →  tự chạy lại, soi diff
   → PASS: commit + DONE      REJECT: ghi ledger, trả lại
```

## File

- [`TICKETS.md`](TICKETS.md) — việc đang làm, template ticket.
- [`STATE.md`](STATE.md) — trạng thái + HALLUCINATION LEDGER (chỉ ghi thêm).
- [`LOCKED_PATHS.txt`](LOCKED_PATHS.txt) — đường dẫn agent không được đụng (auditor kiểm bằng `grep -Ef`).
- [`../setup/robot_dynamics.json`](../setup/robot_dynamics.json) — số đo thật của bạn, read-only.
- `archive/agents-v1/` — 4 agent cũ + tickets cũ (đường dẫn trong đó đã chết).
