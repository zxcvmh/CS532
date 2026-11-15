---
description: Viết/hoàn thiện một ticket kèm locked acceptance test và chuyển sang READY
argument-hint: <TK-ID hoặc mô tả vấn đề>
---

Bạn là Supervisor CS532 (xem CLAUDE.md). Yêu cầu: $ARGUMENTS

1. Mở `agents-doc/TICKETS.md`. Nếu ticket đã có, dùng nó; nếu chưa, tạo theo Template trong file đó.
2. Điền đủ: Role (đúng chủ sở hữu file theo `.agents/workflows/`), Problem (bằng chứng file:line hoặc output lệnh),
   **Allowed files** (≤ 2–3, gồm cả file config phụ nếu bắt buộc), Acceptance command, **Threshold bằng số**,
   Regression commands, Depends on.
3. Viết hoặc cập nhật **locked test** trong `tests/acceptance/` đo chỉ số thật (success %, cm, độ, ms),
   không phải "chạy không lỗi". Mọi hằng số vật lý đọc từ `setup/robot_dynamics.json`.
4. Chạy test: nó phải **FAIL đúng lý do** của ticket. Dán output.
5. Chỉ khi bước 4 đúng: đổi Status → `READY`.
6. Kết thúc bằng prompt copy được cho người dùng dán vào Antigravity (mẫu `agents-doc/PLAYBOOK.md` §3.3).
Không commit ở bước này trừ khi người dùng bảo.
