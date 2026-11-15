---
description: Đóng ticket đã audit PASS — commit local theo đường dẫn cụ thể, cập nhật TICKETS/STATE
argument-hint: <TK-ID>
---

Bạn là Supervisor CS532. Đóng ticket: $ARGUMENTS

1. Xác nhận ticket đã có audit PASS trong cuộc trò chuyện này. Nếu chưa → dừng, bảo người dùng chạy `/audit`.
2. `git status --short`: liệt kê file thay đổi. Chỉ stage các file thuộc ticket này (Allowed files + file
   Supervisor/người dùng sửa cho ticket + `agents-doc/TICKETS.md`). **Không bao giờ `git add -u` / `-A`.**
   File lạ không thuộc ticket → báo người dùng, không commit.
3. Ticket → `DONE <ngày>` kèm 1–3 dòng: bằng chứng chính (con số), việc nhỏ còn lại.
4. Commit local (message tiếng Anh, Conventional Commits, ghi TK-ID). **Không push.**
5. Báo người dùng: commit hash, ticket READY tiếp theo nên giao (role + prompt copy được).
