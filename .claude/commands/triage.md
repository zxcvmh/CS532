---
description: Phân loại một vấn đề mới và nói nên giao cho ai (role Antigravity, Claude, hay người dùng)
argument-hint: <mô tả triệu chứng, file, log>
---

Bạn là Supervisor CS532 (xem CLAUDE.md). Người dùng điều khiển; bạn chỉ phân loại, KHÔNG sửa code.

Vấn đề: $ARGUMENTS

Làm theo thứ tự:
1. Thu thập sự thật tối thiểu để định vị (đọc file:line, grep, log trên Jetson `jetbot@192.168.1.18` nếu người dùng đưa đường dẫn log; chỉ đọc). Không đoán.
2. Đối chiếu bảng định tuyến ở `agents-doc/PLAYBOOK.md` §2 và quyền sở hữu file trong `.agents/workflows/*.md`.
3. Kiểm tra `agents-doc/TICKETS.md`: vấn đề đã có ticket chưa? Nếu có, nói ticket nào và có cần bổ sung không.
4. Trả lời ngắn bằng tiếng Việt:
   - **Nguyên nhân khả dĩ** (kèm bằng chứng file:line, ghi rõ "chưa kiểm chứng" nếu chưa chắc)
   - **Giao cho:** `/role` | Claude | người dùng — và vì sao
   - **Ticket:** mới (đề xuất ID + tiêu đề) hoặc ticket cũ
   - **Bước tiếp theo** cho người dùng: đúng một lệnh/prompt copy được (ví dụ `/ticket TK-XXX ...`)
