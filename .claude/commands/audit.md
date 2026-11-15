---
description: Kiểm tra độc lập REPORT của Antigravity cho một ticket (gọi quality-auditor)
argument-hint: <TK-ID> rồi dán khối ## REPORT
---

Bạn là Supervisor CS532. Yêu cầu audit: $ARGUMENTS

1. Ghi lại cho auditor những gì CHỈ bạn biết: file nào người dùng hoặc Supervisor tự sửa (không tính vào scope coder),
   quyết định của người dùng liên quan, lỗi biết trước nằm ngoài ticket.
2. Gọi subagent `quality-auditor` (nếu không có trong danh sách agent, dùng `general-purpose` và bảo nó
   đọc rồi làm đúng `.claude/agents/quality-auditor.md`). Truyền: TK-ID, REPORT của coder, ghi chú bước 1.
3. Khi có kết quả: tự chạy lại ÍT NHẤT lệnh nghiệm thu chính để đối chiếu con số với auditor.
4. Trả lời người dùng (tiếng Việt):
   - **PASS** → tóm tắt bằng chứng, việc nhỏ còn lại, và bảo người dùng gõ `/close <TK-ID>`.
   - **REJECT** → liệt kê Blocking findings, và đưa sẵn prompt "lần 2" cho Antigravity
     (mẫu `agents-doc/PLAYBOOK.md` §3.4). Nếu là bịa/fake pass, thêm dòng vào HALLUCINATION LEDGER
     trong `agents-doc/STATE.md` (chỉ ghi thêm).
Không commit trong lệnh này.
