# BIỂU MẪU & QUY TRÌNH TÁC NGHIỆP AI HARNESS (TEMPLATES & PROTOCOLS)

*Tài liệu này chứa các biểu mẫu chuẩn hóa để con người và các AI Agent trao đổi thông tin, bàn giao module, và báo cáo lỗi mà không bị mất ngữ cảnh.*

---

## 1. Biểu Mẫu Báo Lỗi / Điểm Nghẽn (Bug / Blocker Report Template)
*Dùng khi một thành viên hoặc một agent phát hiện lỗi trong module của agent khác:*

```markdown
### 🐛 [MÃ LỖI]: [Tên ngắn gọn của lỗi]
- **Module bị lỗi:** [setup / perception / planning / web]
- **Agent chịu trách nhiệm:** [Tên agent phụ trách]
- **Mức độ nghiêm trọng:** [P0 - Chặn hoàn toàn / P1 - Lỗi lớn / P2 - Lỗi nhỏ / P3 - Cải thiện]
- **Môi trường:** [Jetson Nano thật / Laptop mô phỏng]

#### 1. Mô tả Hiện tượng (Symptoms)
[Mô tả những gì xảy ra trên thực tế: vệt đường đi bị méo, web bị đơ, frame drop...]

#### 2. Hành vi Kỳ vọng (Expected Behavior)
[Hệ thống đúng phải hoạt động như thế nào?]

#### 3. Mã nguồn / Đoạn Log liên quan
\`\`\`python
# Paste đoạn code hoặc log lỗi vào đây
\`\`\`

#### 4. Kịch bản Tái hiện (Reproduction Steps)
1. Bước 1...
2. Bước 2...
3. Quan sát thấy lỗi xuất hiện.

#### 5. Đề xuất Sơ bộ từ Người phát hiện (Hint)
[Gợi ý nguyên nhân nếu có, ví dụ: kiểm tra lại hệ trục tọa độ X/Y hoặc chia cho 0]
```

---

## 2. Biểu Mẫu Bàn Giao Module (Module Handoff Protocol)
*Dùng khi một Agent hoàn thành pilot của mình và muốn bàn giao cho `integration-reviewer-agent` hoặc module kế tiếp:*

```markdown
### 📦 BIÊN BẢN BÀN GIAO MODULE: [Tên Module]
- **Agent bàn giao:** [Tên agent]
- **Nhánh Git (Branch):** [tên branch, VD: planning/feature-astar]
- **Ngày bàn giao:** [YYYY-MM-DD]

#### 1. Các Tính năng Đã Hoàn Thành
- [x] Tính năng 1...
- [x] Tính năng 2...

#### 2. Đối chiếu Hợp đồng Giao diện (INTERFACE.md Compliance)
- [x] Định dạng input nhận vào: Đúng 100% theo INTERFACE.md Mục [X]
- [x] Định dạng output phát ra: Đúng 100% theo INTERFACE.md Mục [Y]
- [x] Tần số truyền tin thực tế đo được: [...] Hz

#### 3. Bằng chứng Kiểm thử (Verification Proof)
- **Unit Test Result:** [Số test pass / tổng số test, VD: 5/5 PASSED]
- **Thời gian thực thi trung bình (Latency):** [...] ms trên Jetson Nano
- **Ảnh chụp màn hình / Log kết quả:** [Đính kèm nếu có]

#### 4. Các Giới hạn Chưa Giải Quyết (Known Limitations)
[Ghi rõ những trường hợp biên chưa xử lý, ví dụ: chưa test khi góc quay xe $> 180^\circ$]

#### 5. Kiến thức Thành viên Cần Nắm để Vấn đáp
- **Thuật toán chính sử dụng:** [Tên thuật toán & lý do chọn]
- **Tham số cốt lõi đã điều chỉnh:** [Ví dụ: hệ số Kp, Ki, Kd hoặc bán kính lạm phát inflation_radius]
```

---

## 3. Biểu Mẫu Quy ước Commit (AI-Assisted Commit Convention)
*Bắt buộc áp dụng cho mọi commit theo quy định của CS532 WORKFLOW:*

```
[type]([scope]): [Mô tả ngắn gọn về thay đổi]

AI-assisted: [Tên Agent đã tạo/sửa phần này, VD: planning-control-agent]
Rationale: [Lý do kỹ thuật ngắn gọn vì sao chọn giải pháp này]

Các file thay đổi:
- [path/to/file1]: [Mô tả sửa đổi]
```

*Ví dụ thực tế:*
```
fix(planning): sửa lỗi cắt góc chéo trong thuật toán A* trên ma trận lưới

AI-assisted: planning-control-agent
Rationale: Bổ sung điều kiện kiểm tra ô 8 hướng không cho phép đi chéo nếu 2 ô trực giao kề cạnh là vật cản, ngăn robot va quẹt mép tường.

Các file thay đổi:
- planning/astar.py: Thêm hàm is_valid_diagonal_move()
- planning/test_astar.py: Bổ sung test case CornerCuttingCase
```
