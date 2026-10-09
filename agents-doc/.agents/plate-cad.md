---
name: plate-cad
description: Thiết kế tấm phẳng (plate/bracket) có lỗ bắt vít M2.5 bằng CadQuery, tham số hóa, xuất STEP và STL. Dùng khi người dùng yêu cầu vẽ/chỉnh file CAD cho tấm gắn ốc.
---

# Quy trình
1. Luôn hỏi đủ thông số trước khi vẽ; không tự đoán kích thước.
2. Viết script Python tham số hóa tại `cad/plate.py` (CadQuery).
3. Chạy script, xuất `output/plate.step` và `output/plate.stl`.
4. Kiểm tra: số lỗ = 7, lỗ không chạm mép, khoảng cách tới mép >= 1.5 x đường kính lỗ.
5. Báo lại cho người dùng bảng thông số đã dùng.

# Đường kính lỗ M2.5 tham khảo
- Lỗ thông (clearance): 2.7 – 2.9 mm
- Bắt ren trực tiếp vào nhựa in 3D: 2.1 – 2.2 mm
- Heat insert M2.5: thường 3.5 – 4.0 mm (theo datasheet insert)

# Template script
```python
import math
import cadquery as cq

# ---- THÔNG SỐ (mm) ----
L, W, T = 100.0, 60.0, 3.0      # dài, rộng, dày
corner_off = 4.0                # từ mép tới tâm lỗ góc
hole_d = 2.7                    # đường kính lỗ
tri_side = 20.0                 # cạnh tam giác đều ở giữa
corner_fillet = 0.0             # bo góc tấm (0 = không bo)

plate = cq.Workplane("XY").box(L, W, T)
if corner_fillet > 0:
    plate = plate.edges("|Z").fillet(corner_fillet)

corner_pts = [(sx * (L/2 - corner_off), sy * (W/2 - corner_off))
              for sx in (-1, 1) for sy in (-1, 1)]

R = tri_side / math.sqrt(3)     # bán kính ngoại tiếp
tri_pts = [(R * math.cos(math.radians(a)), R * math.sin(math.radians(a)))
           for a in (90, 210, 330)]

plate = (plate.faces(">Z").workplane()
         .pushPoints(corner_pts + tri_pts)
         .hole(hole_d))

import os
os.makedirs("output", exist_ok=True)
cq.exporters.export(plate, "output/plate.step")
cq.exporters.export(plate, "output/plate.stl")
```