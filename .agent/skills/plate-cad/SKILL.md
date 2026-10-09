---
name: plate-cad
description: Parametric design of sensor mounting plates, LiDAR brackets, and robotic reinforcement plates using CadQuery and build123d. Covers precision hole patterns (regular polygons, equilateral triangles, grids), edge offsets, cable routing cutouts, drop-down L-brackets, fastener clearances (M2, M2.5, M3), FDM supportless printing guidelines (45-degree rule), and fabrication export to STEP, STL, and DXF.
license: MIT
compatibility: Python 3.10-3.12 with CadQuery 2.8+ or build123d 0.12+ and trimesh.
---

# Plate CAD — Parametric Sensor & Robot Mounting Plates

Parametric Python CAD workflow for precision mounting plates, sensor brackets, and robotics reinforcement plates.

---

## 1. Core Engineering Principles

### BRep-First Modeling
- Never use brute-force CSG Boolean operations when native feature operations (`faces().hole()`, `workplane().rect().extrude()`) suffice.
- Model geometry relative to well-defined datum planes (XY workplane at top $Z=0$ or base $Z=0$).

### Fastener & Hole Standards (ISO / DIN)
Always size holes according to mechanical function:
- **M2.0**: Clearance $\varnothing 2.2\text{mm} - 2.4\text{mm}$, Tap drill $\varnothing 1.6\text{mm}$, Heat insert $\varnothing 3.2\text{mm}$.
- **M2.5**: Clearance $\varnothing 2.7\text{mm} - 2.9\text{mm}$, Tap drill $\varnothing 2.05\text{mm}$, Heat insert $\varnothing 3.8\text{mm} - 4.0\text{mm}$.
- **M3.0**: Clearance $\varnothing 3.2\text{mm} - 3.4\text{mm}$, Tap drill $\varnothing 2.5\text{mm}$, Heat insert $\varnothing 4.2\text{mm} - 4.6\text{mm}$.
- **Edge Margin**: Minimum distance from hole center to any plate edge: $E_{\text{min}} \ge 1.5 \times d_{\text{hole}}$ (e.g. $\ge 4.5\text{mm}$ for M2.5).

### FDM 3D Printing Rules
- **Minimum Plate Thickness**: $\ge 2.5\text{mm}$ for light sensor mounts, $\ge 3.5\text{mm}$ for vibrating/rotating sensors (LiDAR).
- **The $45^\circ$ Overhang Rule**: All drop-down brackets, shelves, and undercuts must feature a $45^\circ$ chamfer or gusset to print reliably without support structures.
- **Corner Fillets vs Sharp Edges**: External corners can be sharp ($90^\circ$) if required by mating components, but internal cutouts (cable slots) must have corner radius $R \ge 1.5\text{mm}$ to prevent stress cracks and nozzle dragging.

---

## 2. Standard Sensor Geometries

### LDROBOT STL-19P / D500 / LD19 LiDAR
- **Mounting interface**: 3x M2.5 holes (ear counterbore $\varnothing 4.46\text{mm}$ depth $2\text{mm}$).
- **Ear coordinates (Forward arrow pointing +Y / 12 o'clock)**:
  - Single Left Ear: $(X = -27.0\text{mm}, Y = 0.0\text{mm})$ relative to sensor center.
  - Top-Right Ear: $(X = +4.93\text{mm}, Y = +23.40\text{mm})$.
  - Bottom-Right Ear: $(X = +4.93\text{mm}, Y = -23.40\text{mm})$.
  - Distance between side ears: $46.80\text{mm}$.
  - Distance from side ear line to single ear: $31.92\text{mm}$.
  - Cable connector: JST ZH1.5-4P faces $+X$ (right side).

---

## 3. Idiomatic CadQuery Implementation Template

```python
import cadquery as cq
import trimesh

# 1. Base Plate
plate = (
    cq.Workplane("XY")
    .box(width, length, thickness, centered=(False, False, False))
)

# 2. Corner Holes
corner_pts = [
    (x_left, y_bottom),
    (x_right, y_bottom),
    (x_left, y_top),
    (x_right, y_top),
]
plate = plate.faces(">Z").workplane().pushPoints(corner_pts).hole(dia_corner)

# 3. Sensor Holes
sensor_pts = [(sx1, sy1), (sx2, sy2), (sx3, sy3)]
plate = plate.faces(">Z").workplane().pushPoints(sensor_pts).hole(dia_sensor)

# 4. Cable Slot
plate = (
    plate.faces(">Z").workplane()
    .center(slot_x, slot_y)
    .rect(slot_w, slot_h)
    .cutThruAll()
)

# 5. Export
cq.exporters.export(plate, "output/part.step")
cq.exporters.export(plate, "output/part.stl", angularTolerance=0.1, linearTolerance=0.02)
```

---

## 4. Verification Protocol
After exporting, always verify with `trimesh`:
1. `mesh.is_watertight == True` (Manifold solid, ready for slicing).
2. `mesh.volume > 0` (Non-empty solid geometry).
3. Bounding box matches specifications within $0.05\text{mm}$.
