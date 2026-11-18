#!/usr/bin/env python3
"""
Parametric LiDAR D500 Mounting Plate Generator (Version 4)
Built with build123d / Open CASCADE according to plate-cad, cadquery, and cad-khana skills.

Design Specifications & Mechanical Updates:
- Plate thickness: 2.0mm (optimal for lightweight and easy M2.5 screw tightening/removal)
- Drop-down L-Bracket:
  * Thickness increased to 3.5mm (WALL_THICK = 3.5mm, SHELF_THICKNESS = 3.5mm) to eliminate snapping risk
  * Turned INWARD into plate underbody (Y in [1.0, 26.5mm], X in [30.0, 50.0mm])
  * PCB bed width: 20.5mm clear space (fits 20.0mm USB-UART board with tolerance)
  * Retention lip: 1.5mm height, 1.5mm thickness at inner edge (Y in [25.0, 26.5mm])
  * Dual 45° reinforcing gussets:
    - 2.5mm gusset between plate underbody and hanger wall
    - 1.2mm stress-relief fillet/gusset at inside L corner
- 4x M2.5 Corner Clearance Holes (Dia = 2.8mm, horizontal pitch = 58.0mm)
- 3x M2.5 LiDAR D500 Holes (Dia = 2.7mm, LDROBOT STL-19P standard pattern)
- Cable Pass-Through Slot (13x8mm, R=2mm at X=68.0, Y=40.0mm)
"""

import os
import base64
import build123d as bd
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------
# 1. PARAMETERS DEFINITION (in mm)
# ---------------------------------------------------------
# Main Plate Envelope
PLATE_WIDTH = 80.0       # X dimension
PLATE_LENGTH = 100.0     # Y dimension
PLATE_THICKNESS = 2.0    # Z dimension (Updated to 2.0mm as requested)

# Corner Holes (4x M2.5 Clearance, Dia = 2.8mm)
CORNER_HOLE_DIA = 2.8
CORNER_HOLES = [
    (17.0, 10.0),   # Bottom-Left (17mm from left edge, 10mm from bottom)
    (75.0, 10.0),   # Bottom-Right (58mm pitch from left hole, 5mm from right edge)
    (17.0, 95.0),   # Top-Left (5mm from top)
    (75.0, 95.0),   # Top-Right (5mm from top)
]

# LiDAR D500 (STL-19P) Mounting Holes (3x M2.5, Dia = 2.7mm)
# Center of LiDAR at (40.0, 50.0), forward arrow pointing +Y
LIDAR_HOLE_DIA = 2.7
LIDAR_HOLES = [
    (13.00, 50.00),  # Single Left Ear
    (44.93, 73.40),  # Top-Right Ear (pitch = 46.80mm between two right ears)
    (44.93, 26.60),  # Bottom-Right Ear
]

# Cable Pass-through Slot (At X=68.0, Y=40.0)
SLOT_CENTER_X = 68.0
SLOT_CENTER_Y = 40.0
SLOT_WIDTH = 13.0    # along X
SLOT_HEIGHT = 8.0    # along Y
SLOT_FILLET_R = 2.0

# Inward-facing Drop-down L-Bracket for Signal Board (3.5mm reinforced thickness)
SHELF_X_MIN = 30.0
SHELF_X_MAX = 50.0
SHELF_LENGTH = SHELF_X_MAX - SHELF_X_MIN  # 20.0mm (along X)

SHELF_Y_START = 1.0                       # Hanger wall starts at Y = 1.0mm
WALL_THICK = 3.5                          # Vertical wall thickness: 3.5mm (Y: 1.0 to 4.5mm)
PCB_CLEAR_WIDTH = 20.5                    # Clear bed for 20mm wide PCB (Y: 4.5 to 25.0mm)
LIP_THICK = 1.5                           # Retention lip thickness (Y: 25.0 to 26.5mm)
LIP_HEIGHT = 1.5                          # Retention lip height (Z: -10.0 to -8.5mm)
SHELF_DEPTH = PCB_CLEAR_WIDTH + LIP_THICK # Total shelf floor depth = 22.0mm

SHELF_THICKNESS = 3.5                     # Floor thickness: 3.5mm (Z: -13.5 to -10.0mm)
DROP_TOTAL = 10.0                         # 10mm drop from top face (Z = 0) to shelf surface (Z = -10.0)

# ---------------------------------------------------------
# 2. MODEL GENERATION (Open CASCADE B-Rep)
# ---------------------------------------------------------
def create_model():
    with bd.BuildPart() as part_builder:
        # 1. Base Plate: X in [0, 80], Y in [0, 100], Z in [-2.0, 0] (top face at Z = 0)
        bd.Box(
            PLATE_WIDTH, 
            PLATE_LENGTH, 
            PLATE_THICKNESS, 
            align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MAX)
        )

        # 2. 4 Corner Holes
        with bd.Locations(CORNER_HOLES):
            bd.Hole(radius=CORNER_HOLE_DIA / 2.0, depth=PLATE_THICKNESS)

        # 3. 3 LiDAR Mounting Holes
        with bd.Locations(LIDAR_HOLES):
            bd.Hole(radius=LIDAR_HOLE_DIA / 2.0, depth=PLATE_THICKNESS)

        # 4. Cable Pass-Through Slot with Fillet
        with bd.BuildSketch(bd.Plane.XY.offset(1.0)):
            with bd.Locations([(SLOT_CENTER_X, SLOT_CENTER_Y)]):
                bd.RectangleRounded(SLOT_WIDTH, SLOT_HEIGHT, radius=SLOT_FILLET_R)
        bd.extrude(amount=-(PLATE_THICKNESS + 2.0), mode=bd.Mode.SUBTRACT)

        # 5. Inward-facing Drop-down L-Bracket:
        # 5a. Hanger Wall: X in [30, 50], Y in [1.0, 4.5], Z in [-13.5, -2.0]
        wall_height = DROP_TOTAL + SHELF_THICKNESS - PLATE_THICKNESS # 10.0 + 3.5 - 2.0 = 11.5mm
        with bd.Locations([(SHELF_X_MIN, SHELF_Y_START, -DROP_TOTAL - SHELF_THICKNESS)]):
            bd.Box(
                SHELF_LENGTH, 
                WALL_THICK, 
                wall_height, 
                align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN), 
                mode=bd.Mode.ADD
            )

        # 5b. Sàn đỡ mạch (quay vào trong lòng gầm): X in [30, 50], Y in [4.5, 26.5], Z in [-13.5, -10.0]
        with bd.Locations([(SHELF_X_MIN, SHELF_Y_START + WALL_THICK, -DROP_TOTAL - SHELF_THICKNESS)]):
            bd.Box(
                SHELF_LENGTH, 
                SHELF_DEPTH, 
                SHELF_THICKNESS, 
                align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN), 
                mode=bd.Mode.ADD
            )

        # 5c. Gờ kẹp chống tuột ở mép trong: X in [30, 50], Y in [25.0, 26.5], Z in [-10.0, -8.5]
        with bd.Locations([(SHELF_X_MIN, SHELF_Y_START + WALL_THICK + PCB_CLEAR_WIDTH, -DROP_TOTAL)]):
            bd.Box(
                SHELF_LENGTH, 
                LIP_THICK, 
                LIP_HEIGHT, 
                align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN), 
                mode=bd.Mode.ADD
            )

        # 5d. Gân trợ lực 45° tại giao tuyến trên (thành đứng - gầm tấm):
        gusset_top = 2.5
        with bd.BuildSketch(bd.Plane.YZ.offset(SHELF_X_MIN)):
            with bd.BuildLine():
                p1 = (SHELF_Y_START + WALL_THICK, -PLATE_THICKNESS)
                p2 = (SHELF_Y_START + WALL_THICK + gusset_top, -PLATE_THICKNESS)
                p3 = (SHELF_Y_START + WALL_THICK, -PLATE_THICKNESS - gusset_top)
                bd.Line(p1, p2)
                bd.Line(p2, p3)
                bd.Line(p3, p1)
            bd.make_face()
        bd.extrude(amount=SHELF_LENGTH, mode=bd.Mode.ADD)

        # 5e. Gân trợ lực 45° tại góc trong chữ L (thành đứng - mặt sàn đỡ):
        gusset_bot = 1.2
        with bd.BuildSketch(bd.Plane.YZ.offset(SHELF_X_MIN)):
            with bd.BuildLine():
                p1 = (SHELF_Y_START + WALL_THICK, -DROP_TOTAL)
                p2 = (SHELF_Y_START + WALL_THICK + gusset_bot, -DROP_TOTAL)
                p3 = (SHELF_Y_START + WALL_THICK, -DROP_TOTAL + gusset_bot)
                bd.Line(p1, p2)
                bd.Line(p2, p3)
                bd.Line(p3, p1)
            bd.make_face()
        bd.extrude(amount=SHELF_LENGTH, mode=bd.Mode.ADD)

    return part_builder.part

# ---------------------------------------------------------
# 3. INTERACTIVE 3D VIEWER GENERATOR
# ---------------------------------------------------------
def update_3d_viewer(stl_path, html_path):
    with open(stl_path, "rb") as f:
        stl_b64 = base64.b64encode(f.read()).decode("utf-8")

    html_content = f"""<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>3D Interactive Viewer - LiDAR D500 Mounting Plate (v4)</title>
    <style>
        body {{
            margin: 0;
            padding: 0;
            overflow: hidden;
            background-color: #1e1e24;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            color: #fff;
        }}
        #canvas-container {{
            width: 100vw;
            height: 100vh;
            display: block;
        }}
        #ui-panel {{
            position: absolute;
            top: 15px;
            left: 15px;
            background: rgba(30, 30, 36, 0.88);
            backdrop-filter: blur(8px);
            padding: 16px 20px;
            border-radius: 12px;
            border: 1px solid rgba(255, 255, 255, 0.15);
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.35);
            max-width: 340px;
        }}
        h2 {{
            margin: 0 0 8px 0;
            font-size: 16px;
            color: #4da6ff;
        }}
        p {{
            margin: 4px 0;
            font-size: 13px;
            color: #ccc;
            line-height: 1.4;
        }}
        .highlight {{
            color: #55efc4;
            font-weight: 600;
        }}
        .controls-hint {{
            margin-top: 12px;
            padding-top: 10px;
            border-top: 1px solid rgba(255, 255, 255, 0.1);
            font-size: 12px;
            color: #aaa;
        }}
        .btn {{
            background: #2b5c8f;
            color: white;
            border: none;
            padding: 6px 12px;
            border-radius: 6px;
            cursor: pointer;
            font-size: 12px;
            margin-top: 8px;
            margin-right: 6px;
            transition: background 0.2s;
        }}
        .btn:hover {{
            background: #3b7cbd;
        }}
    </style>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
</head>
<body>
    <div id="ui-panel">
        <h2>Tấm Gia Cố LiDAR D500 (v4)</h2>
        <p>• Kích thước tấm: <b>80 x 100 x 2.0 mm</b> (Dày 2.0mm)</p>
        <p>• Chữ L đỡ mạch: <span class="highlight">Dày 3.5mm chống gãy</span></p>
        <p>• Hướng chữ L: <b>Quay vào trong</b> (Sâu 10mm, gờ 1.5mm)</p>
        <p>• Khoang mạch: <b>20.5 x 20.0 mm</b> (khớp mạch 20mm)</p>
        <p>• 4 lỗ góc M2.5: <b>&empty;2.8mm</b> (ngang 58mm)</p>
        <p>• 3 lỗ LiDAR: <b>&empty;2.7mm</b> (D500 / STL-19P)</p>
        <p>• Lỗ dẫn cáp: <b>13x8mm</b> (tại Y=40mm)</p>
        <div class="controls-hint">
            <b>Thao tác xem 3D:</b><br>
            • Chuột trái: Xoay 360&deg;<br>
            • Cuộn chuột: Phóng to / Thu nhỏ<br>
            • Chuột phải: Di chuyển góc nhìn
        </div>
        <button class="btn" onclick="resetCamera()">Đặt lại góc nhìn</button>
        <button class="btn" onclick="toggleWireframe()">Bật/Tắt Lưới</button>
    </div>
    <div id="canvas-container"></div>

    <script>
        const stlBase64 = "{stl_b64}";

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1e1e24);

        const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.set(120, 140, 160);

        const renderer = new THREE.WebGLRenderer({{ antialias: true }});
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true;
        document.getElementById("canvas-container").appendChild(renderer.domElement);

        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.target.set(40, 50, -5);

        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);

        const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.85);
        dirLight1.position.set(100, 200, 100);
        scene.add(dirLight1);

        const dirLight2 = new THREE.DirectionalLight(0x4da6ff, 0.45);
        dirLight2.position.set(-100, -100, -100);
        scene.add(dirLight2);

        const gridHelper = new THREE.GridHelper(160, 16, 0x4da6ff, 0x333344);
        gridHelper.position.set(40, 50, -13.5);
        gridHelper.rotation.x = Math.PI / 2;
        scene.add(gridHelper);

        const axesHelper = new THREE.AxesHelper(30);
        axesHelper.position.set(0, 0, 0);
        scene.add(axesHelper);

        const binaryString = atob(stlBase64);
        const bytes = new Uint8Array(binaryString.length);
        for (let i = 0; i < binaryString.length; i++) {{
            bytes[i] = binaryString.charCodeAt(i);
        }}

        const loader = new THREE.STLLoader();
        const geometry = loader.parse(bytes.buffer);
        geometry.computeVertexNormals();

        const material = new THREE.MeshPhongMaterial({{
            color: 0x2b7bbb,
            specular: 0x333333,
            shininess: 30,
            side: THREE.DoubleSide
        }});

        const mesh = new THREE.Mesh(geometry, material);
        scene.add(mesh);

        function resetCamera() {{
            camera.position.set(120, 140, 160);
            controls.target.set(40, 50, -5);
            controls.update();
        }}

        let isWireframe = false;
        function toggleWireframe() {{
            isWireframe = !isWireframe;
            material.wireframe = isWireframe;
        }}

        window.addEventListener("resize", () => {{
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        }});

        function animate() {{
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }}
        animate();
    </script>
</body>
</html>
"""
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"Interactive 3D Viewer updated: {html_path}")

# ---------------------------------------------------------
# 4. PREVIEW IMAGE RENDERER
# ---------------------------------------------------------
def render_preview(stl_path, preview_path):
    mesh = trimesh.load_mesh(stl_path)
    fig = plt.figure(figsize=(14, 7), dpi=150)

    # 1. 3D Isometric View
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    poly = Poly3DCollection(mesh.triangles[::2], alpha=0.9, edgecolor="none")
    poly.set_facecolor("#2b7bbb")
    ax1.add_collection3d(poly)
    ax1.set_xlim(mesh.bounds[0][0], mesh.bounds[1][0])
    ax1.set_ylim(mesh.bounds[0][1], mesh.bounds[1][1])
    ax1.set_zlim(mesh.bounds[0][2] - 5, mesh.bounds[1][2] + 5)
    ax1.set_title("3D Isometric View (L-bracket 3.5mm Inward)", fontsize=13, fontweight="bold")
    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.set_zlabel("Z (mm)")
    ax1.view_init(elev=32, azim=-62)

    # 2. 2D Top-Down Projection
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_aspect("equal")
    ax2.set_xlim(-5, 85)
    ax2.set_ylim(-5, 105)
    ax2.set_title("Top-Down View (Plate: 2.0mm, L-Bracket: 3.5mm)", fontsize=13, fontweight="bold")
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.grid(True, linestyle="--", alpha=0.6)

    # Outer Plate Boundary
    ax2.plot([0, 80, 80, 0, 0], [0, 0, 100, 100, 0], "k-", linewidth=2, label="Plate Boundary (80x100mm)")

    # Corner Holes
    for x, y in CORNER_HOLES:
        ax2.add_patch(plt.Circle((x, y), CORNER_HOLE_DIA / 2, color="crimson", fill=True))

    # LiDAR Holes
    for x, y in LIDAR_HOLES:
        ax2.add_patch(plt.Circle((x, y), LIDAR_HOLE_DIA / 2, color="forestgreen", fill=True))

    # Cable Slot
    ax2.add_patch(plt.Rectangle(
        (SLOT_CENTER_X - SLOT_WIDTH / 2, SLOT_CENTER_Y - SLOT_HEIGHT / 2),
        SLOT_WIDTH, SLOT_HEIGHT, color="orange", alpha=0.8
    ))

    # Inward L-bracket Shelf
    ax2.add_patch(plt.Rectangle(
        (SHELF_X_MIN, SHELF_Y_START),
        SHELF_LENGTH, WALL_THICK + SHELF_DEPTH,
        color="purple", alpha=0.35, label="Inward L-Bracket (Thick 3.5mm)"
    ))
    # Retention Lip
    ax2.add_patch(plt.Rectangle(
        (SHELF_X_MIN, SHELF_Y_START + WALL_THICK + PCB_CLEAR_WIDTH),
        SHELF_LENGTH, LIP_THICK,
        color="darkmagenta", alpha=0.6, label="Retention Lip (1.5mm)"
    ))

    ax2.scatter([], [], color="crimson", label="Corner Holes (dia 2.8mm)")
    ax2.scatter([], [], color="forestgreen", label="LiDAR Holes (dia 2.7mm)")
    ax2.scatter([], [], color="orange", label="Cable Slot @ Y=40mm (13x8mm)")
    ax2.legend(loc="upper left", fontsize=8)

    plt.tight_layout()
    plt.savefig(preview_path, dpi=150)
    plt.close()
    print(f"Preview image rendered: {preview_path}")

# ---------------------------------------------------------
# 5. MAIN EXECUTION PROTOCOL
# ---------------------------------------------------------
def main():
    os.makedirs("output", exist_ok=True)
    step_path = "output/lidar_d500_plate.step"
    stl_path = "output/lidar_d500_plate.stl"
    viewer_path = "output/view_3d.html"
    preview_path = "output/lidar_d500_plate_preview.png"

    print("Generating updated 3D model (Plate thickness 2.0mm, L-Bracket thickness 3.5mm)...")
    model = create_model()

    print(f"Exporting STEP to {step_path}...")
    bd.export_step(model, step_path)

    print(f"Exporting STL to {stl_path}...")
    bd.export_stl(model, stl_path, tolerance=0.01, angular_tolerance=0.1)

    # Verification via trimesh
    print("\n================ TRIMESH AUDIT REPORT ================")
    mesh = trimesh.load_mesh(stl_path)
    print(f"  Watertight (Manifold): {mesh.is_watertight}")
    print(f"  Total Volume:          {mesh.volume:.2f} mm^3 ({mesh.volume/1000:.2f} cm^3)")
    print(f"  Bounding Box Min:      [{mesh.bounds[0][0]:.2f}, {mesh.bounds[0][1]:.2f}, {mesh.bounds[0][2]:.2f}]")
    print(f"  Bounding Box Max:      [{mesh.bounds[1][0]:.2f}, {mesh.bounds[1][1]:.2f}, {mesh.bounds[1][2]:.2f}]")
    bb_size = mesh.bounds[1] - mesh.bounds[0]
    print(f"  Overall Envelope:      {bb_size[0]:.2f} mm (X) x {bb_size[1]:.2f} mm (Y) x {bb_size[2]:.2f} mm (Z)")
    print("======================================================")

    assert mesh.is_watertight, "ERROR: Mesh is not watertight!"
    assert bb_size[0] >= 79.9 and bb_size[1] >= 99.9, "ERROR: Bounding box size mismatch!"
    assert abs(mesh.bounds[0][2] - (-13.5)) < 0.1, f"ERROR: Bottom Z mismatch! Expected -13.5, got {mesh.bounds[0][2]}"

    # Update 3D Viewer & Render Preview
    update_3d_viewer(stl_path, viewer_path)
    render_preview(stl_path, preview_path)

    # Copy to artifact dir
    artifact_preview = "/home/zxcvmh/.gemini/antigravity/brain/2036ff51-7502-4a06-bab3-65015cf9f502/lidar_d500_plate_preview.png"
    if os.path.exists(artifact_preview):
        import shutil
        shutil.copyfile(preview_path, artifact_preview)
        print(f"Copied updated preview to artifact directory: {artifact_preview}")

    print("\n>>> VERIFICATION COMPLETE: All files generated and verified successfully! <<<")

if __name__ == "__main__":
    main()
