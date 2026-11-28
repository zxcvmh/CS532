### TK-CV-05: AprilTag / ArUco drift anchor and optional goal (should-have)
- Role: perception
- Status: CANCELLED 2026-11-27 (human confirmed ArUco/AprilTag is not used; Jetson inventory `docs/measurements/2026-11-16/jetson_inventory.txt` also verified OpenCV 4.1.1 has `cv2.aruco available: False`).
- Plan: one tag on a stand in the room; pose of the tag in the map frame corrects accumulated drift when seen.
  Threshold: tag position error <= 5 cm vs a ruler at 1-2 m; drift after 5 minutes of driving with vs without the anchor.
  Cut first if week-2 field time runs short. Depends on: TK-CV-03.
