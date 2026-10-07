import numpy as np
def estimate_distance_from_lidar(bbox, frame_width, lidar_ranges, h_fov_deg=62.0):
    """
    bbox: [xmin, ymin, xmax, ymax]
    frame_width: chiều rộng khung hình camera (vd: 640)
    lidar_ranges: mảng khoảng cách từ LiDAR (theo góc từ 0 đến 360 độ)
    h_fov_deg: góc nhìn ngang của camera
    """
    xmin, _, xmax, _ = bbox
    # Tâm của bounding box trên trục X
    x_center = (xmin + xmax) / 2.0
    
    # Góc lệch tâm (độ) so với trục chính camera (-fov/2 đến +fov/2)
    angle_offset = ((x_center / frame_width) - 0.5) * h_fov_deg
    
    # Giả sử camera nhìn thẳng phía trước (0 độ của LiDAR)
    # Lấy dải góc bao trùm bounding box
    angle_min = ((xmin / frame_width) - 0.5) * h_fov_deg
    angle_max = ((xmax / frame_width) - 0.5) * h_fov_deg
    
    # Lọc các điểm đo của LiDAR trong dải góc này
    # (Cần chuẩn hóa index tương ứng với mảng lidar_ranges)
    total_samples = len(lidar_ranges)
    idx_start = int((angle_min % 360) / 360.0 * total_samples)
    idx_end = int((angle_max % 360) / 360.0 * total_samples)
    
    selected_ranges = [
        lidar_ranges[i % total_samples] 
        for i in range(min(idx_start, idx_end), max(idx_start, idx_end) + 1)
        if 0.1 < lidar_ranges[i % total_samples] < 10.0  # Lọc nhiễu
    ]
    
    if len(selected_ranges) > 0:
        return float(np.median(selected_ranges)) # Khoảng cách thực tế (mét)
    return None