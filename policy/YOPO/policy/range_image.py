import numpy as np
import torch


def pointcloud_to_range_image(points, vertical_lines=64, horizontal_num=360,
                               vertical_fov_up=38.7, vertical_fov_down=-38.7,
                               max_range=20.0, min_range=0.1):
    if points.shape[0] == 0:
        range_image = np.zeros((vertical_lines, horizontal_num, 5), dtype=np.float32)
        return range_image

    fov_up = vertical_fov_up / 180.0 * np.pi
    fov_down = vertical_fov_down / 180.0 * np.pi
    fov_total = abs(fov_down) + abs(fov_up)

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    depth = np.sqrt(x ** 2 + y ** 2 + z ** 2)

    yaw = -np.arctan2(y, x)
    pitch = np.arcsin(np.clip(z / (depth + 1e-8), -1.0, 1.0))

    row_idx = (1.0 - (pitch - fov_down) / fov_total) * vertical_lines
    col_idx = 0.5 * (yaw / np.pi + 1.0) * horizontal_num

    row_idx = np.floor(row_idx).astype(np.int32)
    col_idx = np.floor(col_idx).astype(np.int32)

    valid = (row_idx >= 0) & (row_idx < vertical_lines) & \
            (col_idx >= 0) & (col_idx < horizontal_num) & \
            (depth > min_range) & (depth < max_range)

    row_idx = row_idx[valid]
    col_idx = col_idx[valid]
    depth = depth[valid]
    x = x[valid]
    y = y[valid]
    z = z[valid]

    range_image = np.zeros((vertical_lines, horizontal_num, 5), dtype=np.float32)
    order = np.argsort(-depth)
    row_idx = row_idx[order]
    col_idx = col_idx[order]
    depth = depth[order]
    x = x[order]
    y = y[order]
    z = z[order]

    range_image[row_idx, col_idx, 0] = depth / max_range
    range_image[row_idx, col_idx, 1] = x / max_range
    range_image[row_idx, col_idx, 2] = y / max_range
    range_image[row_idx, col_idx, 3] = z / max_range
    range_image[row_idx, col_idx, 4] = 1.0

    return range_image


def pointcloud_to_range_image_torch(points, vertical_lines=64, horizontal_num=360,
                                     vertical_fov_up=38.7, vertical_fov_down=-38.7,
                                     max_range=20.0, min_range=0.1):
    if points.shape[0] == 0:
        range_image = torch.zeros((vertical_lines, horizontal_num, 5), dtype=torch.float32, device=points.device)
        return range_image

    fov_up = vertical_fov_up / 180.0 * np.pi
    fov_down = vertical_fov_down / 180.0 * np.pi
    fov_total = abs(fov_down) + abs(fov_up)

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    depth = torch.sqrt(x ** 2 + y ** 2 + z ** 2)

    yaw = -torch.atan2(y, x)
    pitch = torch.asin(torch.clamp(z / (depth + 1e-8), -1.0, 1.0))

    row_idx = ((1.0 - (pitch - fov_down) / fov_total) * vertical_lines).long()
    col_idx = (0.5 * (yaw / np.pi + 1.0) * horizontal_num).long()

    valid = (row_idx >= 0) & (row_idx < vertical_lines) & \
            (col_idx >= 0) & (col_idx < horizontal_num) & \
            (depth > min_range) & (depth < max_range)

    row_idx = row_idx[valid]
    col_idx = col_idx[valid]
    depth = depth[valid]
    x = x[valid]
    y = y[valid]
    z = z[valid]

    range_image = torch.zeros((vertical_lines, horizontal_num, 5), dtype=torch.float32, device=points.device)
    order = torch.argsort(-depth)
    row_idx = row_idx[order]
    col_idx = col_idx[order]
    depth = depth[order]
    x = x[order]
    y = y[order]
    z = z[order]

    range_image[row_idx, col_idx, 0] = depth / max_range
    range_image[row_idx, col_idx, 1] = x / max_range
    range_image[row_idx, col_idx, 2] = y / max_range
    range_image[row_idx, col_idx, 3] = z / max_range
    range_image[row_idx, col_idx, 4] = 1.0

    return range_image


def load_ply_pointcloud(ply_path):
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(ply_path)
    return np.asarray(pcd.points, dtype=np.float32)
