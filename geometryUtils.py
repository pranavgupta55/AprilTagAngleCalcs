import math
import pygame


def world_to_screen(pos_ft, box_rect, world_size_ft=8.0):
    """Rigid mapping: 0-8ft rigidly maps to the box's pixel bounds."""
    scale = box_rect.width / world_size_ft
    px = box_rect.left + (pos_ft[0] * scale)
    py = box_rect.top + box_rect.height - (pos_ft[1] * scale)  # Invert Y so 0 is bottom
    return pygame.Vector2(px, py)


def screen_to_world(pos_px, box_rect, world_size_ft=8.0):
    """Rigid mapping: Pixels back to 0-8ft."""
    scale = box_rect.width / world_size_ft
    wx = (pos_px[0] - box_rect.left) / scale
    wy = (box_rect.bottom - pos_px[1]) / scale  # Invert Y
    return pygame.Vector2(wx, wy)


def project_3d_to_sensor(point_3d, cam_pos_2d, cam_yaw, focal_len):
    """
    True 3D Pinhole projection.
    point_3d: (X, Y, Z) in world feet.
    cam_pos_2d: (X, Y) of camera in world feet. (Camera Z is assumed 0 for simplicity).
    cam_yaw: The angle the camera is pointing.
    Returns: (sensor_x, sensor_y, depth_z)
    """
    dx = point_3d[0] - cam_pos_2d[0]
    dy = point_3d[1] - cam_pos_2d[1]
    dz = point_3d[2]  # Height above ground

    fw_x = math.cos(cam_yaw)
    fw_y = math.sin(cam_yaw)

    rt_x = math.cos(cam_yaw - math.pi / 2)
    rt_y = math.sin(cam_yaw - math.pi / 2)

    depth_z = (dx * fw_x) + (dy * fw_y)
    lateral_x = (dx * rt_x) + (dy * rt_y)
    height_y = dz

    if depth_z <= 0.1:  # Point is behind or perfectly inside the camera lens
        return None

    # Pinhole projection formula
    sensor_x = focal_len * (lateral_x / depth_z)
    sensor_y = focal_len * (height_y / depth_z)

    return sensor_x, sensor_y, depth_z


def get_tag_corners_3d(center_ft, heading, size_ft=0.5):
    """Generates the 4 corners of an AprilTag in 3D space.
    Assumes the tag is a vertical square facing the normal of the drone."""
    w, h = size_ft / 2, size_ft / 2
    dx = math.cos(heading) * w
    dy = math.sin(heading) * w

    return[
        (center_ft[0] - dx, center_ft[1] - dy, size_ft),  # Top-Left
        (center_ft[0] + dx, center_ft[1] + dy, size_ft),  # Top-Right
        (center_ft[0] + dx, center_ft[1] + dy, 0.0),  # Bottom-Right
        (center_ft[0] - dx, center_ft[1] - dy, 0.0),  # Bottom-Left
    ]