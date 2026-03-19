from text import drawText
from fontDict import fonts
from calcs import *
from geometryUtils import *

pygame.init()

# --- Screen & Rigid Layout Setup ---
LOGICAL_RES = (2560, 1440)
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
sw, sh = screen.get_size()
scale_factor = min(sw / LOGICAL_RES[0], sh / LOGICAL_RES[1])
offset_x = (sw - (LOGICAL_RES[0] * scale_factor)) / 2
offset_y = (sh - (LOGICAL_RES[1] * scale_factor)) / 2

screen_ui = pygame.Surface(LOGICAL_RES).convert_alpha()
clock = pygame.time.Clock()


class Endesga:
    black = [19, 19, 19]
    my_blue = [32, 36, 46]
    greyVD = [50, 50, 50]
    greyD = [100, 100, 100]
    greyL = [200, 200, 200]
    cream = [237, 171, 80]
    white = [255, 255, 255]
    debug_red = [255, 96, 141]
    network_green = [64, 128, 67]
    sebastian_lague_purple = [70, 74, 124]
    very_light_blue = [199, 207, 221]


# --- Simulation State ---
WORLD_SIZE = 8.0
drone_pos = pygame.Vector2(4.0, 4.0)
drone_heading = math.radians(135)
wing_span = 2.0
tag_size = 0.5

cam_pos = pygame.Vector2(0.0, 0.0)
cam_look_angle = math.radians(45)
cam_fov = 60.0
focal_len = 1000.0

# --- Rigid UI Layout ---
BOX_SIZE = 1200
box_rect = pygame.Rect(LOGICAL_RES[0] - BOX_SIZE - 60, (LOGICAL_RES[1] - BOX_SIZE) // 2, BOX_SIZE, BOX_SIZE)
left_panel_w = box_rect.left

# Distinct Tag Colors
col_tag_c = Endesga.network_green
col_tag_l = [40, 180, 180]
col_tag_r = [180, 180, 40]
tag_cols = [col_tag_l, col_tag_c, col_tag_r]


# --- Helper Functions ---
def get_ray_box_intersect(ox, oy, angle, size):
    nx, ny = math.cos(angle), math.sin(angle)
    best_t = float('inf')
    int_pt = None
    wall_dir = None
    if abs(nx) > 1e-6:
        for x_wall in [0, size]:
            t = (x_wall - ox) / nx
            if t > 0:
                y = oy + t * ny
                if 0 <= y <= size and t < best_t:
                    best_t, int_pt, wall_dir = t, (x_wall, y), pygame.Vector2(0, 1)
    if abs(ny) > 1e-6:
        for y_wall in [0, size]:
            t = (y_wall - oy) / ny
            if t > 0:
                x = ox + t * nx
                if 0 <= x <= size and t < best_t:
                    best_t, int_pt, wall_dir = t, (x, y_wall), pygame.Vector2(1, 0)
    return int_pt, wall_dir


def draw_arc_lines(surface, center, angle1, angle2, radius, color, width=4):
    diff = (angle2 - angle1)
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    pts = []
    steps = max(15, int(abs(diff) * 20))
    for i in range(steps + 1):
        a = angle1 + diff * (i / steps)
        pts.append((center[0] + math.cos(a) * radius, center[1] - math.sin(a) * radius))
    if len(pts) > 1:
        pygame.draw.lines(surface, color, False, pts, width)


running = True
dragging = False

while running:
    mx_raw, my_raw = pygame.mouse.get_pos()
    mx = (mx_raw - offset_x) / scale_factor
    my = (my_raw - offset_y) / scale_factor
    dt = clock.tick(60) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: running = False
        if event.type == pygame.MOUSEBUTTONDOWN and box_rect.collidepoint(mx, my): dragging = True
        if event.type == pygame.MOUSEBUTTONUP: dragging = False
        if event.type == pygame.MOUSEWHEEL: drone_heading += event.y * 0.10

    keys = pygame.key.get_pressed()
    speed = 4.0 * dt
    if keys[pygame.K_w]: drone_pos.y += speed
    if keys[pygame.K_s]: drone_pos.y -= speed
    if keys[pygame.K_a]: drone_pos.x -= speed
    if keys[pygame.K_d]: drone_pos.x += speed
    if keys[pygame.K_q]: drone_heading += 2.0 * dt
    if keys[pygame.K_e]: drone_heading -= 2.0 * dt

    if dragging:
        drone_pos = screen_to_world((mx, my), box_rect)

    drone_pos.x = max(0.3, min(WORLD_SIZE - 0.3, drone_pos.x))
    drone_pos.y = max(0.3, min(WORLD_SIZE - 0.3, drone_pos.y))

    # --- Calculations ---
    offset = pygame.Vector2(math.cos(drone_heading), math.sin(drone_heading)) * (wing_span / 2)
    tag_centers = [drone_pos - offset, drone_pos, drone_pos + offset]

    normal_angle = drone_heading - math.pi / 2
    cam_ray_angle = math.atan2(drone_pos.y - cam_pos.y, drone_pos.x - cam_pos.x)
    dist_to_center = distance((cam_pos.x, cam_pos.y), (drone_pos.x, drone_pos.y))

    # --- Rendering ---
    screen.fill(Endesga.black)
    screen_ui.fill((0, 0, 0, 0))

    # 1. 8x8 Grid System
    pygame.draw.rect(screen_ui, Endesga.my_blue, box_rect)
    pygame.draw.rect(screen_ui, Endesga.greyVD, box_rect, 4)
    for i in range(1, int(WORLD_SIZE)):
        x_px = world_to_screen((i, 0), box_rect).x
        pygame.draw.line(screen_ui, Endesga.greyVD, (x_px, box_rect.top), (x_px, box_rect.bottom), 2)
        y_px = world_to_screen((0, i), box_rect).y
        pygame.draw.line(screen_ui, Endesga.greyVD, (box_rect.left, y_px), (box_rect.right, y_px), 2)

    # 2. Camera FOV
    cam_px = world_to_screen(cam_pos, box_rect)
    fov_surf = pygame.Surface(LOGICAL_RES, pygame.SRCALPHA)
    p1 = cam_px + pygame.Vector2(math.cos(cam_look_angle - math.radians(cam_fov / 2)),
                                 -math.sin(cam_look_angle - math.radians(cam_fov / 2))) * 2000
    p2 = cam_px + pygame.Vector2(math.cos(cam_look_angle + math.radians(cam_fov / 2)),
                                 -math.sin(cam_look_angle + math.radians(cam_fov / 2))) * 2000
    pygame.draw.polygon(fov_surf, (*Endesga.sebastian_lague_purple, 50), [cam_px, p1, p2])

    screen_ui.set_clip(box_rect)
    screen_ui.blit(fov_surf, (0, 0))
    screen_ui.set_clip(None)

    # Label Camera Origin
    drawText(screen_ui, Endesga.white, fonts["bold24"], cam_px.x + 20, cam_px.y - 40, "Camera (0,0)")

    # 3. Ray Casting & Intersections
    px_centers = [world_to_screen(p, box_rect) for p in tag_centers]

    # Distance/Depth Ray to Center
    pygame.draw.line(screen_ui, (*Endesga.white, 150), cam_px, px_centers[1], 3)
    mid_ray = cam_px + (px_centers[1] - cam_px) * 0.5
    drawText(screen_ui, Endesga.white, fonts["bold26"], mid_ray.x - 30, mid_ray.y - 40, f"ZC = {dist_to_center:.1f} ft")

    # --- ANGLE BASHING & CONSTRUCTION LINES (TOP-DOWN) ---
    wL, wR = tag_centers[0], tag_centers[2]
    cam_rt = pygame.Vector2(math.cos(cam_look_angle - math.pi / 2), math.sin(cam_look_angle - math.pi / 2))

    # Project Wing vector onto Camera Right vector to form a right triangle
    vec_LR = wR - wL
    lat_dist = vec_LR.dot(cam_rt)
    w_corner = wL + cam_rt * lat_dist

    px_L = world_to_screen(wL, box_rect)
    px_R = world_to_screen(wR, box_rect)
    px_corner = world_to_screen(w_corner, box_rect)

    # Draw the construction Right Triangle
    pygame.draw.line(screen_ui, Endesga.very_light_blue, px_L, px_corner, 3)
    pygame.draw.line(screen_ui, Endesga.debug_red, px_corner, px_R, 4)

    # Label Depth Delta (ΔZ)
    mid_dZ = (px_corner + px_R) / 2
    drawText(screen_ui, Endesga.debug_red, fonts["bold26"], mid_dZ.x + 15, mid_dZ.y, "ΔZ (Depth Delta)")

    # Draw Incidence Angle 'B' using WORLD coordinates to ensure perfect alignment with draw_arc_lines
    angle_lat_world = math.atan2(w_corner.y - wL.y, w_corner.x - wL.x)
    angle_wing_world = math.atan2(wR.y - wL.y, wR.x - wL.x)
    draw_arc_lines(screen_ui, px_L, angle_lat_world, angle_wing_world, 70, Endesga.cream, 4)

    B_val = abs((angle_wing_world - angle_lat_world + math.pi) % (2 * math.pi) - math.pi)
    drawText(screen_ui, Endesga.cream, fonts["bold30"], px_L.x + 25, px_L.y - 60, f"B = {math.degrees(B_val):.1f}°")

    # Normal Line & Box Impact
    intersect_pt, wall_dir = get_ray_box_intersect(drone_pos.x, drone_pos.y, normal_angle, WORLD_SIZE)
    if intersect_pt:
        intersect_px = world_to_screen(intersect_pt, box_rect)
        pygame.draw.line(screen_ui, Endesga.debug_red, px_centers[1], intersect_px, 4)
        pygame.draw.circle(screen_ui, Endesga.white, (int(intersect_px.x), int(intersect_px.y)), 10, 4)

        # Wall Impact Arc
        ray_vec = pygame.Vector2(math.cos(normal_angle), math.sin(normal_angle))
        impact_deg = math.degrees(math.acos(abs(ray_vec.dot(wall_dir))))
        wall_angle_rad = math.atan2(wall_dir.y, wall_dir.x)
        draw_arc_lines(screen_ui, intersect_px, normal_angle - math.pi, wall_angle_rad, 50, Endesga.cream, 4)
        drawText(screen_ui, Endesga.cream, fonts["bold26"], intersect_px.x - 55, intersect_px.y - 55,
                 f"{impact_deg:.1f}°")

    # Draw Drone Wing
    drawRoundedLine(pygame, screen_ui, px_centers[0], px_centers[2], Endesga.cream, 12)
    for i, pt in enumerate(px_centers):
        pygame.draw.circle(screen_ui, tag_cols[i], (int(pt.x), int(pt.y)), 12)

    # --- 5. LEFT SIDEBAR UI ---
    drawText(screen_ui, Endesga.white, fonts["bold50"], 50, 40, "POSE ESTIMATION VISUALIZER")

    # =========================================================
    # SYSTEM PARAMETERS & CONSTANTS
    # =========================================================
    drawText(screen_ui, Endesga.white, fonts["bold30"], 50, 110, "0. SYSTEM PARAMETERS & CONSTANTS")

    k_mat_x = 50
    drawText(screen_ui, Endesga.greyL, fonts["bold26"], k_mat_x, 150, "Intrinsic Matrix (K):")
    drawText(screen_ui, Endesga.very_light_blue, fonts["bold24"], k_mat_x, 190, f"| {focal_len:<6.1f}   0.0      cx |")
    drawText(screen_ui, Endesga.very_light_blue, fonts["bold24"], k_mat_x, 220, f"| 0.0      {focal_len:<6.1f}   cy |")
    drawText(screen_ui, Endesga.very_light_blue, fonts["bold24"], k_mat_x, 250, "| 0.0      0.0      1.0|")

    params_x = 450
    drawText(screen_ui, Endesga.greyL, fonts["bold26"], params_x, 150, f"Camera FOV: {cam_fov:.1f}°")
    drawText(screen_ui, Endesga.greyL, fonts["bold26"], params_x, 190, f"Tag Size (w): {tag_size:.2f} ft")
    drawText(screen_ui, Endesga.greyL, fonts["bold26"], params_x, 230, f"Wing Span (W): {wing_span:.2f} ft")

    # =========================================================
    # SENSOR VIEW (TRAPEZOIDS)
    # =========================================================
    drawText(screen_ui, Endesga.white, fonts["bold30"], 50, 310, "1. SENSOR PROJECTION (3D -> 2D Camera Plane)")
    fv_rect = pygame.Rect(50, 360, left_panel_w - 100, 300)
    pygame.draw.rect(screen_ui, Endesga.my_blue, fv_rect, 0, 12)
    pygame.draw.rect(screen_ui, Endesga.greyVD, fv_rect, 4, 12)

    sensor_cx, sensor_cy = fv_rect.centerx, fv_rect.centery - 20

    screen_ui.set_clip(fv_rect.inflate(-6, -6))
    tag_centers_proj = []
    center_tag_2d_corners = []

    for i, center_ft in enumerate(tag_centers):
        corners_3d = get_tag_corners_3d(center_ft, drone_heading, tag_size)
        corners_2d = []
        depth_sum = 0
        valid = True
        for c3d in corners_3d:
            proj = project_3d_to_sensor(c3d, cam_pos, cam_look_angle, focal_len)
            if not proj: valid = False; break
            sx, sy, sz = proj
            corners_2d.append((sensor_cx + sx, sensor_cy - sy))
            depth_sum += sz

        if valid:
            avg_depth = depth_sum / 4
            tag_centers_proj.append((sensor_cx +
                                     project_3d_to_sensor((center_ft[0], center_ft[1], tag_size / 2), cam_pos,
                                                          cam_look_angle, focal_len)[0], avg_depth))
            pygame.draw.polygon(screen_ui, tag_cols[i], corners_2d)
            pygame.draw.polygon(screen_ui, Endesga.white, corners_2d, 3)

            if i == 1:
                center_tag_2d_corners = corners_2d
        else:
            tag_centers_proj.append(None)

    # DIMENSIONS OVERLAY
    if all(tag_centers_proj) and center_tag_2d_corners:
        u_L, z_L = tag_centers_proj[0]
        u_C, z_C = tag_centers_proj[1]
        u_R, z_R = tag_centers_proj[2]

        # 3-Tag Deltas (Below trapezoids)
        dim_y = sensor_cy + 110
        pygame.draw.line(screen_ui, Endesga.greyL, (u_L, dim_y), (u_C, dim_y), 4)
        pygame.draw.line(screen_ui, Endesga.greyL, (u_C, dim_y), (u_R, dim_y), 4)
        for u in [u_L, u_C, u_R]:
            pygame.draw.line(screen_ui, Endesga.greyL, (u, dim_y - 15), (u, dim_y + 15), 4)

        d1 = abs(u_C - u_L)
        d2 = abs(u_R - u_C)
        drawText(screen_ui, tag_cols[0], fonts["bold26"], (u_L + u_C) / 2, dim_y + 20, "Δ1", justify="center")
        drawText(screen_ui, tag_cols[2], fonts["bold26"], (u_C + u_R) / 2, dim_y + 20, "Δ2", justify="center")

        # 1-Tag Trapezoid Heights (With flipping logic to prevent overlap)
        tl, tr, br, bl = center_tag_2d_corners
        h_left = abs(tl[1] - bl[1])
        h_right = abs(tr[1] - br[1])

        # Check if the tag is physically flipped on screen
        flip_mult = -1 if tl[0] > tr[0] else 1

        # Left physical edge
        pygame.draw.line(screen_ui, Endesga.debug_red, (tl[0], tl[1]), (tl[0] - 30 * flip_mult, tl[1]), 3)
        pygame.draw.line(screen_ui, Endesga.debug_red, (bl[0], bl[1]), (bl[0] - 30 * flip_mult, bl[1]), 3)
        pygame.draw.line(screen_ui, Endesga.debug_red, (tl[0] - 20 * flip_mult, tl[1]), (tl[0] - 20 * flip_mult, bl[1]),
                         3)
        align_L = "right" if flip_mult == 1 else "left"
        drawText(screen_ui, Endesga.debug_red, fonts["bold24"], tl[0] - 35 * flip_mult, (tl[1] + bl[1]) / 2 - 15, "hL",
                 justify=align_L)

        # Right physical edge
        pygame.draw.line(screen_ui, Endesga.debug_red, (tr[0], tr[1]), (tr[0] + 30 * flip_mult, tr[1]), 3)
        pygame.draw.line(screen_ui, Endesga.debug_red, (br[0], br[1]), (br[0] + 30 * flip_mult, br[1]), 3)
        pygame.draw.line(screen_ui, Endesga.debug_red, (tr[0] + 20 * flip_mult, tr[1]), (tr[0] + 20 * flip_mult, br[1]),
                         3)
        align_R = "left" if flip_mult == 1 else "right"
        drawText(screen_ui, Endesga.debug_red, fonts["bold24"], tr[0] + 35 * flip_mult, (tr[1] + br[1]) / 2 - 15, "hR",
                 justify=align_R)

    screen_ui.set_clip(None)

    # =========================================================
    # MEASURED VALUES
    # =========================================================
    if all(tag_centers_proj) and center_tag_2d_corners:
        math_y = fv_rect.bottom + 40
        drawText(screen_ui, Endesga.white, fonts["bold36"], 50, math_y, "2. MEASURED PIXEL & DEPTH VALUES")

        drawText(screen_ui, Endesga.greyL, fonts["bold30"], 50, math_y + 60, "Depths (Z):")
        drawText(screen_ui, tag_cols[0], fonts["bold30"], 270, math_y + 60, f"ZL = {z_L:.2f} ft")
        drawText(screen_ui, tag_cols[1], fonts["bold30"], 490, math_y + 60, f"ZC = {z_C:.2f} ft")
        drawText(screen_ui, tag_cols[2], fonts["bold30"], 710, math_y + 60, f"ZR = {z_R:.2f} ft")

        drawText(screen_ui, Endesga.greyL, fonts["bold30"], 50, math_y + 110, "Tag Spacing:")
        drawText(screen_ui, tag_cols[0], fonts["bold30"], 270, math_y + 110, f"Δ1 = {d1:.1f} px")
        drawText(screen_ui, tag_cols[2], fonts["bold30"], 490, math_y + 110, f"Δ2 = {d2:.1f} px")

        drawText(screen_ui, Endesga.greyL, fonts["bold30"], 50, math_y + 160, "Center Edges:")
        drawText(screen_ui, Endesga.debug_red, fonts["bold30"], 270, math_y + 160, f"hL = {h_left:.1f} px")
        drawText(screen_ui, Endesga.debug_red, fonts["bold30"], 490, math_y + 160, f"hR = {h_right:.1f} px")

        # =========================================================
        # CALCULATIONS
        # =========================================================
        calc_y = math_y + 240
        drawText(screen_ui, Endesga.white, fonts["bold36"], 50, calc_y, "3. YAW (B) ESTIMATION")

        col1_x = 50
        drawText(screen_ui, Endesga.cream, fonts["bold26"], col1_x, calc_y + 60,
                 "Method A: Multi-Tag Spacing (Δ1 vs Δ2)", wrap=True, maxLen=580)
        drawText(screen_ui, Endesga.very_light_blue, fonts["bold24"], col1_x, calc_y + 100,
                 "sin(B) ≈ ( 2 · ZC / W ) · ( Δ1 - Δ2 ) / ( Δ1 + Δ2 )", wrap=True, maxLen=580)

        ratio_term_3tag = (d1 - d2) / (d1 + d2 + 1e-6)
        sin_B_3tag = (2 * z_C / wing_span) * ratio_term_3tag
        sin_B_3tag = max(-1.0, min(1.0, sin_B_3tag))
        B_est_3tag = math.degrees(math.asin(sin_B_3tag))

        drawText(screen_ui, Endesga.white, fonts["bold26"], col1_x, calc_y + 150,
                 f"sin(B) ≈ ( 2 · {z_C:.1f} / {wing_span:.1f} ) · {ratio_term_3tag:.3f}", wrap=True, maxLen=580)
        drawText(screen_ui, tag_cols[1], fonts["bold30"], col1_x, calc_y + 190, f"B ≈ {B_est_3tag:.1f}°", wrap=True,
                 maxLen=580)

        col2_x = 680
        drawText(screen_ui, Endesga.cream, fonts["bold26"], col2_x, calc_y + 60,
                 "Method B: Single-Tag Distortion (hL vs hR)", wrap=True, maxLen=580)
        drawText(screen_ui, Endesga.very_light_blue, fonts["bold24"], col2_x, calc_y + 100,
                 "sin(B) ≈ ( 2 · ZC / w ) · ( hL - hR ) / ( hL + hR )", wrap=True, maxLen=580)

        ratio_term_1tag = (h_left - h_right) / (h_left + h_right + 1e-6)
        sin_B_1tag = (2 * z_C / tag_size) * ratio_term_1tag
        sin_B_1tag = max(-1.0, min(1.0, sin_B_1tag))
        B_est_1tag = math.degrees(math.asin(sin_B_1tag))

        drawText(screen_ui, Endesga.white, fonts["bold26"], col2_x, calc_y + 150,
                 f"sin(B) ≈ ( 2 · {z_C:.1f} / {tag_size:.1f} ) · {ratio_term_1tag:.3f}", wrap=True, maxLen=580)
        drawText(screen_ui, tag_cols[1], fonts["bold30"], col2_x, calc_y + 190, f"B ≈ {B_est_1tag:.1f}°", wrap=True,
                 maxLen=580)

        # Global Result
        global_angle = (math.degrees(cam_ray_angle) + B_est_3tag) % 360
        res_y = calc_y + 260
        pygame.draw.line(screen_ui, Endesga.greyVD, (50, res_y), (left_panel_w - 50, res_y), 4)
        drawText(screen_ui, Endesga.white, fonts["bold30"], 50, res_y + 30, "FINAL GLOBAL NORMAL RECOVERY:")
        drawText(screen_ui, Endesga.greyL, fonts["bold30"], 50, res_y + 80,
                 f"Global Normal = Camera Ray ({math.degrees(cam_ray_angle):.1f}°) + B ({B_est_3tag:.1f}°) = {global_angle:.1f}°",
                 wrap=True, maxLen=left_panel_w - 100)

    # Controls
    drawText(screen_ui, Endesga.greyD, fonts["bold24"], 50, LOGICAL_RES[1] - 60,
             "WASD / Mouse Drag: Move Drone  |  Scroll Wheel / Q, E: Rotate Drone")

    # Final Scaling & Blit
    final_screen = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    final_screen.fill(Endesga.black)
    scaled_ui = pygame.transform.smoothscale(screen_ui,
                                             (int(LOGICAL_RES[0] * scale_factor), int(LOGICAL_RES[1] * scale_factor)))
    final_screen.blit(scaled_ui, (offset_x, offset_y))

    screen.blit(final_screen, (0, 0))
    pygame.display.flip()

pygame.quit()