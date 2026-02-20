#!/usr/bin/env python3
"""
Spacemouse teleop visualizer — moves/rotates a 3D box using the same jog
logic as the executor, so you can validate input feel without the robot.

Usage (in a sourced workspace terminal):
    python3 teleop_box_viz.py

Controls shown on screen. Close window or Ctrl-C to quit.
"""

import sys
import math
import threading
import numpy as np


# ── Minimal rotation helpers (no scipy dependency) ────────────────────────────
def _rotvec_to_mat(rvec):
    """Rodrigues formula: rotation vector → 3×3 matrix."""
    angle = np.linalg.norm(rvec)
    if angle < 1e-10:
        return np.eye(3)
    axis = rvec / angle
    K = np.array([
        [ 0,       -axis[2],  axis[1]],
        [ axis[2],  0,       -axis[0]],
        [-axis[1],  axis[0],  0      ],
    ])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _euler_xyz_to_mat(r, p, y):
    """Intrinsic XYZ Euler → 3×3 rotation matrix."""
    return _rotvec_to_mat(np.array([r, 0, 0])) @ \
           _rotvec_to_mat(np.array([0, p, 0])) @ \
           _rotvec_to_mat(np.array([0, 0, y]))


def _mat_to_euler_xyz(R):
    """3×3 rotation matrix → intrinsic XYZ Euler angles (degrees)."""
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        rx = math.degrees(math.atan2( R[2,1], R[2,2]))
        ry = math.degrees(math.atan2(-R[2,0], sy))
        rz = math.degrees(math.atan2( R[1,0], R[0,0]))
    else:
        rx = math.degrees(math.atan2(-R[1,2], R[1,1]))
        ry = math.degrees(math.atan2(-R[2,0], sy))
        rz = 0.0
    return rx, ry, rz


def _make_view_rot():
    rx = _euler_xyz_to_mat(math.radians(-25), 0, 0)
    ry = _euler_xyz_to_mat(0, math.radians(30), 0)
    return rx @ ry

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from ur5_teleop_msgs.msg import PoseDelta

# ── pygame ─────────────────────────────────────────────────────────────────────
import pygame

# ──────────────────────────────────────────────────────────────────────────────
# Jog parameters — mirror what is in teleop_config.yaml
JOG_LINEAR    = 0.05   # m/s
JOG_ANGULAR   = 0.3    # rad/s
JOG_PERIOD    = 0.05   # seconds between jog steps (20 Hz)  ← faster than executor
AXIS_THRESHOLD = 0.05  # same threshold as executor

# Display
WIDTH, HEIGHT = 900, 650
BG            = (30,  30,  40)
BOX_COLOR     = (80, 160, 220)
EDGE_COLOR    = (200, 230, 255)
AXIS_COLORS   = [(220, 60, 60), (60, 200, 60), (60, 100, 220)]  # X Y Z
TEXT_COLOR    = (230, 230, 230)
WARN_COLOR    = (255, 180, 50)
ZERO_COLOR    = (100, 200, 100)

SCALE         = 800   # pixels per metre (box is ~0.15 m wide → looks good)
ORIGIN        = np.array([WIDTH // 2, HEIGHT // 2 - 30], dtype=float)


# ──────────────────────────────────────────────────────────────────────────────
# Box geometry: 8 corners of a 0.15×0.10×0.06 m box, centred at origin
_W, _H, _D = 0.15, 0.10, 0.06
BOX_CORNERS = np.array([
    [-_W, -_H, -_D], [+_W, -_H, -_D], [+_W, +_H, -_D], [-_W, +_H, -_D],
    [-_W, -_H, +_D], [+_W, -_H, +_D], [+_W, +_H, +_D], [-_W, +_H, +_D],
]) / 2

BOX_EDGES = [
    (0,1),(1,2),(2,3),(3,0),   # bottom face
    (4,5),(5,6),(6,7),(7,4),   # top face
    (0,4),(1,5),(2,6),(3,7),   # verticals
]

BOX_FACES = [
    ([0,1,2,3], (60,  120, 180)),  # bottom  — dark blue
    ([4,5,6,7], (80,  160, 220)),  # top     — blue
    ([0,1,5,4], (50,  100, 160)),  # front   — darker
    ([2,3,7,6], (70,  140, 200)),  # back
    ([1,2,6,5], (90,  170, 230)),  # right   — lighter
    ([0,3,7,4], (40,   90, 150)),  # left    — darkest
]

# Axis arrows in local frame
AXIS_ARROWS = np.array([[0.12,0,0],[0,0.12,0],[0,0,0.12]])


def project(points_3d, pos, rot_mat, view_rot):
    """Isometric-ish projection: rotate to view, then orthographic."""
    pts = (rot_mat @ points_3d.T).T + pos
    pts_view = (view_rot @ pts.T).T
    screen = pts_view[:, :2] * SCALE + ORIGIN
    depth  = pts_view[:, 2]
    return screen, depth


VIEW_ROT = _make_view_rot()


# ──────────────────────────────────────────────────────────────────────────────
class TeleopState:
    """Thread-safe store of the latest spacemouse axis values and box pose."""

    def __init__(self):
        self._lock = threading.Lock()
        self.axis = None          # latest PoseDelta or None
        self.pos  = np.array([0.0, 0.0, 0.0])
        self.rot  = np.eye(3)
        self.last_msg_time = None

    def update_axis(self, msg):
        with self._lock:
            self.axis = msg
            self.last_msg_time = pygame.time.get_ticks()

    def jog_step(self):
        """Apply one jog tick. Returns (axis snapshot, steps_applied dict)."""
        with self._lock:
            axis = self.axis
        if axis is None:
            return None, {}

        is_zero = (axis.dx == 0.0 and axis.dy == 0.0 and axis.dz == 0.0 and
                   axis.droll == 0.0 and axis.dpitch == 0.0 and axis.dyaw == 0.0)
        if is_zero:
            return axis, {}

        thr = AXIS_THRESHOLD
        dt  = JOG_PERIOD

        sx = axis.dx * JOG_LINEAR  * dt if abs(axis.dx)    > thr else 0.0
        sy = axis.dy * JOG_LINEAR  * dt if abs(axis.dy)    > thr else 0.0
        sz = axis.dz * JOG_LINEAR  * dt if abs(axis.dz)    > thr else 0.0
        sr = axis.droll  * JOG_ANGULAR * dt if abs(axis.droll)  > thr else 0.0
        sp = axis.dpitch * JOG_ANGULAR * dt if abs(axis.dpitch) > thr else 0.0
        sy2= axis.dyaw   * JOG_ANGULAR * dt if abs(axis.dyaw)   > thr else 0.0

        with self._lock:
            self.pos += np.array([sx, sy, sz])
            if sr != 0.0 or sp != 0.0 or sy2 != 0.0:
                delta_rot = _euler_xyz_to_mat(sr, sp, sy2)
                self.rot = delta_rot @ self.rot

        return axis, {'x': sx, 'y': sy, 'z': sz, 'roll': sr, 'pitch': sp, 'yaw': sy2}

    def get_pose(self):
        with self._lock:
            return self.pos.copy(), self.rot.copy()

    def reset(self):
        with self._lock:
            self.pos = np.array([0.0, 0.0, 0.0])
            self.rot = np.eye(3)


# ──────────────────────────────────────────────────────────────────────────────
class TeleopSubscriber(Node):
    def __init__(self, state: TeleopState):
        super().__init__('teleop_box_viz')
        self._state = state
        self.create_subscription(PoseDelta, '/ur5/teleop_delta', self._cb, 10)
        self.get_logger().info('Subscribed to /ur5/teleop_delta')

    def _cb(self, msg: PoseDelta):
        self._state.update_axis(msg)


# ──────────────────────────────────────────────────────────────────────────────
def draw_box(surf, pos, rot_mat):
    corners_s, depths = project(BOX_CORNERS, pos, rot_mat, VIEW_ROT)

    # Sort faces by average depth (painter's algorithm)
    face_order = []
    for verts, color in BOX_FACES:
        avg_d = np.mean(depths[verts])
        face_order.append((avg_d, verts, color))
    face_order.sort(key=lambda x: x[0])

    for _, verts, color in face_order:
        pts = [corners_s[i] for i in verts]
        pygame.draw.polygon(surf, color, [(int(p[0]), int(p[1])) for p in pts])

    for a, b in BOX_EDGES:
        pa = (int(corners_s[a][0]), int(corners_s[a][1]))
        pb = (int(corners_s[b][0]), int(corners_s[b][1]))
        pygame.draw.line(surf, EDGE_COLOR, pa, pb, 1)

    # Draw local XYZ axes from box centre
    origin_s, _ = project(np.array([[0,0,0]]), pos, rot_mat, VIEW_ROT)
    o = origin_s[0]
    for i, arrow in enumerate(AXIS_ARROWS):
        tip_s, _ = project(np.array([arrow]), pos, rot_mat, VIEW_ROT)
        t = tip_s[0]
        pygame.draw.line(surf, AXIS_COLORS[i],
                         (int(o[0]), int(o[1])), (int(t[0]), int(t[1])), 2)


def draw_hud(surf, font, small_font, state: TeleopState, steps: dict, axis):
    now = pygame.time.get_ticks()

    # ── Active axes bar ──────────────────────────────────────────────────────
    y = 10
    surf.blit(font.render('SpaceMouse Teleop Visualizer', True, TEXT_COLOR), (10, y))
    y += 30

    # Connection status
    if state.last_msg_time is None:
        status_txt = 'Waiting for /ur5/teleop_delta ...'
        status_col = WARN_COLOR
    elif now - state.last_msg_time > 500:
        status_txt = 'No data (spacemouse idle or node stopped)'
        status_col = WARN_COLOR
    else:
        status_txt = 'Receiving data'
        status_col = ZERO_COLOR
    surf.blit(small_font.render(status_txt, True, status_col), (10, y))
    y += 22

    # Raw axis values
    if axis is not None:
        raw_vals = [
            ('dx',    axis.dx,    steps.get('x',    0.0), AXIS_COLORS[0]),
            ('dy',    axis.dy,    steps.get('y',    0.0), AXIS_COLORS[1]),
            ('dz',    axis.dz,    steps.get('z',    0.0), AXIS_COLORS[2]),
            ('droll', axis.droll, steps.get('roll', 0.0), (200,150,50)),
            ('dpitch',axis.dpitch,steps.get('pitch',0.0), (200,100,200)),
            ('dyaw',  axis.dyaw,  steps.get('yaw',  0.0), (100,200,200)),
        ]
        surf.blit(small_font.render(
            f'{"Axis":<8} {"Raw":>8}  {"Above thr?":>12}  {"Step (m/rad)":>14}',
            True, (160,160,160)), (10, y))
        y += 18
        for name, raw, step, col in raw_vals:
            above = abs(raw) > AXIS_THRESHOLD
            above_txt = f'YES ({raw:+.3f})' if above else f'no  ({raw:+.3f})'
            step_txt  = f'{step:+.5f}' if above else '—'
            line = f'{name:<8} {raw:>+8.3f}  {above_txt:>12}  {step_txt:>14}'
            color = col if above else (120, 120, 120)
            surf.blit(small_font.render(line, True, color), (10, y))
            y += 17

    y += 8
    # Box pose
    pos, rot_mat = state.get_pose()
    euler = _mat_to_euler_xyz(rot_mat)
    surf.blit(small_font.render(
        f'Box pos:  x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f} m',
        True, TEXT_COLOR), (10, y)); y += 17
    surf.blit(small_font.render(
        f'Box rot:  r={euler[0]:+.1f}°  p={euler[1]:+.1f}°  y={euler[2]:+.1f}°',
        True, TEXT_COLOR), (10, y)); y += 17

    y += 8
    surf.blit(small_font.render(
        f'Threshold={AXIS_THRESHOLD}  linear={JOG_LINEAR} m/s  '
        f'angular={JOG_ANGULAR} rad/s  period={JOG_PERIOD} s',
        True, (140,140,140)), (10, y)); y += 17

    # Reset hint
    surf.blit(small_font.render('[R] Reset box    [Q/Esc] Quit', True, (140,140,140)),
              (10, HEIGHT - 24))

    # Grid / floor reference
    for gx in range(-5, 6):
        x0 = ORIGIN[0] + gx * 0.1 * SCALE * VIEW_ROT[0, 0]
        y0 = ORIGIN[1] + gx * 0.1 * SCALE * VIEW_ROT[1, 0]
        pygame.draw.line(surf, (50, 50, 60),
                         (int(x0 - 0.5*SCALE*VIEW_ROT[0,1]),
                          int(y0 - 0.5*SCALE*VIEW_ROT[1,1])),
                         (int(x0 + 0.5*SCALE*VIEW_ROT[0,1]),
                          int(y0 + 0.5*SCALE*VIEW_ROT[1,1])), 1)


# ──────────────────────────────────────────────────────────────────────────────
def ros_spin(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    state = TeleopState()
    node = TeleopSubscriber(state)

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    pygame.init()
    surf = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption('SpaceMouse Teleop Box Visualizer')
    font       = pygame.font.SysFont('monospace', 16, bold=True)
    small_font = pygame.font.SysFont('monospace', 14)
    clock = pygame.time.Clock()

    jog_accum = 0.0   # accumulate time between jog steps
    last_steps = {}
    last_axis  = None

    running = True
    while running:
        dt_ms = clock.tick(60)
        dt_s  = dt_ms / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_r:
                    state.reset()

        # Apply jog at JOG_PERIOD rate
        jog_accum += dt_s
        if jog_accum >= JOG_PERIOD:
            jog_accum %= JOG_PERIOD
            last_axis, last_steps = state.jog_step()

        # Draw
        surf.fill(BG)
        pos, rot_mat = state.get_pose()
        draw_box(surf, pos, rot_mat)
        draw_hud(surf, font, small_font, state, last_steps, last_axis)
        pygame.display.flip()

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
