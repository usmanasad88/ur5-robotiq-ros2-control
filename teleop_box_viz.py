#!/usr/bin/env python3
"""
Teleop visualizer — moves/rotates a 3D box to validate controller input
without needing a real robot.

Supports two input modes:

  --input ros    (default) Subscribe to /ur5/teleop_delta — same as before.
                 Run alongside spacemouse_teleop_node or quest_teleop_node.

  --input quest  Read Quest 3S directly via OculusReader (no ROS needed).
                 Implements the same DROID P-controller as quest_rtde_teleop:
                   target = box_origin + (vr_pos - vr_origin)
                 Box tracks the controller displacement from where grip was pressed.

Usage:
    python3 teleop_box_viz.py                  # ROS mode (spacemouse)
    python3 teleop_box_viz.py --input quest    # Quest direct mode
    python3 teleop_box_viz.py --input quest --controller left
    python3 teleop_box_viz.py --input quest --rmat-reorder -2 -1 -3 4

Controls (both modes):
    Grip (Quest) / any delta (ROS)   move box
    Joystick click (Quest)           reset forward direction
    R                                reset box to origin
    Q / Esc                          quit
"""

import sys
import math
import threading
import argparse
import numpy as np



# ── Rotation helpers ──────────────────────────────────────────────────────────

def _rotvec_to_mat(rvec):
    angle = np.linalg.norm(rvec)
    if angle < 1e-10:
        return np.eye(3)
    axis = rvec / angle
    K = np.array([[ 0,       -axis[2],  axis[1]],
                  [ axis[2],  0,       -axis[0]],
                  [-axis[1],  axis[0],  0      ]])
    return np.eye(3) + math.sin(angle)*K + (1-math.cos(angle))*(K@K)


def _euler_xyz_to_mat(r, p, y):
    return (_rotvec_to_mat(np.array([r,0,0])) @
            _rotvec_to_mat(np.array([0,p,0])) @
            _rotvec_to_mat(np.array([0,0,y])))


def _mat_to_euler_xyz(R):
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
    return (_euler_xyz_to_mat(math.radians(-25), 0, 0) @
            _euler_xyz_to_mat(0, math.radians(30), 0))


# ── Quest rotation helpers (from quest_rtde_teleop) ──────────────────────────

def _rmat_to_quat(R):
    trace = R[0,0]+R[1,1]+R[2,2]
    if trace > 0:
        s = 0.5/np.sqrt(trace+1.0)
        return np.array([0.25/s,(R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s])
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s = 2.0*np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
        return np.array([(R[2,1]-R[1,2])/s,0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s])
    elif R[1,1]>R[2,2]:
        s = 2.0*np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
        return np.array([(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s])
    else:
        s = 2.0*np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
        return np.array([(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s])


def _quat_mult(q1, q2):
    w1,x1,y1,z1=q1; w2,x2,y2,z2=q2
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2,
                     w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2])


def _quat_conj(q):
    return np.array([q[0],-q[1],-q[2],-q[3]])


def _quat_to_mat(q):
    w,x,y,z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def _vec_to_reorder_mat(vec):
    n = len(vec)
    M = np.zeros((n,n))
    for i,v in enumerate(vec):
        M[i,int(abs(v))-1] = np.sign(v)
    return M


# ── Display constants ─────────────────────────────────────────────────────────

WIDTH, HEIGHT = 900, 650
BG            = (30,  30,  40)
EDGE_COLOR    = (200, 230, 255)
AXIS_COLORS   = [(220,60,60),(60,200,60),(60,100,220)]
TEXT_COLOR    = (230, 230, 230)
WARN_COLOR    = (255, 180,  50)
ZERO_COLOR    = (100, 200, 100)

SCALE  = 800
ORIGIN = np.array([WIDTH//2, HEIGHT//2-30], dtype=float)

_W,_H,_D = 0.15, 0.10, 0.06
BOX_CORNERS = np.array([
    [-_W,-_H,-_D],[+_W,-_H,-_D],[+_W,+_H,-_D],[-_W,+_H,-_D],
    [-_W,-_H,+_D],[+_W,-_H,+_D],[+_W,+_H,+_D],[-_W,+_H,+_D],
])/2

BOX_EDGES = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7)]
BOX_FACES = [([0,1,2,3],(60,120,180)),([4,5,6,7],(80,160,220)),
             ([0,1,5,4],(50,100,160)),([2,3,7,6],(70,140,200)),
             ([1,2,6,5],(90,170,230)),([0,3,7,4],(40, 90,150))]
AXIS_ARROWS = np.array([[0.12,0,0],[0,0.12,0],[0,0,0.12]])

VIEW_ROT = _make_view_rot()

# Jog params (ROS mode)
JOG_LINEAR    = 0.05
JOG_ANGULAR   = 0.3
JOG_PERIOD    = 0.05
AXIS_THRESHOLD = 0.05

# Quest direct mode — how much the box moves per metre of controller movement
QUEST_POS_SCALE = 3.0   # amplify so small wrist movements are visible
QUEST_ROT_SCALE = 1.0


# ── Shared box state ──────────────────────────────────────────────────────────

class BoxState:
    def __init__(self):
        self._lock = threading.Lock()
        self.pos  = np.zeros(3)
        self.rot  = np.eye(3)
        self.last_update = None   # for status display

    def set_pose(self, pos, rot):
        with self._lock:
            self.pos  = np.array(pos)
            self.rot  = np.array(rot)
            self.last_update = _ticks()

    def apply_delta(self, dp, dr_mat):
        with self._lock:
            self.pos += dp
            self.rot  = dr_mat @ self.rot
            self.last_update = _ticks()

    def get(self):
        with self._lock:
            return self.pos.copy(), self.rot.copy()

    def reset(self):
        with self._lock:
            self.pos = np.zeros(3)
            self.rot = np.eye(3)


def _ticks():
    import pygame; return pygame.time.get_ticks()


# ── ROS input thread ──────────────────────────────────────────────────────────

class RosInputThread:
    """Subscribes to /ur5/teleop_delta and drives the box (original behaviour)."""

    def __init__(self, box: BoxState):
        import rclpy                              # type: ignore[import]
        from rclpy.node import Node               # type: ignore[import]

        self._box       = box
        self._axis      = None
        self._jog_accum = 0.0

        rclpy.init()

        # Build the subscriber node without an inner class to avoid linter
        # complaints about the non-standard `self_` parameter name.
        node = Node('teleop_box_viz')             # type: ignore[call-arg]

        def _cb(msg):                             # noqa: ANN001
            self._axis = msg
            box.last_update = _ticks()

        # Import the message type lazily so the linter doesn't flag it as
        # unresolved (the ROS install path isn't on the type-checker's path).
        import importlib
        PoseDelta = importlib.import_module('ur5_teleop_msgs.msg').PoseDelta

        node.create_subscription(PoseDelta, '/ur5/teleop_delta', _cb, 10)
        node.get_logger().info('Subscribed to /ur5/teleop_delta')

        self._node = node

        self._thread = threading.Thread(
            target=lambda: rclpy.spin(self._node), daemon=True)
        self._thread.start()

    def tick(self, dt_s: float):
        """Call each frame; returns (info_dict, mode_label)."""
        ax = self._axis
        if ax is None:
            return {}, 'ros'

        self._jog_accum += dt_s
        if self._jog_accum < JOG_PERIOD:
            return {'axis': ax}, 'ros'
        self._jog_accum %= JOG_PERIOD

        thr = AXIS_THRESHOLD
        dt  = JOG_PERIOD
        sx  = ax.dx    * JOG_LINEAR  * dt if abs(ax.dx)    > thr else 0.0
        sy  = ax.dy    * JOG_LINEAR  * dt if abs(ax.dy)    > thr else 0.0
        sz  = ax.dz    * JOG_LINEAR  * dt if abs(ax.dz)    > thr else 0.0
        sr  = ax.droll * JOG_ANGULAR * dt if abs(ax.droll) > thr else 0.0
        sp  = ax.dpitch* JOG_ANGULAR * dt if abs(ax.dpitch)> thr else 0.0
        sy2 = ax.dyaw  * JOG_ANGULAR * dt if abs(ax.dyaw)  > thr else 0.0

        dr = _euler_xyz_to_mat(sr, sp, sy2)
        self._box.apply_delta(np.array([sx,sy,sz]), dr)
        return {'axis': ax}, 'ros'

    def shutdown(self):
        pass


# ── Quest direct input thread ─────────────────────────────────────────────────

class QuestInputThread:
    """
    Reads Quest 3S via OculusReader and applies DROID-style origin tracking
    directly to the box — same logic as quest_rtde_teleop._control_loop()
    but the box replaces the robot TCP.

    target_pos  = box_origin + (vr_pos  - vr_origin)
    target_quat = dR_vr      * box_origin_quat
    Box snaps to target instantly (no P-controller delay in sim).
    """

    def __init__(self, box: BoxState, controller: str = 'right', rmat_reorder=None):
        try:
            from oculus_reader.reader import OculusReader
        except ImportError:
            print("ERROR: oculus_reader not found.")
            print("  pip install git+https://github.com/rail-berkeley/oculus_reader.git")
            sys.exit(1)

        self._box = box
        self.cid      = 'r' if controller == 'right' else 'l'
        self.grip_key = self.cid.upper() + 'G'
        self.joy_key  = self.cid.upper() + 'J'
        self.trig_key = 'rightTrig' if self.cid == 'r' else 'leftTrig'

        rmat_reorder = rmat_reorder or [-2,-1,-3,4]
        self.global_to_env = _vec_to_reorder_mat(rmat_reorder)

        # shared state
        self._lock            = threading.Lock()
        self._poses           = {}
        self._buttons         = {}
        self._enabled         = False
        self._vr_to_global    = np.eye(4)
        self._reset_orient    = True

        self._box_origin_pos  = None
        self._box_origin_quat = None
        self._vr_origin_pos   = None
        self._vr_origin_quat  = None

        self.gripper_val = 0.0
        self.status      = 'Connecting to Quest via ADB ...'

        self._reader = OculusReader()
        self.status  = 'Quest connected — hold GRIP to move box'

        self._thread = threading.Thread(
            target=self._loop, daemon=True, name='quest_reader')
        self._thread.start()

    def _loop(self, hz=50):
        while True:
            __import__('time').sleep(1.0/hz)
            try:
                poses, buttons = self._reader.get_transformations_and_buttons()
            except Exception:
                continue
            if not poses:
                continue

            with self._lock:
                prev_en  = self._enabled
                now_en   = bool(buttons.get(self.grip_key, False))
                toggled  = prev_en != now_en

                # orientation alignment
                if self._reset_orient or buttons.get(self.joy_key, False):
                    raw  = np.asarray(poses.get(self.cid, np.eye(4)))
                    stop = buttons.get(self.joy_key, False) or now_en
                    if stop:
                        self._reset_orient = False
                    try:
                        self._vr_to_global = np.linalg.inv(raw)
                    except np.linalg.LinAlgError:
                        self._vr_to_global = np.eye(4)
                        self._reset_orient = True

                if toggled:
                    self._box_origin_pos  = None
                    self._box_origin_quat = None
                    self._vr_origin_pos   = None
                    self._vr_origin_quat  = None

                self._poses   = poses
                self._buttons = buttons
                self._enabled = now_en
                self.gripper_val = float(buttons.get(self.trig_key, [0.0])[0])
                self._box.last_update = _ticks()

    def tick(self, _dt_s: float):
        """Apply one control tick; returns (info_dict, 'quest')."""
        with self._lock:
            poses        = dict(self._poses)
            buttons      = dict(self._buttons)
            enabled      = self._enabled
            vr_to_global = self._vr_to_global.copy()
            origin_ready = self._box_origin_pos is not None
            gripper      = self.gripper_val

        if not enabled:
            self._box_origin_pos = None   # will re-baseline on next enable
            return {'enabled': False, 'gripper': gripper}, 'quest'

        if self.cid not in poses:
            return {'enabled': True, 'gripper': gripper}, 'quest'

        # VR pose in env frame (DROID _process_reading)
        raw    = np.asarray(poses[self.cid])
        mat    = self.global_to_env @ vr_to_global @ raw
        vr_pos  = mat[:3,3]
        vr_quat = _rmat_to_quat(mat[:3,:3])

        # Set origins on first enabled tick
        if not origin_ready:
            box_pos, box_rot = self._box.get()
            box_quat = _rmat_to_quat(box_rot)
            with self._lock:
                self._box_origin_pos  = box_pos.copy()
                self._box_origin_quat = box_quat.copy()
                self._vr_origin_pos   = vr_pos.copy()
                self._vr_origin_quat  = vr_quat.copy()
            return {'enabled': True, 'gripper': gripper}, 'quest'

        with self._lock:
            bo_pos  = self._box_origin_pos.copy()
            bo_quat = self._box_origin_quat.copy()
            vo_pos  = self._vr_origin_pos.copy()
            vo_quat = self._vr_origin_quat.copy()

        # DROID displacement tracking: box snaps to target (instant in sim)
        target_pos  = bo_pos + (vr_pos - vo_pos) * QUEST_POS_SCALE
        dR_vr       = _quat_mult(vr_quat, _quat_conj(vo_quat))
        target_quat = _quat_mult(dR_vr, bo_quat)
        target_rot  = _quat_to_mat(target_quat)

        self._box.set_pose(target_pos, target_rot)

        displacement = np.linalg.norm(vr_pos - vo_pos)
        return {
            'enabled':      True,
            'displacement': displacement,
            'target_pos':   target_pos,
            'gripper':      gripper,
        }, 'quest'

    def shutdown(self):
        pass


# ── Drawing ───────────────────────────────────────────────────────────────────

def project(pts3d, pos, rot, view):
    p  = (rot @ pts3d.T).T + pos
    pv = (view @ p.T).T
    return pv[:,:2]*SCALE + ORIGIN, pv[:,2]


def draw_box(surf, pos, rot):
    cs, depths = project(BOX_CORNERS, pos, rot, VIEW_ROT)
    faces = sorted([(np.mean(depths[v]), v, c) for v,c in BOX_FACES])
    for _,v,c in faces:
        pygame.draw.polygon(surf, c, [(int(cs[i,0]),int(cs[i,1])) for i in v])
    for a,b in BOX_EDGES:
        pygame.draw.line(surf, EDGE_COLOR,
                         (int(cs[a,0]),int(cs[a,1])),
                         (int(cs[b,0]),int(cs[b,1])), 1)
    o,_ = project(np.array([[0,0,0]]), pos, rot, VIEW_ROT)
    for i,arrow in enumerate(AXIS_ARROWS):
        t,_ = project(np.array([arrow]), pos, rot, VIEW_ROT)
        pygame.draw.line(surf, AXIS_COLORS[i],
                         (int(o[0,0]),int(o[0,1])),
                         (int(t[0,0]),int(t[0,1])), 2)


def draw_hud(surf, font, sfont, box: BoxState, info: dict, mode: str):
    import pygame as pg
    now = pg.time.get_ticks()
    y = 10

    title = 'Quest Controller Viz' if mode == 'quest' else 'SpaceMouse Teleop Viz'
    surf.blit(font.render(title, True, TEXT_COLOR), (10, y)); y += 30

    # status
    if box.last_update is None:
        stxt, scol = 'Waiting for input ...', WARN_COLOR
    elif now - box.last_update > 1000:
        stxt, scol = 'No data (controller idle?)', WARN_COLOR
    else:
        stxt, scol = 'Receiving data', ZERO_COLOR
    surf.blit(sfont.render(stxt, True, scol), (10, y)); y += 22

    if mode == 'quest':
        enabled = info.get('enabled', False)
        gripper = info.get('gripper', 0.0)
        disp    = info.get('displacement', None)

        col = ZERO_COLOR if enabled else WARN_COLOR
        surf.blit(sfont.render(
            f'Grip: {"ENABLED — robot moving" if enabled else "release — holding still"}',
            True, col), (10, y)); y += 18

        # Gripper bar
        bar_w = 200
        filled = int(bar_w * gripper)
        pg.draw.rect(surf, (60,60,70), (10, y, bar_w, 14))
        pg.draw.rect(surf, (100,220,100) if gripper > 0.1 else (80,80,90),
                     (10, y, filled, 14))
        surf.blit(sfont.render(f' Trigger (gripper): {gripper:.2f}',
                               True, TEXT_COLOR), (10+bar_w+6, y)); y += 20

        if disp is not None:
            surf.blit(sfont.render(
                f'Controller displacement: {disp*100:+.1f} cm',
                True, TEXT_COLOR), (10, y)); y += 18

        tp = info.get('target_pos')
        if tp is not None:
            surf.blit(sfont.render(
                f'Target: x={tp[0]:+.4f}  y={tp[1]:+.4f}  z={tp[2]:+.4f} m',
                True, (160,160,160)), (10, y)); y += 18

        y += 4
        surf.blit(sfont.render(
            f'Joystick click  = reset forward direction',
            True, (120,120,120)), (10, y)); y += 16
        surf.blit(sfont.render(
            f'rmat_reorder controls axis mapping (see --help)',
            True, (100,100,100)), (10, y)); y += 16
    else:
        # ROS mode: show raw axis values (original HUD)
        ax = info.get('axis')
        if ax is not None:
            rows = [('dx',ax.dx,'x'),('dy',ax.dy,'y'),('dz',ax.dz,'z'),
                    ('droll',ax.droll,'r'),('dpitch',ax.dpitch,'p'),('dyaw',ax.dyaw,'Y')]
            for name,val,_ in rows:
                above = abs(val) > AXIS_THRESHOLD
                col = TEXT_COLOR if above else (100,100,100)
                surf.blit(sfont.render(
                    f'{name:<8} {val:+.3f}  {"▶" if above else " "}',
                    True, col), (10, y)); y += 16

    y += 6
    pos, rot = box.get()
    euler = _mat_to_euler_xyz(rot)
    surf.blit(sfont.render(
        f'Box pos: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f} m',
        True, TEXT_COLOR), (10, y)); y += 17
    surf.blit(sfont.render(
        f'Box rot: r={euler[0]:+.1f}°  p={euler[1]:+.1f}°  y={euler[2]:+.1f}°',
        True, TEXT_COLOR), (10, y))

    # floor grid
    for gx in range(-5, 6):
        x0 = ORIGIN[0] + gx*0.1*SCALE*VIEW_ROT[0,0]
        y0 = ORIGIN[1] + gx*0.1*SCALE*VIEW_ROT[1,0]
        pygame.draw.line(surf, (50,50,60),
                         (int(x0-0.5*SCALE*VIEW_ROT[0,1]),int(y0-0.5*SCALE*VIEW_ROT[1,1])),
                         (int(x0+0.5*SCALE*VIEW_ROT[0,1]),int(y0+0.5*SCALE*VIEW_ROT[1,1])),1)

    pygame.draw.line(surf, (60,60,70), (0, HEIGHT-28), (WIDTH, HEIGHT-28), 1)
    surf.blit(sfont.render('[R] Reset box    [Q/Esc] Quit',
                           True, (120,120,120)), (10, HEIGHT-22))


# ── Main ──────────────────────────────────────────────────────────────────────

import pygame   # noqa: E402  (after constants so SCALE etc are defined)


def main():
    parser = argparse.ArgumentParser(
        description='Teleop box visualizer — validate controller input without robot')
    parser.add_argument('--input', choices=['ros','quest'], default='ros',
                        help='Input source: ros (default) or quest')
    parser.add_argument('--controller', choices=['right','left'], default='right',
                        help='Quest controller side (quest mode only)')
    parser.add_argument('--rmat-reorder', nargs=4, type=int, default=[-2,-1,-3,4],
                        metavar='N',
                        help='Axis reorder for Quest (default: -2 -1 -3 4). '
                             'Flip sign to negate, swap indices to swap axes.')
    args = parser.parse_args()

    pygame.init()
    surf  = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(
        'Quest VR Box Visualizer' if args.input == 'quest'
        else 'SpaceMouse Teleop Box Visualizer')
    font   = pygame.font.SysFont('monospace', 16, bold=True)
    sfont  = pygame.font.SysFont('monospace', 14)
    clock  = pygame.time.Clock()

    box = BoxState()

    if args.input == 'quest':
        inp = QuestInputThread(box, args.controller, args.rmat_reorder)
    else:
        inp = RosInputThread(box)

    last_info = {}
    last_mode = args.input
    running   = True

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
                    box.reset()

        last_info, last_mode = inp.tick(dt_s)

        surf.fill(BG)
        pos, rot = box.get()
        draw_box(surf, pos, rot)
        draw_hud(surf, font, sfont, box, last_info, last_mode)
        pygame.display.flip()

    inp.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
