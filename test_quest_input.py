#!/usr/bin/env python3
"""
Headless test for QuestInputThread logic.
Run this to verify the Quest controller drives the box correctly
before opening the full pygame visualizer.

Usage:
    python3 test_quest_input.py
    python3 test_quest_input.py --controller left
    python3 test_quest_input.py --rmat-reorder -2 -1 -3 4

Controls (in the terminal):
    Hold GRIP button on controller → box should move
    Joystick click                 → reset forward direction
    Ctrl-C                         → quit
"""

import sys
import time
import argparse
import numpy as np

# ── inline the same helpers from teleop_box_viz ──────────────────────────────

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

def _vec_to_reorder_mat(vec):
    n = len(vec)
    M = np.zeros((n,n))
    for i,v in enumerate(vec):
        M[i,int(abs(v))-1] = np.sign(v)
    return M

QUEST_POS_SCALE = 3.0

# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controller', choices=['right','left'], default='right')
    parser.add_argument('--rmat-reorder', nargs=4, type=int, default=[-2,-1,-3,4], metavar='N')
    args = parser.parse_args()

    try:
        from oculus_reader.reader import OculusReader
    except ImportError:
        print("ERROR: oculus_reader not installed.")
        sys.exit(1)

    cid      = 'r' if args.controller == 'right' else 'l'
    grip_key = cid.upper() + 'G'
    joy_key  = cid.upper() + 'J'
    trig_key = 'rightTrig' if cid == 'r' else 'leftTrig'
    global_to_env = _vec_to_reorder_mat(args.rmat_reorder)

    print(f"Connecting to Quest ({args.controller} controller)...")
    reader = OculusReader()
    print("Connected! Waiting for controller data...")
    print("Hold GRIP to move, joystick-click to reset forward. Ctrl-C to quit.\n")

    vr_to_global   = np.eye(4)
    reset_orient   = True
    prev_enabled   = False

    box_pos        = np.zeros(3)
    box_quat       = np.array([1.0, 0.0, 0.0, 0.0])
    box_origin_pos = None
    box_origin_quat= None
    vr_origin_pos  = None
    vr_origin_quat = None

    last_print = 0.0
    frame = 0

    try:
        while True:
            time.sleep(1.0 / 50)
            frame += 1

            poses, buttons = reader.get_transformations_and_buttons()
            if not poses or cid not in poses:
                continue

            raw      = np.asarray(poses[cid])
            enabled  = bool(buttons.get(grip_key, False))
            joy_held = bool(buttons.get(joy_key, False))
            gripper  = float(buttons.get(trig_key, [0.0])[0])
            toggled  = (enabled != prev_enabled)

            # orientation reset
            if reset_orient or joy_held:
                try:
                    vr_to_global = np.linalg.inv(raw)
                except np.linalg.LinAlgError:
                    vr_to_global = np.eye(4)
                    reset_orient = True
                else:
                    if joy_held or enabled:
                        reset_orient = False

            # clear origins on toggle
            if toggled:
                box_origin_pos = None

            prev_enabled = enabled

            if not enabled:
                if toggled:
                    print("Grip released — box frozen")
                continue

            # env-frame pose
            mat     = global_to_env @ vr_to_global @ raw
            vr_pos  = mat[:3, 3]
            vr_quat = _rmat_to_quat(mat[:3, :3])

            # set origins on first enabled tick
            if box_origin_pos is None:
                box_origin_pos  = box_pos.copy()
                box_origin_quat = box_quat.copy()
                vr_origin_pos   = vr_pos.copy()
                vr_origin_quat  = vr_quat.copy()
                print("Grip pressed — origin set. Move controller to move box.")
                continue

            # DROID displacement tracking
            target_pos  = box_origin_pos + (vr_pos - vr_origin_pos) * QUEST_POS_SCALE
            dR_vr       = _quat_mult(vr_quat, _quat_conj(vr_origin_quat))
            target_quat = _quat_mult(dR_vr, box_origin_quat)

            box_pos  = target_pos
            box_quat = target_quat

            # print at 2 Hz
            now = time.time()
            if now - last_print > 0.5:
                last_print = now
                disp = np.linalg.norm(vr_pos - vr_origin_pos)
                print(f"  box pos: x={box_pos[0]:+.4f}  y={box_pos[1]:+.4f}  z={box_pos[2]:+.4f} m"
                      f"   ctrl disp: {disp*100:+.1f} cm   grip={gripper:.2f}  frame={frame}")

    except KeyboardInterrupt:
        print("\nDone.")

if __name__ == '__main__':
    main()
