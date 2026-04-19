"""
Interactive corner picker.

Click the 4 table corners in order TL -> TR -> BR -> BL. World coordinates
are auto-assigned from TABLE_W_MM / TABLE_H_MM (TL = origin, +X to TR,
+Y to BL). After 4 clicks, press 'q' or Enter to compute and save the
homography.

Controls (in the image window):
    left-click    add the next corner (TL, TR, BR, BL in order)
    right-click   undo last point
    u             undo last point (keyboard)
    q / Enter     finish and compute H
    Esc           abort
"""

from pathlib import Path

import cv2
import numpy as np


IMAGE_PATH = Path("/home/mani/Repos/aura/demo_data/layup_demo/bulk_data/markers.jpg")
OUTPUT_DIR = Path("/home/mani/Repos/Any6D/pipeline_output")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Table dimensions in mm — update to match your table.
TABLE_W_MM = 1500.0   # TL -> TR
TABLE_H_MM = 750.0    # TL -> BL

PREVIEW_MAX_W = 1400
PREVIEW_MAX_H = 900

CORNER_LABELS = ["TL", "TR", "BR", "BL"]
CORNER_WORLD = [
    (0.0,          0.0),
    (TABLE_W_MM,   0.0),
    (TABLE_W_MM,   TABLE_H_MM),
    (0.0,          TABLE_H_MM),
]


class Picker:
    def __init__(self, bgr):
        self.bgr_full = bgr
        H, W = bgr.shape[:2]
        s = min(PREVIEW_MAX_W / W, PREVIEW_MAX_H / H, 1.0)
        self.scale = s
        self.preview = cv2.resize(bgr, (int(W * s), int(H * s)))
        self.corners = []   # list of (u_full, v_full)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.corners) >= 4:
                print("[click] already have 4 corners. Press 'u' to undo or "
                      "'q'/Enter to finish.")
                return
            u_full = x / self.scale
            v_full = y / self.scale
            lbl = CORNER_LABELS[len(self.corners)]
            xw, yw = CORNER_WORLD[len(self.corners)]
            self.corners.append((u_full, v_full))
            print(f"[click] {lbl} pixel=({u_full:.0f},{v_full:.0f}) "
                  f"-> world=({xw:.0f},{yw:.0f}) mm")
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.undo()

    def undo(self):
        if self.corners:
            dropped = self.corners.pop()
            print(f"[undo] removed {CORNER_LABELS[len(self.corners)]} "
                  f"({dropped[0]:.0f},{dropped[1]:.0f})")

    def render(self):
        vis = self.preview.copy()
        for i, (u, v) in enumerate(self.corners):
            pu, pv = int(u * self.scale), int(v * self.scale)
            cv2.circle(vis, (pu, pv), 10, (0, 255, 0), 2)
            xw, yw = CORNER_WORLD[i]
            cv2.putText(vis, f"{CORNER_LABELS[i]}({xw:.0f},{yw:.0f})",
                        (pu + 12, pv - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if len(self.corners) < 4:
            nxt = CORNER_LABELS[len(self.corners)]
            hint = f"Click {nxt} ({len(self.corners)}/4) | R-click/u: undo | q/Enter: finish"
        else:
            hint = "4/4 corners picked | R-click/u: undo | q/Enter: finish"
        cv2.putText(vis, hint, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        return vis


class Querier:
    """Second-phase UI: click anywhere, get world (x,y) via the homography."""
    def __init__(self, bgr, scale, H):
        self.bgr_full = bgr
        self.scale = scale
        self.H = H
        H_img, W_img = bgr.shape[:2]
        self.preview = cv2.resize(bgr, (int(W_img * scale), int(H_img * scale)))
        self.queries = []  # list of (u_full, v_full, x_mm, y_mm)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            u = x / self.scale
            v = y / self.scale
            pt = np.array([[[u, v]]], dtype=np.float32)
            xw, yw = cv2.perspectiveTransform(pt, self.H)[0, 0]
            self.queries.append((u, v, float(xw), float(yw)))
            print(f"[query] pixel=({u:.0f},{v:.0f}) -> world=({xw:.1f},{yw:.1f}) mm")
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.queries:
                dropped = self.queries.pop()
                print(f"[undo] removed query ({dropped[2]:.1f},{dropped[3]:.1f})")

    def render(self):
        vis = self.preview.copy()
        for (u, v, xw, yw) in self.queries:
            pu, pv = int(u * self.scale), int(v * self.scale)
            cv2.circle(vis, (pu, pv), 6, (0, 200, 255), -1)
            cv2.putText(vis, f"({xw:.0f},{yw:.0f})", (pu + 10, pv - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        # Draw warped 100mm grid so the user sees the mapped frame
        H_inv = np.linalg.inv(self.H)
        for x in np.arange(0, TABLE_W_MM + 1e-3, 100):
            ln = np.array([[[x, 0]], [[x, TABLE_H_MM]]], dtype=np.float32)
            px = cv2.perspectiveTransform(ln, H_inv)[..., :2] * self.scale
            p0 = tuple(px[0, 0].astype(int)); p1 = tuple(px[1, 0].astype(int))
            cv2.line(vis, p0, p1, (200, 200, 0), 1)
        for y in np.arange(0, TABLE_H_MM + 1e-3, 100):
            ln = np.array([[[0, y]], [[TABLE_W_MM, y]]], dtype=np.float32)
            px = cv2.perspectiveTransform(ln, H_inv)[..., :2] * self.scale
            p0 = tuple(px[0, 0].astype(int)); p1 = tuple(px[1, 0].astype(int))
            cv2.line(vis, p0, p1, (200, 200, 0), 1)
        cv2.putText(vis,
                    "Query: L-click point | R-click: undo | q/Enter: exit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)
        return vis


def run_query_mode(bgr, scale, H):
    q = Querier(bgr, scale, H)
    window = "query: click any point on the table for its (x,y) mm"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, q.on_mouse)
    print("\nQuery mode: click any point on the table to read its world (x,y).")
    print("  R-click to undo, 'q'/Enter to quit.\n")
    while True:
        cv2.imshow(window, q.render())
        key = cv2.waitKey(20) & 0xFF
        if key in (ord('q'), 13, 27):
            break
    cv2.destroyAllWindows()
    if q.queries:
        out = OUTPUT_DIR / "queried_points.txt"
        with open(out, "w") as f:
            f.write("# u_px v_px x_mm y_mm\n")
            for (u, v, xw, yw) in q.queries:
                f.write(f"{u:.2f} {v:.2f} {xw:.2f} {yw:.2f}\n")
        print(f"[save] {len(q.queries)} queried points -> {out}")


def solve_and_save(picker):
    if len(picker.corners) < 4:
        print(f"[abort] need 4 corners, have {len(picker.corners)}")
        return

    pixel_pts = np.asarray(picker.corners, dtype=np.float32)
    world_pts = np.asarray(CORNER_WORLD, dtype=np.float32)
    H, _ = cv2.findHomography(pixel_pts, world_pts, method=0)
    print(f"\n[H pixel->world_mm]\n{H}")

    reproj = cv2.perspectiveTransform(
        pixel_pts.reshape(-1, 1, 2), H
    ).reshape(-1, 2)
    err = np.linalg.norm(reproj - world_pts, axis=1)
    print(f"[residuals mm]: {np.round(err, 2)}  mean={err.mean():.2f} max={err.max():.2f}")

    vis = picker.bgr_full.copy()
    for (u, v), lbl, (xw, yw) in zip(picker.corners, CORNER_LABELS, CORNER_WORLD):
        cv2.circle(vis, (int(u), int(v)), 45, (0, 255, 0), 8)
        cv2.putText(vis, f"{lbl}({xw:.0f},{yw:.0f})", (int(u) + 55, int(v)),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 6)
    H_inv = np.linalg.inv(H)
    for x in np.arange(0, TABLE_W_MM + 1e-3, 100):
        ln = np.array([[[x, 0]], [[x, TABLE_H_MM]]], dtype=np.float32)
        px = cv2.perspectiveTransform(ln, H_inv).astype(int)
        cv2.line(vis, tuple(px[0, 0]), tuple(px[1, 0]), (200, 200, 0), 2)
    for y in np.arange(0, TABLE_H_MM + 1e-3, 100):
        ln = np.array([[[0, y]], [[TABLE_W_MM, y]]], dtype=np.float32)
        px = cv2.perspectiveTransform(ln, H_inv).astype(int)
        cv2.line(vis, tuple(px[0, 0]), tuple(px[1, 0]), (200, 200, 0), 2)

    cv2.imwrite(str(OUTPUT_DIR / "picked_overlay.jpg"),
                cv2.resize(vis, (1632, 1228)))
    np.save(OUTPUT_DIR / "H_pixel_to_world_mm.npy", H)
    with open(OUTPUT_DIR / "picked_correspondences.txt", "w") as f:
        f.write("# label u_px v_px x_mm y_mm\n")
        for (u, v), lbl, (xw, yw) in zip(picker.corners, CORNER_LABELS, CORNER_WORLD):
            f.write(f"{lbl} {u:.2f} {v:.2f} {xw:.2f} {yw:.2f}\n")
    print(f"[save] -> {OUTPUT_DIR}/H_pixel_to_world_mm.npy")
    print(f"[save] -> {OUTPUT_DIR}/picked_overlay.jpg")
    print(f"[save] -> {OUTPUT_DIR}/picked_correspondences.txt")

    run_query_mode(picker.bgr_full, picker.scale, H)


def main():
    bgr = cv2.imread(str(IMAGE_PATH))
    if bgr is None:
        raise FileNotFoundError(IMAGE_PATH)

    picker = Picker(bgr)
    window = "pick corners: TL, TR, BR, BL in order"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, picker.on_mouse)
    print(f"Table: {TABLE_W_MM:.0f} x {TABLE_H_MM:.0f} mm")
    print("Click corners in order: TL -> TR -> BR -> BL. 'q'/Enter to finish.\n")
    while True:
        cv2.imshow(window, picker.render())
        key = cv2.waitKey(20) & 0xFF
        if key in (ord('q'), 13):
            break
        if key == 27:
            cv2.destroyAllWindows()
            print("[abort]")
            return
        if key == ord('u'):
            picker.undo()
    cv2.destroyAllWindows()
    solve_and_save(picker)


if __name__ == "__main__":
    main()
