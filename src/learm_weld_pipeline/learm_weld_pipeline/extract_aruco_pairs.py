#!/usr/bin/env python3
"""
Extract four corner ArUco markers (pixel coords) and pair them with A4 mm coordinates.

Usage (portrait A4, origin bottom-left):
  python3 extract_aruco_pairs.py --image /path/to/snapshot.jpg

If your markers are inset from the sheet edges (e.g., 10 mm margin):
  python3 extract_aruco_pairs.py --image /path/to/snapshot.jpg --margin_mm 10

Landscape (long side horizontal):
  python3 extract_aruco_pairs.py --image /path/to/snapshot.jpg --orientation landscape

Outputs: /tmp/pairs.csv with columns: u,v,x_mm,y_mm
"""

import argparse
import sys
import os
import csv
import cv2
import numpy as np

# Try multiple dictionaries (common sets)
ARUCO_DICTS = [
    cv2.aruco.DICT_4X4_50,
    cv2.aruco.DICT_4X4_100,
    cv2.aruco.DICT_5X5_50,
    cv2.aruco.DICT_5X5_100,
    cv2.aruco.DICT_6X6_50,
    cv2.aruco.DICT_6X6_100,
]

def detect_four_markers(img_bgr):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    best = None
    for dict_id in ARUCO_DICTS:
        adict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(adict, params)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None or len(ids) < 4:
            continue
        # Keep the four that form the outermost rectangle (convex hull of marker centers)
        centers = np.array([c[0].mean(axis=0) for c in corners])  # (N,2)
        # Take the 4 extreme by convex hull area (fallback to first 4)
        try:
            hull_idx = cv2.convexHull(centers.astype(np.float32), returnPoints=False).flatten()
            hull_pts = centers[hull_idx]
        except Exception:
            hull_pts = centers
        # If more than 4, pick the 4 that maximize area via combinations (cheap heuristic)
        if len(hull_pts) >= 4:
            if len(hull_pts) > 4:
                # choose 4 farthest apart using simple greedy farthest-point sampling
                sel = []
                rest = list(range(len(hull_pts)))
                # start from point farthest from center
                center = hull_pts.mean(axis=0)
                first = max(rest, key=lambda i: np.linalg.norm(hull_pts[i]-center))
                sel.append(first); rest.remove(first)
                # add farthest from current set, 3 times
                for _ in range(3):
                    nxt = max(rest, key=lambda i: min(np.linalg.norm(hull_pts[i]-hull_pts[j]) for j in sel))
                    sel.append(nxt); rest.remove(nxt)
                picked = hull_pts[sel]
            else:
                picked = hull_pts
        else:
            picked = centers[:4]

        if best is None or len(picked)==4:
            best = picked  # (4,2)
            if len(picked)==4:
                break
    return best  # ndarray (4,2) or None

def order_corners_tl_tr_br_bl(pts):
    # pts: (4,2) ndarray
    pts = np.array(pts, dtype=float)
    s = pts.sum(axis=1)            # TL has smallest sum, BR largest
    diff = np.diff(pts, axis=1)    # TR has smallest (x - y), BL largest
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]
    return np.array([tl, tr, br, bl], dtype=float)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True, help="Path to snapshot image")
    ap.add_argument("--orientation", choices=["portrait","landscape"], default="portrait",
                    help="A4 orientation (portrait: 210w x 297h; landscape: 297w x 210h)")
    ap.add_argument("--margin_mm", type=float, default=0.0,
                    help="If markers are inset from edges, specify uniform margin in mm")
    ap.add_argument("--out_csv", default="/tmp/pairs.csv", help="Where to write u,v,x_mm,y_mm")
    args = ap.parse_args()

    if not os.path.exists(args.image):
        print(f"Image not found: {args.image}", file=sys.stderr)
        sys.exit(1)

    img = cv2.imread(args.image)
    if img is None:
        print(f"Failed to load image: {args.image}", file=sys.stderr)
        sys.exit(1)

    det = detect_four_markers(img)
    if det is None or len(det) != 4:
        print("Could not detect four outer ArUco markers. "
              "Try better lighting / larger markers.", file=sys.stderr)
        sys.exit(2)

    # Order the detected marker centers as TL, TR, BR, BL
    tl, tr, br, bl = order_corners_tl_tr_br_bl(det)

    # A4 dimensions (mm)
    if args.orientation == "portrait":
        W, H = 210.0, 297.0
    else:
        W, H = 297.0, 210.0

    m = max(0.0, float(args.margin_mm))
    # Real-world coordinates (origin at bottom-left, X right, Y up)
    # TL(0,H), TR(W,H), BR(W,0), BL(0,0) � then apply margin if markers are inset.
    x_tl, y_tl = m, H - m
    x_tr, y_tr = W - m, H - m
    x_br, y_br = W - m, m
    x_bl, y_bl = m, m

    # Image pixel coordinates (u,v) for those corners (source points)
    # Note: OpenCV uses (x=col=u, y=row=v)
    pairs = [
        (tl[0], tl[1], x_tl, y_tl),
        (tr[0], tr[1], x_tr, y_tr),
        (br[0], br[1], x_br, y_br),
        (bl[0], bl[1], x_bl, y_bl),
    ]

    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    with open(args.out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["u","v","x_mm","y_mm"])
        for r in pairs:
            w.writerow([f"{r[0]:.3f}", f"{r[1]:.3f}", f"{r[2]:.3f}", f"{r[3]:.3f}"])

    # Visualize (optional)
    vis = img.copy()
    for (u,v, xm, ym) in pairs:
        cv2.circle(vis, (int(round(u)), int(round(v))), 6, (0,255,0), 2)
        cv2.putText(vis, f"({xm:.0f},{ym:.0f})", (int(u)+8, int(v)-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
    preview = "/tmp/aruco_preview.jpg"
    cv2.imwrite(preview, vis)
    print(f"Wrote pairs to {args.out_csv}")
    print(f"Saved preview to {preview}")
    print("Order (TL, TR, BR, BL) with origin at bottom-left and A4 size applied.")

if __name__ == "__main__":
    main()

