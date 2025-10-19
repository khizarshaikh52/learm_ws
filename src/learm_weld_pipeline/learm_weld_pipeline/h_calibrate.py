#!/usr/bin/env python3
import argparse, yaml, numpy as np

def load_points(pth):
    # CSV with headers: u,v,x_mm,y_mm  (pixel u,v) -> (base x,y in mm)
    rows = []
    with open(pth,'r') as f:
        for i, line in enumerate(f):
            if i == 0 and any(h in line for h in ['u','v','x','y']):  # skip header
                continue
            parts = line.strip().split(',')
            if len(parts) < 4: 
                continue
            u,v,x,y = map(float, parts[:4])
            rows.append((u,v,x,y))
    if len(rows) < 4:
        raise ValueError("Need at least 4 correspondences (u,v,x_mm,y_mm)")
    return rows

def dlt_homography(uv, xy):
    # Direct Linear Transform for H (planar)
    A = []
    for (u,v), (x,y) in zip(uv, xy):
        A.append([u, v, 1, 0, 0, 0, -x*u, -x*v, -x])
        A.append([0, 0, 0, u, v, 1, -y*u, -y*v, -y])
    A = np.asarray(A, dtype=float)
    _, _, VT = np.linalg.svd(A)
    h = VT[-1,:] / VT[-1,-1]
    H = h.reshape(3,3)
    return H

def reproj_rmse(H, uv, xy):
    e2 = []
    for (u,v),(x,y) in zip(uv,xy):
        p = np.array([u,v,1.0])
        q = H @ p
        q /= q[2]
        e2.append((q[0]-x)**2 + (q[1]-y)**2)
    return (np.mean(e2))**0.5

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pairs_csv', required=True,
                    help='CSV with columns: u,v,x_mm,y_mm')
    ap.add_argument('--yaml_out', required=True,
                    help='Path to write homography.yaml (update/overwrite)')
    ap.add_argument('--Z_work_mm', type=float, default=5.0)
    ap.add_argument('--step_mm', type=float, default=2.0)
    ap.add_argument('--corner_tol_mm', type=float, default=0.8)
    args = ap.parse_args()

    rows = load_points(args.pairs_csv)
    uv = [(r[0], r[1]) for r in rows]
    xy = [(r[2], r[3]) for r in rows]

    H = dlt_homography(uv, xy)
    rmse = reproj_rmse(H, uv, xy)
    print("Computed H (pixel->base mm):")
    print(H)
    print(f"Reprojection RMSE: {rmse:.3f} mm over {len(rows)} points")

    data = {
        'homography': {'H': [float(x) for x in H.flatten().tolist()]},
        'Z_work_mm': float(args.Z_work_mm),
        'tool_offset_mm': [0.0, 0.0, 30.0],
        'path': {'step_mm': float(args.step_mm), 'corner_tol_mm': float(args.corner_tol_mm)},
        'motion': {'speed_deg_s': 20.0, 'dwell_start_s': 0.3, 'dwell_end_s': 0.3},
        'joints': {'names': ["shoulder_pan","shoulder_lift","elbow1","elbow2","wrist_flex","wrist_roll","grip_left"]},
    }

    with open(args.yaml_out, 'w') as f:
        yaml.safe_dump(data, f)
    print(f"Wrote calibrated homography config to: {args.yaml_out}")

if __name__ == '__main__':
    main()
