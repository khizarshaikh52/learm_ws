from math import hypot

def douglas_peucker(points, eps):
    """Simplify polyline (list of (x,y)) with Douglas�Peucker."""
    if len(points) < 3:
        return points

    def dist(p, a, b):
        (x, y), (x1, y1), (x2, y2) = p, a, b
        dx, dy = x2 - x1, y2 - y1
        if dx == dy == 0:
            return hypot(x - x1, y - y1)
        t = ((x - x1) * dx + (y - y1) * dy) / float(dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        proj = (x1 + t * dx, y1 + t * dy)
        return hypot(x - proj[0], y - proj[1])

    def rec(pts):
        if len(pts) <= 2:
            return pts
        a, b = pts[0], pts[-1]
        idx, dmax = 0, 0.0
        for i, p in enumerate(pts[1:-1], start=1):
            d = dist(p, a, b)
            if d > dmax:
                idx, dmax = i, d
        if dmax > eps:
            left = rec(pts[:idx + 1])
            right = rec(pts[idx:])
            return left[:-1] + right
        else:
            return [a, b]

    return rec(points)

def resample_by_step(points, step):
    """Resample polyline with approximately constant spacing `step`."""
    if not points:
        return []
    out = [points[0]]
    acc = 0.0
    for i in range(1, len(points)):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]
        seglen = hypot(x2 - x1, y2 - y1)
        if seglen == 0:
            continue
        t = step - acc
        while t <= seglen:
            u = t / seglen
            out.append((x1 + u * (x2 - x1), y1 + u * (y2 - y1)))
            t += step
        acc = seglen - (t - step)
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out
