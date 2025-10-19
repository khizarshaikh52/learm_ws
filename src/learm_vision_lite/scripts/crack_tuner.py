import cv2
import numpy as np

def _skeletonize(binary: np.ndarray) -> np.ndarray:
    """Thin a binary mask to ~1px skeleton using morphological thinning."""
    binary = (binary > 0).astype(np.uint8) * 255
    skel = np.zeros_like(binary)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    img = binary.copy()
    while True:
        eroded = cv2.erode(img, element)
        temp   = cv2.dilate(eroded, element)
        temp   = cv2.subtract(img, temp)
        skel   = cv2.bitwise_or(skel, temp)
        img    = eroded
        if cv2.countNonZero(img) == 0:
            break
    return skel

def detect_cracks(
    bgr: np.ndarray,
    *,
    blur: int = 3,
    clahe_clip: float = 2.0,
    block_size: int = 31,
    C: int = 7,
    min_area: int = 120,
    open_ksize: int = 3,
    do_skeleton: bool = True,
):
    """
    Dark-on-white crack detector.
    Returns: (mask_uint8, pretty_overlay_bgr, metrics_dict)
    metrics: {cx, cy, dx, dy, area_px, length_px}
    """
    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # normalize lighting
    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))
    nrm = clahe.apply(gray)
    if blur > 0:
        nrm = cv2.GaussianBlur(nrm, (blur | 1, blur | 1), 0)

    # cracks = dark ? invert threshold result
    th = cv2.adaptiveThreshold(
        nrm, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
        block_size | 1, C
    )

    # light cleanup, preserves skinny lines
    if open_ksize > 0:
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (open_ksize, open_ksize))
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, k, iterations=1)

    # remove tiny blobs
    num, labels, stats, _ = cv2.connectedComponentsWithStats(th, 8)
    mask = np.zeros_like(th)
    for i in range(1, num):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            mask[labels == i] = 255

    # thin (for nicer �length� and outlines)
    skel = _skeletonize(mask) if do_skeleton else mask

    # metrics
    M = cv2.moments(mask, binaryImage=True)
    if M["m00"] > 0:
        cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
    else:
        cx, cy = -1, -1
    center = (w // 2, h // 2)
    dx = cx - center[0] if cx >= 0 else 0
    dy = cy - center[1] if cy >= 0 else 0
    length_px = int(cv2.countNonZero(skel))
    area_px   = int(cv2.countNonZero(mask))

    # --- pretty overlay (thin outlines + soft blend) ---
    overlay = bgr.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)  # thin green outline
    if cx >= 0:
        cv2.circle(overlay, (cx, cy), 3, (0, 0, 255), -1)     # red = centroid
        cv2.circle(overlay, center, 3, (255, 0, 0), -1)       # blue = image center
        cv2.line(overlay, center, (cx, cy), (255, 0, 0), 1)   # offset line
    pretty = cv2.addWeighted(bgr, 0.8, overlay, 0.8, 0.0)

    # small, clean HUD
    font = cv2.FONT_HERSHEY_SIMPLEX
    txt1 = f"area={area_px}px  len~={length_px}px"
    txt2 = f"offset=({dx},{dy}) px"
    cv2.putText(pretty, txt1, (10, 20), font, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(pretty, txt1, (10, 20), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(pretty, txt2, (10, 40), font, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(pretty, txt2, (10, 40), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    metrics = {"cx": cx, "cy": cy, "dx": dx, "dy": dy, "area_px": area_px, "length_px": length_px}
    return mask, pretty, metrics
