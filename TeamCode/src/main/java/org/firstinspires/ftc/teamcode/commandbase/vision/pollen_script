# ============================================================================
#  POLLEN DETECTOR  —  Limelight 3A Python SnapScript pipeline (FTC BIOBUZZ)
#  Team 23405 "Crash Out!"
#
#  Shape-first, color-last:
#    1. Downscale + normalise illumination (CLAHE on luminance)
#    2. Build two "structure" images: luminance and relative chroma
#    3. Arc/circle candidates from gradient spikes (Hough gradient, both images)
#    4. Verify each candidate:
#         a. edge support  – % of the perimeter with strong, RADIAL gradient
#         b. hole texture  – dark blobs inside the ball, relative to the ball
#         c. (optional) size-vs-distance check from camera geometry
#    5. Non-max suppression (keeps touching/overlapping balls, drops duplicates)
#    6. Classify with color only AFTER detection (gray-world white balance +
#       median hue of the ball's surface, holes excluded)
#
#  Nothing in steps 1-5 uses a fixed color range, and every threshold is
#  relative to the local image, so it survives venue lighting changes.
#
#  llpython output (up to 32 doubles):
#    [0]          = number of balls reported (max MAX_OUT)
#    then per ball: tx_deg, ty_deg, radius_px(full-res), class, score
#    class: 0 = POLLEN (yellow), 1 = OTHER (e.g. NECTAR)
#    Sorted: pollen first, then by score.
# ============================================================================

import math
import cv2
import numpy as np

# ------------------------------- TUNABLES ----------------------------------
PROC_W          = 400      # internal processing width (px). 320 = faster, 480 = sees farther
MIN_R           = 4        # min ball radius at PROC_W (px)
MAX_R           = 100      # max ball radius at PROC_W (px)

# Hough (arc candidate) settings
HOUGH_DP        = 1.5
HOUGH_P1_ALT    = 180      # Scharr/Canny high threshold for HOUGH_GRADIENT_ALT
HOUGH_P2_ALT    = 0.55     # circle "perfectness" 0..1 (keep permissive; verification filters)
HOUGH_P1        = 100      # fallback HOUGH_GRADIENT settings (older OpenCV)
HOUGH_P2        = 16
MAX_CANDIDATES  = 30

# Edge-support verification
N_SAMPLES       = 48       # points sampled around the perimeter
ALIGN_MIN       = 0.75     # |cos| between gradient and radial direction
EDGE_REL        = 1.8      # edge must be this x stronger than the local median gradient
SUPPORT_MIN     = 0.40     # min fraction of perimeter supported (0.4 tolerates ~half occlusion)
CONTRAST_MIN    = 12       # min inside-vs-outside difference (grey levels, on the normalised images)

CHROMA_GAIN     = 400.0    # scales the chroma/brightness image into 0..255

# Hole verification (the pollen's holes)
HOLE_CHECK_MIN_R = 10      # below this radius holes aren't resolvable -> neutral score
HOLES_FULL       = 3       # this many holes found -> full hole score
HOLE_ABS_MIN     = 8       # min black-hat response (grey levels) to count as a hole

# Final scoring
W_SUPPORT       = 0.6
W_HOLES         = 0.4
SCORE_MIN       = 0.45

# Color classification (OpenCV hue is 0..180; yellow ~ 20-35)
POLLEN_HUE      = 27
HUE_TOL         = 12
SAT_MIN         = 70

# Optional geometry check (turn on once you've measured your mount)
USE_GEOMETRY    = False
CAM_HEIGHT_IN   = 8.0      # lens height above floor
CAM_PITCH_DEG   = 20.0     # positive = tilted DOWN
BALL_RADIUS_IN  = 1.4      # POLLEN is 2.8 in diameter
GEOM_TOL        = (0.6, 1.8)

# Camera
HFOV_DEG        = 54.5     # Limelight 3A horizontal FOV (verify against the LL docs)

MAX_OUT         = 6        # 1 + 6*5 = 31 doubles, fits the 32-double llpython limit
DRAW            = True
# ---------------------------------------------------------------------------

CLAHE = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
_ANG = np.linspace(0.0, 2.0 * np.pi, N_SAMPLES, endpoint=False)
COS = np.cos(_ANG).astype(np.float32)
SIN = np.sin(_ANG).astype(np.float32)
HAS_ALT = hasattr(cv2, "HOUGH_GRADIENT_ALT")


# ------------------------------- helpers -----------------------------------
def find_circles(img):
    """Arc/circle candidates from gradient spikes."""
    if HAS_ALT:
        c = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT_ALT, dp=HOUGH_DP,
                             minDist=MIN_R, param1=HOUGH_P1_ALT,
                             param2=HOUGH_P2_ALT, minRadius=MIN_R,
                             maxRadius=MAX_R)
    else:
        c = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=HOUGH_DP,
                             minDist=MIN_R, param1=HOUGH_P1,
                             param2=HOUGH_P2, minRadius=MIN_R,
                             maxRadius=MAX_R)
    if c is None:
        return []
    return [tuple(map(float, x)) for x in c[0][:MAX_CANDIDATES]]


def edge_support(cx, cy, r, gx, gy, mag, W, H):
    """Fraction of the perimeter where the gradient is strong AND points radially.
    Searches r-2..r+2 and returns (best_fraction, refined_radius)."""
    x0, x1 = max(0, int(cx - r - 3)), min(W, int(cx + r + 4))
    y0, y1 = max(0, int(cy - r - 3)), min(H, int(cy + r + 4))
    if x1 - x0 < 3 or y1 - y0 < 3:
        return 0.0, r
    ref = float(np.median(mag[y0:y1, x0:x1])) + 1.0   # local, lighting-relative reference

    best, best_r = 0.0, r
    for dr in (-2, -1, 0, 1, 2):
        rr = r + dr
        if rr < 3:
            continue
        px = cx + rr * COS
        py = cy + rr * SIN
        inb = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        if inb.sum() < N_SAMPLES // 3:
            continue
        xs = px[inb].astype(np.int32)
        ys = py[inb].astype(np.int32)
        vx, vy, m = gx[ys, xs], gy[ys, xs], mag[ys, xs] + 1e-6
        align = np.abs(vx * COS[inb] + vy * SIN[inb]) / m
        on = (align > ALIGN_MIN) & (m > EDGE_REL * ref)
        frac = float(on.mean())
        if frac > best:
            best, best_r = frac, rr
    return best, best_r


def ring_contrast(cx, cy, r, imgs, W, H):
    """Median just inside the rim vs just outside, on each normalised image.
    Kills 'circles' Hough hallucinates in flat, noisy floor."""
    best = 0.0
    for img in imgs:
        vals = []
        for k in (0.72, 0.82, 1.2, 1.32):
            px = np.clip(cx + k * r * COS, 0, W - 1).astype(np.int32)
            py = np.clip(cy + k * r * SIN, 0, H - 1).astype(np.int32)
            vals.append(img[py, px].astype(np.float32))
        inner = np.concatenate(vals[:2])
        outer = np.concatenate(vals[2:])
        best = max(best, abs(float(np.median(inner)) - float(np.median(outer))))
    return best


def suppress(dets):
    """Drop duplicates and circles nested inside a better ball (holes, highlights),
    keep touching/overlapping neighbours."""
    dets.sort(key=lambda d: -d[0])
    kept = []
    for d in dets:
        bad = False
        for k in kept:
            dist = math.hypot(d[1] - k[1], d[2] - k[2])
            if dist < 0.5 * max(d[3], k[3]):
                bad = True
            elif d[3] < k[3] and dist < 0.9 * k[3]:
                bad = True   # centre sits inside a better, bigger ball: hole/shadow/highlight arc
            if bad:
                break
        if not bad:
            kept.append(d)
    return kept


def hole_analysis(cx, cy, r, L, W, H):
    """Find the holes inside the ball: dark blobs relative to the ball surface.
    Returns (score 0..1, hole_mask_or_None, (x0, y0))."""
    if r < HOLE_CHECK_MIN_R:
        return 0.5, None, (0, 0)
    ri = int(r * 0.85)
    x0, x1 = max(0, int(cx - ri)), min(W, int(cx + ri + 1))
    y0, y1 = max(0, int(cy - ri)), min(H, int(cy + ri + 1))
    patch = L[y0:y1, x0:x1]
    if patch.size < 25:
        return 0.0, None, (x0, y0)

    disk = np.zeros(patch.shape, np.uint8)
    cv2.circle(disk, (int(cx - x0), int(cy - y0)), ri, 255, -1)

    k = max(3, int(r * 0.45) | 1)
    se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    bh = cv2.morphologyEx(patch, cv2.MORPH_BLACKHAT, se)   # dark spots vs surroundings

    vals = bh[disk > 0].astype(np.float32)
    surf = float(np.median(patch[disk > 0]))
    thr = max(HOLE_ABS_MIN, 0.08 * surf, float(vals.mean() + 1.5 * vals.std()))
    bw = ((bh > thr) & (disk > 0)).astype(np.uint8)

    n, _, stats, _ = cv2.connectedComponentsWithStats(bw, connectivity=8)
    amin = math.pi * (0.06 * r) ** 2
    amax = math.pi * (0.40 * r) ** 2
    holes = sum(1 for i in range(1, n) if amin <= stats[i, cv2.CC_STAT_AREA] <= amax)
    return min(1.0, holes / float(HOLES_FULL)), bw, (x0, y0)


def geometry_ok(cy, r, H, f):
    """Is the radius plausible for a floor ball at this image row?"""
    ang = math.radians(CAM_PITCH_DEG) + math.atan((cy - H / 2.0) / f)
    if ang <= math.radians(1.0):
        return True          # at/above horizon: can't judge (airborne ball), don't reject
    d = (CAM_HEIGHT_IN - BALL_RADIUS_IN) / math.sin(ang)
    if d <= 0:
        return True
    r_exp = f * BALL_RADIUS_IN / d
    ratio = r / max(r_exp, 1e-3)
    return GEOM_TOL[0] <= ratio <= GEOM_TOL[1]


def classify(cx, cy, r, small_wb, hole_mask, hole_off, W, H):
    """Color used ONLY to label a verified ball. Returns (class, hue, sat)."""
    rs = max(2, int(r * 0.7))
    x0, x1 = max(0, int(cx - rs)), min(W, int(cx + rs + 1))
    y0, y1 = max(0, int(cy - rs)), min(H, int(cy + rs + 1))
    patch = small_wb[y0:y1, x0:x1]
    if patch.size == 0:
        return 1, 0.0, 0.0
    m = np.zeros(patch.shape[:2], np.uint8)
    cv2.circle(m, (int(cx - x0), int(cy - y0)), rs, 255, -1)
    if hole_mask is not None:            # "fill in" the holes: exclude them from sampling
        hx, hy = hole_off
        sub = np.zeros_like(m)
        ys0, xs0 = y0 - hy, x0 - hx
        hm = hole_mask[max(0, ys0):max(0, ys0) + m.shape[0],
                       max(0, xs0):max(0, xs0) + m.shape[1]]
        sub[:hm.shape[0], :hm.shape[1]] = hm
        m[sub > 0] = 0
    px = patch[m > 0]
    if len(px) < 5:
        return 1, 0.0, 0.0
    hsv = cv2.cvtColor(px.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    hue = float(np.median(hsv[:, 0]))
    sat = float(np.median(hsv[:, 1]))
    is_pollen = abs(hue - POLLEN_HUE) <= HUE_TOL and sat >= SAT_MIN
    return (0 if is_pollen else 1), hue, sat


# ------------------------------- pipeline ----------------------------------
def runPipeline(image, llrobot):
    h0, w0 = image.shape[:2]
    s = PROC_W / float(w0)
    small = cv2.resize(image, (PROC_W, int(round(h0 * s))), interpolation=cv2.INTER_AREA)
    H, W = small.shape[:2]
    f = (W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))

    # 1. illumination normalisation
    lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
    L = CLAHE.apply(lab[:, :, 0])
    L = cv2.GaussianBlur(L, (5, 5), 1.2)

    # 2. chroma-per-brightness image (a saturation-like measure that stays put when
    #    the light gets brighter/dimmer, and stays near zero on the grey mat so
    #    sensor noise isn't amplified into fake edges)
    a = lab[:, :, 1].astype(np.float32) - 128.0
    b = lab[:, :, 2].astype(np.float32) - 128.0
    C = cv2.magnitude(a, b) / (lab[:, :, 0].astype(np.float32) + 20.0)
    C8 = cv2.GaussianBlur(np.clip(C * CHROMA_GAIN, 0, 255).astype(np.uint8), (5, 5), 1.2)

    # gradients: per pixel keep whichever channel has the stronger edge
    gxL = cv2.Sobel(L, cv2.CV_32F, 1, 0, ksize=3)
    gyL = cv2.Sobel(L, cv2.CV_32F, 0, 1, ksize=3)
    gxC = cv2.Sobel(C8, cv2.CV_32F, 1, 0, ksize=3)
    gyC = cv2.Sobel(C8, cv2.CV_32F, 0, 1, ksize=3)
    mL, mC = cv2.magnitude(gxL, gyL), cv2.magnitude(gxC, gyC)
    useC = mC > mL
    gx = np.where(useC, gxC, gxL)
    gy = np.where(useC, gyC, gyL)
    mag = np.maximum(mL, mC)

    # 3. arc candidates from both structure images
    cands = find_circles(L) + find_circles(C8)

    # gray-world white balance (for classification only)
    means = np.array(cv2.mean(small)[:3], np.float32) + 1e-3
    gains = np.clip(means.mean() / means, 0.6, 1.6)
    small_wb = np.clip(small.astype(np.float32) * gains, 0, 255).astype(np.uint8)

    # 4. verification
    dets = []
    for (cx, cy, r) in cands:
        sup, r_ref = edge_support(cx, cy, r, gx, gy, mag, W, H)
        if sup < SUPPORT_MIN:
            continue
        if ring_contrast(cx, cy, r_ref, (L, C8), W, H) < CONTRAST_MIN:
            continue
        at_edge = cx - r_ref < 2 or cy - r_ref < 2 or cx + r_ref > W - 2 or cy + r_ref > H - 2
        if USE_GEOMETRY and not at_edge and not geometry_ok(cy, r_ref, H, f):
            continue
        hs, hmask, hoff = hole_analysis(cx, cy, r_ref, L, W, H)
        score = W_SUPPORT * sup + W_HOLES * hs
        if score < SCORE_MIN:
            continue
        cls, hue, sat = classify(cx, cy, r_ref, small_wb, hmask, hoff, W, H)
        dets.append([score, cx, cy, r_ref, cls])

    # 5. NMS – drop duplicates/nested circles, keep touching neighbours
    kept = suppress(dets)
    kept.sort(key=lambda d: (d[4], -d[0]))       # pollen first, then best score
    kept = kept[:MAX_OUT]

    # 6. outputs
    llpython = [float(len(kept))]
    largestContour = np.array([[]])
    best_area = -1.0
    inv = 1.0 / s
    for score, cx, cy, r, cls in kept:
        tx = math.degrees(math.atan((cx - W / 2.0) / f))
        ty = -math.degrees(math.atan((cy - H / 2.0) / f))
        llpython += [tx, ty, r * inv, float(cls), score]

        C_full = (int(cx * inv), int(cy * inv))
        R_full = max(1, int(r * inv))
        if cls == 0 and r > best_area:            # crosshair contour = largest pollen
            best_area = r
            largestContour = cv2.ellipse2Poly(C_full, (R_full, R_full), 0, 0, 360, 10).reshape(-1, 1, 2)
        if DRAW:
            col = (0, 255, 255) if cls == 0 else (255, 0, 255)
            cv2.circle(image, C_full, R_full, col, 2)
            cv2.putText(image, "%s %.2f" % ("P" if cls == 0 else "O", score),
                        (C_full[0] - R_full, C_full[1] - R_full - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

    while len(llpython) < 32:
        llpython.append(0.0)
    return largestContour, image, llpython