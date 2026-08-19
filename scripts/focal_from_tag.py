#!/usr/bin/env python3
"""Focal length from tilted views of a square tag. No ruler, no checkerboard.

Solves for ONE unknown, fx, assuming square pixels, no skew, the principal
point at the image centre and no distortion. That is a much easier problem than
the full calibrateCamera fit, which with 4 corners a view could not identify:
across settings its fx wandered between 797 and 1038 px, and split-half
validation disagreed by up to 40%.

The geometry: a homography H maps the tag's plane to the image. Writing
K = [[f,0,cx],[0,f,cy],[0,0,1]] and B = K^-T K^-1, the two columns h1, h2 of H
correspond to two orthonormal rotation columns, which gives Zhang's pair of
constraints

    h1^T B h2 = 0                  (the axes are perpendicular)
    h1^T B h1 = h2^T B h2          (the axes have equal length)

With cx, cy fixed, each constraint alone solves for f. Two constraints per
frame, over every frame, so f is heavily over-determined - and the two
estimates agreeing is itself a check.

Where this fails is stated by the maths rather than hidden: both denominators
carry h31 and h32, the terms that vanish when the tag faces the lens squarely.
A fronto-parallel view says nothing about focal length, so views are weighted by
how tilted they are and near-flat ones are dropped.

TAG SIZE IS IRRELEVANT HERE. It scales the homography, and the scale cancels.

Usage:
    python3 focal_from_tag.py --from-video session.avi --out cal.yaml
    python3 focal_from_tag.py --width 800 --height 600 --seconds 20
"""

import argparse
import math

import cv2
import numpy as np
import yaml

FAMILIES = {
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '36h10': cv2.aruco.DICT_APRILTAG_36h10,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}

# Unit square in the tag plane; any scale gives the same f.
OBJ2D = np.array([[-.5, .5], [.5, .5], [.5, -.5], [-.5, -.5]], np.float32)


def focal_from_homography(H, cx, cy):
    """Return (f_perp, f_norm, cond_perp, cond_norm).

    The conditioning numbers matter as much as the estimates. Each constraint
    divides by a term that vanishes for a fronto-parallel tag, and the two
    vanish under different circumstances: the perpendicularity denominator is
    h31*h32, near zero unless the tag is tilted about BOTH image axes, while
    the equal-norm denominator is h32^2 - h31^2, which survives a tilt about
    one axis. Hand-held tilting is mostly single-axis, which is why the
    equal-norm estimate is the steady one and perpendicularity is the one that
    scatters.
    """
    h11, h12 = H[0, 0], H[0, 1]
    h21, h22 = H[1, 0], H[1, 1]
    h31, h32 = H[2, 0], H[2, 1]

    a1, b1, c1 = h11 - cx * h31, h21 - cy * h31, h31
    a2, b2, c2 = h12 - cx * h32, h22 - cy * h32, h32

    # Normalise the conditioning by the homography's own scale so it compares
    # across frames at different distances.
    scale = max(abs(h11), abs(h21), abs(h12), abs(h22), 1e-12)

    f_perp = None
    denom = c1 * c2
    cond_perp = abs(denom) / (scale * scale)
    if abs(denom) > 1e-12:
        v = -(a1 * a2 + b1 * b2) / denom
        if v > 0:
            f_perp = math.sqrt(v)

    f_norm = None
    denom2 = c2 * c2 - c1 * c1
    cond_norm = abs(denom2) / (scale * scale)
    if abs(denom2) > 1e-12:
        v = (a1 * a1 + b1 * b1 - a2 * a2 - b2 * b2) / denom2
        if v > 0:
            f_norm = math.sqrt(v)

    return f_perp, f_norm, cond_perp, cond_norm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--from-video', default='')
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--width', type=int, default=800)
    ap.add_argument('--height', type=int, default=600)
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--family', default='36h11', choices=sorted(FAMILIES))
    ap.add_argument('--min-side-px', type=float, default=60.0)
    ap.add_argument('--min-tilt-deg', type=float, default=12.0,
                    help='views flatter than this carry no focal information')
    ap.add_argument('--gamma', type=float, default=0.45,
                    help='<1 lifts shadows. The tag is often backlit against a '
                         'window, where raw frames detect 14%% of the time and '
                         'gamma 0.45 detects 100%%. 1.0 disables.')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    lut = (np.array([((i / 255.0) ** args.gamma) * 255 for i in range(256)],
                    np.uint8) if args.gamma != 1.0 else None)

    dic = cv2.aruco.Dictionary_get(FAMILIES[args.family])
    par = cv2.aruco.DetectorParameters_create()
    par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

    if args.from_video:
        cap = cv2.VideoCapture(args.from_video)
        if not cap.isOpened():
            raise SystemExit(f'Cannot open {args.from_video}')
        live = False
    else:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        live = True
        for _ in range(10):
            cap.read()

    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H_ = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cx, cy = W / 2.0, H_ / 2.0

    import time
    t0 = time.time()
    perp, norm, tilts, frames, seen = [], [], [], 0, 0

    while True:
        if live and time.time() - t0 > args.seconds:
            break
        ok, frame = cap.read()
        if not ok:
            if live:
                continue
            break
        frames += 1
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if lut is not None:
            g = cv2.LUT(g, lut)
        corners, ids, _ = cv2.aruco.detectMarkers(g, dic, parameters=par)
        if ids is None:
            continue
        pts = corners[0].reshape(4, 2).astype(np.float32)
        side = np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)])
        if side < args.min_side_px:
            continue
        seen += 1

        Hm, _ = cv2.findHomography(OBJ2D, pts)
        if Hm is None:
            continue
        fp, fn, cp, cn = focal_from_homography(Hm, cx, cy)

        # Tilt, for reporting and for the flatness cut.
        fx0 = (H_ / 2) / math.tan(math.radians(43) / 2)
        K0 = np.array([[fx0, 0, cx], [0, fx0, cy], [0, 0, 1]], np.float64)
        obj3 = np.hstack([OBJ2D, np.zeros((4, 1), np.float32)])
        ok2, rvec, _ = cv2.solvePnP(obj3, pts, K0, np.zeros(5),
                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok2:
            continue
        R, _ = cv2.Rodrigues(rvec)
        tilt = math.degrees(math.acos(min(1.0, abs(float(R[2, 2])))))
        if tilt < args.min_tilt_deg:
            continue

        tilts.append(tilt)
        if fp and 100 < fp < 5000:
            perp.append((fp, cp))
        if fn and 100 < fn < 5000:
            norm.append((fn, cn))

    cap.release()

    print(f"  {frames} frames, tag usable in {seen}, "
          f"{len(tilts)} tilted past {args.min_tilt_deg:.0f} deg")
    if tilts:
        print(f"  tilt range {min(tilts):.0f}-{max(tilts):.0f} deg")
    if len(perp) < 10 and len(norm) < 10:
        raise SystemExit('  Too few tilted views to solve for focal length.')

    def report(name, vals):
        """Median over the best-conditioned half; a badly conditioned frame
        is not a noisy measurement of f, it is barely a measurement at all."""
        if len(vals) < 10:
            print(f"  {name:16s} too few estimates ({len(vals)})")
            return None, None
        arr = np.array(vals)
        cut = np.median(arr[:, 1])
        good = arr[arr[:, 1] >= cut][:, 0]
        med = float(np.median(good))
        q1, q3 = np.percentile(good, [25, 75])
        iqr_pct = 100 * (q3 - q1) / med
        print(f"  {name:16s} median {med:7.1f} px   IQR {q1:.0f}-{q3:.0f} "
              f"({iqr_pct:.0f}%)   n={len(good)}/{len(arr)} best-conditioned")
        return med, iqr_pct

    print()
    f_perp, s_perp = report('perpendicular', perp)
    f_norm, s_norm = report('equal-norm', norm)

    # Take whichever constraint the data actually supports: the spread of its
    # own best-conditioned estimates is the evidence, not a preference.
    options = [(s, f, n) for f, s, n in
               ((f_perp, s_perp, 'perpendicular'), (f_norm, s_norm, 'equal-norm'))
               if f is not None]
    if not options:
        raise SystemExit('  No usable focal estimate.')
    options.sort()
    spread_best, f, which = options[0]
    print()
    if len(options) == 2:
        print(f"  the two constraints differ by "
              f"{100*abs(f_perp-f_norm)/((f_perp+f_norm)/2):.1f}%")
    print(f"  taking the {which} constraint: its own spread is "
          f"{spread_best:.0f}%, the tighter of the two")
    print(f"  fx = fy = {f:.1f} px  (+/- roughly {spread_best/2:.0f}%)")
    print(f"  implied vertical FOV = {math.degrees(2*math.atan((H_/2)/f)):.1f} deg")
    print(f"  a tag distance computed with this scales directly with fx.")

    if args.out:
        out = {
            'image_width': W, 'image_height': H_, 'camera_name': 'usb_camera',
            'camera_matrix': {'rows': 3, 'cols': 3,
                              'data': [f, 0., cx, 0., f, cy, 0., 0., 1.]},
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {'rows': 1, 'cols': 5,
                                        'data': [0.] * 5},
            'rectification_matrix': {'rows': 3, 'cols': 3,
                                     'data': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
            'projection_matrix': {'rows': 3, 'cols': 4,
                                  'data': [f, 0., cx, 0., 0., f, cy, 0.,
                                           0., 0., 1., 0.]},
            'calibration_method': 'focal_from_tag homography, principal point '
                                  'assumed at image centre, distortion zero',
        }
        with open(args.out, 'w') as fh:
            yaml.safe_dump(out, fh, default_flow_style=False)
        print(f"\n  wrote {args.out}")


if __name__ == '__main__':
    main()
