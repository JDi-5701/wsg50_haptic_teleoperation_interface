#!/usr/bin/env python3
"""Record a tag-waving session to video, and score its coverage live.

Calibrating straight off a live camera means that if the collected views turn
out to be too alike, or the settings need changing, someone has to wave the tag
again. Recording first decouples the two: the video can be re-analysed as often
as needed, and this script says while recording whether the coverage is good
enough to bother.

What makes a view useful is TILT. Views square to the lens cannot separate
focal length from distance - a small tag up close and a large one far away
project identically - so the coverage score here is the spread of tag tilts,
not the number of frames.

Usage:
    python3 record_tag_views.py --seconds 90 --out session.avi
    # then
    python3 calibrate_from_tag.py --mode multiview --from-video session.avi \
        --tag-size 0.087 --out cal.yaml
"""

import argparse
import math
import time

import cv2
import numpy as np

FAMILIES = {
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '36h10': cv2.aruco.DICT_APRILTAG_36h10,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--seconds', type=float, default=90.0)
    ap.add_argument('--family', default='36h11', choices=sorted(FAMILIES))
    ap.add_argument('--out', default='tag_session.avi')
    ap.add_argument('--report-every', type=float, default=5.0)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open /dev/video{args.device}')
    for _ in range(10):
        cap.read()
    ok, probe = cap.read()
    h, w = probe.shape[:2]

    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'MJPG'),
                             args.fps, (w, h))
    if not writer.isOpened():
        raise SystemExit(f'Cannot open {args.out} for writing')

    dic = cv2.aruco.Dictionary_get(FAMILIES[args.family])
    par = cv2.aruco.DetectorParameters_create()
    par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

    # Provisional intrinsics: only ever used to judge tilt, never kept.
    fx0 = (h / 2.0) / math.tan(math.radians(43.0) / 2.0)
    K0 = np.array([[fx0, 0, w / 2.0], [0, fx0, h / 2.0], [0, 0, 1]], np.float64)
    obj = np.array([[-.5, .5, 0], [.5, .5, 0], [.5, -.5, 0], [-.5, -.5, 0]],
                   np.float32)          # unit tag: scale is irrelevant for tilt

    print(f'Recording {args.seconds:.0f}s to {args.out} at {w}x{h}.')
    print('Tilt the tag left, right, up and down, and vary the distance.\n')

    t0 = time.time()
    frames = seen = 0
    tilts = []
    next_report = args.report_every

    while time.time() - t0 < args.seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        writer.write(frame)
        frames += 1

        corners, ids, _ = cv2.aruco.detectMarkers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dic, parameters=par)
        if ids is not None:
            seen += 1
            pts = corners[0].reshape(4, 2).astype(np.float32)
            ok2, rvec, _ = cv2.solvePnP(obj, pts, K0, np.zeros(5),
                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok2:
                R, _ = cv2.Rodrigues(rvec)
                n = R[:, 2]
                tilt = math.degrees(math.acos(min(1.0, abs(float(n[2])))))
                axis = math.degrees(math.atan2(float(n[1]), float(n[0])))
                tilts.append((tilt * math.cos(math.radians(axis)),
                              tilt * math.sin(math.radians(axis))))

        el = time.time() - t0
        if el >= next_report:
            next_report += args.report_every
            print(f'  [{el:5.1f}s] {frames} frames, tag in {seen} '
                  f'({100*seen/max(frames,1):.0f}%), {coverage_line(tilts)}',
                  flush=True)

    cap.release()
    writer.release()

    el = time.time() - t0
    print(f'\n  {frames} frames in {el:.1f}s ({frames/el:.1f} fps), '
          f'tag seen in {seen} ({100*seen/max(frames,1):.0f}%)')
    print(f'  {coverage_line(tilts)}')
    spread = tilt_spread(tilts)
    if spread < 40:
        print(f'  COVERAGE TOO LOW. {spread:.0f} deg of tilt spread will not '
              f'separate focal length from distance - record again and tilt '
              f'the tag further.')
    else:
        print(f'  Coverage is enough to calibrate.')
    print(f'\n  wrote {args.out}')


def tilt_spread(tilts):
    if len(tilts) < 2:
        return 0.0
    a = np.array(tilts)
    return float(np.max(np.linalg.norm(a[:, None, :] - a[None, :, :], axis=-1)))


def coverage_line(tilts):
    if not tilts:
        return 'no tilt data yet'
    a = np.array(tilts)
    mags = np.linalg.norm(a, axis=1)
    # Which quadrants of tilt direction have been visited at all.
    quad = set()
    for x, y in a:
        if math.hypot(x, y) > 8:
            quad.add((x > 0, y > 0))
    return (f'tilt max {mags.max():.0f} deg, spread {tilt_spread(tilts):.0f} deg, '
            f'{len(quad)}/4 directions')


if __name__ == '__main__':
    main()
