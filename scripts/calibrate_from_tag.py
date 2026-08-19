#!/usr/bin/env python3
"""Intrinsic calibration from a single AprilTag, for when you cannot print a
checkerboard. Not a ROS node.

Two modes, and they answer different questions.

  --mode distance   ONE measurement, focal length only.
      Hold the tag flat and square to the lens at a distance you measure with a
      tape, and fx follows from fx = side_px * D / tag_size. This calibrates
      the one parameter the tag's distance actually scales with, and its
      accuracy is just how well you measured D: 250 mm read to +/-2 mm gives fx
      to better than 1%. It learns nothing about the principal point or
      distortion, which is fine - both matter far less for a tag pose than fx.

  --mode multiview  MANY views, full intrinsics.
      A single tag gives only 4 points per view, so this needs a lot of views
      and, above all, TILTED ones. It is insensitive to tag_size: scaling the
      object points scales the recovered translations and leaves the focal
      length and principal point untouched. Views square to the lens carry no
      information separating focal length from distance - a small tag up close
      and a large one far away look identical - so the script refuses views
      that repeat a tilt it already has, and refuses to solve at all if the
      collected tilts are too alike. Distortion stays fixed at zero unless you
      ask for it; 4 points per view cannot constrain it honestly.

A checkerboard remains better than either. scripts/calibrate_camera.py does
that, and its result supersedes anything from here.

Usage:
    # measure the distance from the LENS to the tag face first, in metres
    python3 calibrate_from_tag.py --mode distance --tag-size 0.038 --distance 0.25

    # collect views live: tilt the tag left, right, up, down, near, far
    python3 calibrate_from_tag.py --mode multiview --tag-size 0.087 --out cal.yaml

    # or off a recording from record_tag_views.py, which can be re-run freely
    python3 calibrate_from_tag.py --mode multiview --from-video session.avi \
        --tag-size 0.087 --out cal.yaml
"""

import argparse
import math
import time

import cv2
import numpy as np
import yaml

FAMILIES = {
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '36h10': cv2.aruco.DICT_APRILTAG_36h10,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


class VideoSource:
    """A live camera or a recorded file, behind one read()."""

    def __init__(self, path=None, device=0, width=640, height=480):
        if path:
            self.cap = cv2.VideoCapture(path)
            if not self.cap.isOpened():
                raise SystemExit(f'Cannot open {path}')
            self.live = False
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        else:
            self.cap = open_camera(device, width, height)
            self.live = True
            self.width, self.height = width, height

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


def open_camera(device, width, height):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open /dev/video{device}')
    for _ in range(10):
        cap.read()
    return cap


def make_detector(family):
    dic = cv2.aruco.Dictionary_get(FAMILIES[family])
    par = cv2.aruco.DetectorParameters_create()
    par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return dic, par


def detect(gray, dic, par, tag_id):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dic, parameters=par)
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten()):
        if tag_id is None or int(i) == tag_id:
            return c.reshape(4, 2).astype(np.float32)
    return None


def mean_side(pts):
    return float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4])
                          for k in range(4)]))


def tag_object_points(size):
    h = size / 2.0
    return np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]],
                    dtype=np.float32)


# ----------------------------------------------------------------------
def mode_distance(args):
    dic, par = make_detector(args.family)
    cap = open_camera(args.device, args.width, args.height)

    sides, samples = [], 0
    t0 = time.time()
    while samples < args.samples and time.time() - t0 < 30:
        ok, frame = cap.read()
        if not ok:
            continue
        pts = detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dic, par, args.tag_id)
        if pts is None:
            continue
        sides.append(mean_side(pts))
        samples += 1
    cap.release()

    if samples < 5:
        raise SystemExit(f'Only saw the tag {samples} times - check it is in view.')

    side = float(np.mean(sides))
    sd = float(np.std(sides))
    fx = side * args.distance / args.tag_size

    print(f'  tag side      : {side:.2f} px  (sd {sd:.2f} over {samples} frames)')
    print(f'  distance      : {args.distance*1000:.1f} mm (yours, by tape)')
    print(f'  tag size      : {args.tag_size*1000:.1f} mm')
    print()
    print(f'  fx = fy       = {fx:.1f} px')
    print(f'  implied vertical FOV = '
          f'{math.degrees(2*math.atan((args.height/2)/fx)):.1f} deg')
    print()
    # The pixel noise is tiny next to the tape measure, so quote the latter.
    err_mm = args.distance_tolerance
    print(f'  a +/-{err_mm*1000:.0f} mm error in your distance gives '
          f'fx +/-{fx*err_mm/args.distance:.1f} px '
          f'({100*err_mm/args.distance:.1f}%), and tag distances scale the same way')

    if args.out:
        write_yaml(args.out, args.width, args.height, fx, fx,
                   args.width / 2.0, args.height / 2.0, [0.] * 5, None)
        print(f'\n  wrote {args.out}')
        print('  NOTE: principal point assumed at image centre, distortion zero.')


# ----------------------------------------------------------------------
def mode_multiview(args):
    dic, par = make_detector(args.family)
    src = VideoSource(args.from_video or None, args.device, args.width, args.height)
    args.width, args.height = src.width, src.height
    obj = tag_object_points(args.tag_size)
    # Only the recovered translations scale with tag_size; calibrateCamera's
    # focal length and principal point do not. So a wrong tag_size here costs
    # nothing that matters - unlike in distance mode.

    # Provisional intrinsics, only ever used to judge how tilted a view is.
    fx0 = (args.height / 2.0) / math.tan(math.radians(43.0) / 2.0)
    K0 = np.array([[fx0, 0, args.width / 2.0],
                   [0, fx0, args.height / 2.0],
                   [0, 0, 1]], dtype=np.float64)

    kept_img, kept_tilts = [], []
    print(f'Collecting {args.views} views. Tilt the tag: left, right, up, down, '
          f'and vary the distance.')
    print(f'Views closer than {args.min_tilt_sep:.0f} deg to one already kept '
          f'are skipped.\n')

    t0 = time.time()
    while len(kept_img) < args.views:
        if src.live and time.time() - t0 > args.timeout:
            break
        ok, frame = src.read()
        if not ok:
            if src.live:
                continue
            break                       # end of file
        pts = detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dic, par, args.tag_id)
        if pts is None:
            continue

        ok, rvec, tvec = cv2.solvePnP(obj, pts, K0, np.zeros(5),
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        # Tag normal in camera coordinates; its angle off the optical axis is
        # the tilt that makes a view informative.
        normal = R[:, 2]
        tilt = math.degrees(math.acos(min(1.0, abs(float(normal[2])))))
        axis = math.degrees(math.atan2(float(normal[1]), float(normal[0])))
        v = np.array([tilt * math.cos(math.radians(axis)),
                      tilt * math.sin(math.radians(axis))])

        if any(np.linalg.norm(v - k) < args.min_tilt_sep for k in kept_tilts):
            continue
        if mean_side(pts) < args.min_side_px:
            continue                    # too small for trustworthy corners

        kept_img.append(pts)
        kept_tilts.append(v)
        print(f'  view {len(kept_img):2d}/{args.views}  tilt {tilt:5.1f} deg '
              f'about {axis:+7.1f} deg')

    src.release()

    if len(kept_img) < args.min_views:
        where = args.from_video or f'{args.timeout:.0f}s of live capture'
        raise SystemExit(
            f'\nOnly {len(kept_img)} distinct views in {where} - need '
            f'{args.min_views}. The tag has to MOVE: tilt it to different '
            f'angles rather than holding it still.')

    spread = float(np.max([np.linalg.norm(a - b)
                           for a in kept_tilts for b in kept_tilts]))
    print(f'\n  {len(kept_img)} views, tilt spread {spread:.1f} deg')
    if spread < args.min_spread:
        raise SystemExit(
            f'  Tilt spread {spread:.1f} deg is too small (need '
            f'{args.min_spread:.0f}). Views this alike cannot separate focal '
            f'length from distance, so any answer would be meaningless.')

    flags = cv2.CALIB_FIX_ASPECT_RATIO
    if not args.estimate_distortion:
        flags |= (cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 |
                  cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    if args.fix_principal_point:
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT

    K_init = K0.copy()
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        [obj] * len(kept_img), kept_img, (args.width, args.height),
        K_init, np.zeros(5), flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)

    print(f'  RMS reprojection error : {rms:.4f} px')
    print(f'  fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  cx={K[0,2]:.1f}  cy={K[1,2]:.1f}')
    print(f'  implied vertical FOV   : '
          f'{math.degrees(2*math.atan((args.height/2)/K[1,1])):.1f} deg')
    if rms > 1.0:
        print('  WARNING: error above 1 px - collect more, better-spread views.')

    if args.out:
        write_yaml(args.out, args.width, args.height, K[0, 0], K[1, 1],
                   K[0, 2], K[1, 2], list(dist.flatten()), float(rms))
        print(f'\n  wrote {args.out}')


# ----------------------------------------------------------------------
def write_yaml(path, w, h, fx, fy, cx, cy, dist, rms):
    out = {
        'image_width': int(w),
        'image_height': int(h),
        'camera_name': 'usb_camera',
        'camera_matrix': {'rows': 3, 'cols': 3,
                          'data': [float(fx), 0., float(cx),
                                   0., float(fy), float(cy),
                                   0., 0., 1.]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': len(dist),
                                    'data': [float(v) for v in dist]},
        'rectification_matrix': {'rows': 3, 'cols': 3,
                                 'data': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
        'projection_matrix': {'rows': 3, 'cols': 4,
                              'data': [float(fx), 0., float(cx), 0.,
                                       0., float(fy), float(cy), 0.,
                                       0., 0., 1., 0.]},
    }
    if rms is not None:
        out['rms_reprojection_error_px'] = rms
    with open(path, 'w') as fh:
        yaml.safe_dump(out, fh, default_flow_style=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['distance', 'multiview'], required=True)
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--family', default='36h11', choices=sorted(FAMILIES))
    ap.add_argument('--tag-size', type=float, required=True,
                    help='metres, outer edge of the black square')
    ap.add_argument('--tag-id', type=int, default=None)
    ap.add_argument('--out', default='')

    ap.add_argument('--distance', type=float,
                    help='distance mode: lens to tag face, in metres')
    ap.add_argument('--distance-tolerance', type=float, default=0.002,
                    help='distance mode: how well you trust your tape, metres')
    ap.add_argument('--samples', type=int, default=60)

    ap.add_argument('--views', type=int, default=25)
    ap.add_argument('--min-views', type=int, default=12)
    ap.add_argument('--min-tilt-sep', type=float, default=6.0,
                    help='multiview: degrees a new view must differ by')
    ap.add_argument('--min-spread', type=float, default=40.0,
                    help='multiview: required spread of tilts, degrees')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--from-video', default='',
                    help='calibrate from a recording instead of the live camera')
    ap.add_argument('--min-side-px', type=float, default=40.0,
                    help='skip views where the tag is smaller than this')
    ap.add_argument('--estimate-distortion', action='store_true')
    ap.add_argument('--fix-principal-point', action='store_true')
    args = ap.parse_args()

    if args.mode == 'distance':
        if args.distance is None:
            raise SystemExit('--mode distance needs --distance <metres>')
        mode_distance(args)
    else:
        mode_multiview(args)


if __name__ == '__main__':
    main()
