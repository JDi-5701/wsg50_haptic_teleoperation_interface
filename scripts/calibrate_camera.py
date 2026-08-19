#!/usr/bin/env python3
"""Intrinsic calibration from a printed checkerboard. Not a ROS node.

Until this has been run, usb_camera_node guesses fx from vertical_fov_deg and
says so in its log. The guess is good enough to detect a tag and roughly right
for its orientation, but the distance to the tag scales directly with fx, so
any translation you read is only as trustworthy as that guess.

Calibrate at the resolution you will actually capture at. This sensor keeps the
vertical field and crops horizontally for 4:3 modes, so a calibration taken at
1280x720 cannot simply be rescaled to 640x480 - the camera node will refuse to
do it silently and log an error instead.

Usage:
    # print an 9x6 checkerboard, measure one square, then:
    python3 calibrate_camera.py --squares 9x6 --square-size 0.025 \
        --width 640 --height 480 --out ~/camera_640x480.yaml

Hold the board at a range of angles and distances and fill the frame corners -
20 or so good views. Press SPACE to keep a view, q to finish, and the script
reports the reprojection error it achieved (under ~0.5 px is a good result).

Headless (no window): pass --auto to grab a view every --interval seconds.
"""

import argparse
import time

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--squares', default='9x6',
                    help='INNER corner count, e.g. 9x6 for a 10x7 square board')
    ap.add_argument('--square-size', type=float, default=0.025, help='metres')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--views', type=int, default=20)
    ap.add_argument('--auto', action='store_true',
                    help='no window; capture a view every --interval seconds')
    ap.add_argument('--interval', type=float, default=2.0)
    ap.add_argument('--out', default='camera_calibration.yaml')
    args = ap.parse_args()

    nx, ny = (int(v) for v in args.squares.lower().split('x'))
    objp = np.zeros((nx * ny, 3), np.float32)
    objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2) * args.square_size

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open /dev/video{args.device}')

    obj_points, img_points = [], []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    shape = None
    last = 0.0

    print(f'Looking for a {nx}x{ny} inner-corner board, '
          f'{args.square_size*1000:.0f} mm squares.')
    while len(obj_points) < args.views:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(
            gray, (nx, ny),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        take = False
        if args.auto:
            take = found and (time.time() - last) > args.interval
        else:
            vis = frame.copy()
            if found:
                cv2.drawChessboardCorners(vis, (nx, ny), corners, found)
            cv2.putText(vis, f'{len(obj_points)}/{args.views}  '
                             f'{"BOARD" if found else "no board"}  SPACE=keep q=done',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('calibrate', vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            take = found and key == ord(' ')

        if take:
            refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(refined)
            last = time.time()
            print(f'  kept view {len(obj_points)}/{args.views}')

    cap.release()
    if not args.auto:
        cv2.destroyAllWindows()

    if len(obj_points) < 5:
        raise SystemExit(f'Only {len(obj_points)} views - need at least 5.')

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, shape, None, None)

    errors = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        errors.append(cv2.norm(img_points[i], proj, cv2.NORM_L2) /
                      len(proj))
    mean_err = float(np.mean(errors))

    print()
    print(f'  RMS reprojection error : {rms:.4f} px')
    print(f'  mean per-point error   : {mean_err:.4f} px'
          f'   {"(good)" if mean_err < 0.5 else "(high - recapture)"}')
    print(f'  fx={K[0,0]:.2f}  fy={K[1,1]:.2f}  cx={K[0,2]:.2f}  cy={K[1,2]:.2f}')

    out = {
        'image_width': int(shape[0]),
        'image_height': int(shape[1]),
        'camera_name': 'usb_camera',
        'camera_matrix': {'rows': 3, 'cols': 3,
                          'data': [float(v) for v in K.flatten()]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': len(dist.flatten()),
                                    'data': [float(v) for v in dist.flatten()]},
        'rectification_matrix': {'rows': 3, 'cols': 3,
                                 'data': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
        'projection_matrix': {
            'rows': 3, 'cols': 4,
            'data': [float(K[0, 0]), 0., float(K[0, 2]), 0.,
                     0., float(K[1, 1]), float(K[1, 2]), 0.,
                     0., 0., 1., 0.]},
        'rms_reprojection_error_px': float(rms),
    }
    with open(args.out, 'w') as fh:
        yaml.safe_dump(out, fh, default_flow_style=False)
    print(f'\n  wrote {args.out}')
    print(f'  use it:  ros2 run wsg50_haptic_teleoperation_interface '
          f'usb_camera_node.py --ros-args -p camera_info_url:={args.out} '
          f'-p width:={shape[0]} -p height:={shape[1]}')


if __name__ == '__main__':
    main()
