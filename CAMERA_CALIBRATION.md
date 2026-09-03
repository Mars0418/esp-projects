# Camera ground-plane calibration

`camera_extrinsic_calibrator.py` is a PC-side calibration tool. The temporary
calibration-only build in `camera-line-follow-test` keeps the motors stopped,
disables TFT/vision/odometry, and sends the camera's original 640x480 MJPEG
frames at about 1 fps over 921600 baud. The desktop tool's initial `CALIB,1`
command is ignored because this firmware starts directly in calibration mode.
The mounted camera's raw image is upside down, so the desktop tool rotates each
frame by 180 degrees before preview, corner detection and calibration. Exported
parameters therefore match the first-person image used by runtime pose code.

## Parameters

- Enter the number of checkerboard **inner corners on the complete board**, not
  the number of squares and not only the corners currently visible.
- The current 11x8-square board defaults to 10x7 inner corners.
- Every accepted calibration image must contain all configured inner corners.
  The outer board edge may approach or leave the image, but no inner corner can
  be cropped.
- The square edge is 25 mm for the current board.
- `bottom-center distance` defaults to the measured 80 mm. It defines the
  ground point at pixel `(79.5, 119)` as `(x, y) = (0, 80 mm)` relative to the
  vehicle center.

The coordinate system is top-view `+x` right and `+y` forward. The calibration
only maps pixels whose camera rays intersect the flat ground in front of the
camera. Objects above the ground do not have valid distances from this model.

## Run

First flash the current `camera-line-follow-test` firmware. Close `idf.py
monitor` and every other serial program, then run from the repository root:

```powershell
.\start_camera_extrinsic_calibrator.ps1
```

To preselect a port:

```powershell
.\start_camera_extrinsic_calibrator.ps1 -Port COM11
```

The dependency and calibration-math self-test does not require hardware:

```powershell
.\start_camera_extrinsic_calibrator.ps1 -SelfTest
```

## Workflow

1. Enter the checkerboard inner-corner columns and rows. Keep square size at
   `25` mm.
2. Connect. The preview must show all inner corners.
3. Capture 10-20 intrinsic samples while moving and tilting the checkerboard so
   it covers the center, edges and corners of the image. Keep the camera fixed.
4. Select `Calculate intrinsics`. Prefer RMS below 1 pixel; lower is better.
5. Lay the checkerboard flat on the ground, with its near edge parallel to the
   image bottom and covering as much of the driving region as practical.
6. Confirm the measured bottom-center distance, currently `80` mm, then select
   `Calibrate ground extrinsics`. A successful calculation automatically saves
   the result beside the tool.
7. Use `Export` only when an additional copy is needed elsewhere.

Export creates:

- `camera_ground_calibration.json`: high-resolution and scaled 160x120 camera
  matrices, distortion, pose and ground homographies.
- `camera_ground_calibration_pixel_lut.npz`: 160x120 `x_mm`, `y_mm` and `valid`
  arrays for direct per-pixel lookup.

Calibration is tied to the exact 160x120 crop, resize and camera mounting pose.
Changing the camera mount or image pipeline requires recalibration.
