# Quarter-circle goal vision

The basic and advanced goal methods remain separate. The current build is the
advanced-only test: the basic detector silently supplies the black candidate
region, then `quarter_goal_pose.c` estimates the calibrated field pose on each
decoded 160x120 RGB565 frame. Red/white ball detection and line following are
not called, and the motors remain in safe stop.

## Basic detector

`main/black_marker_vision.c` selects a low-luminance, near-neutral black
connected component. Compactness, quarter-disk fill ratio and local contrast
reject thin court lines and soft shadows. It reports the centroid of the
selected black pixels, not the centre of the bounding box:

```text
GOAL_BASIC found=1 confidence=87 center_raw=(82,43) black_pixels=146
```

The magenta rectangle is the component box and the white cross is its centroid.
The coordinates are raw decoder coordinates. Black thresholds default to
maximum luminance 85, maximum channel 110 and maximum RGB spread 24. They are
stored in `s_thresholds` near the top of `black_marker_vision.c`.

Only a component whose black-pixel centroid lies in the centred 100x90 trust
region is reported. At 160x120 this is raw `x=30..129, y=15..104`; candidates
outside that region are returned as `found=0`.

The basic result uses the same short temporal policy as ball tracking: 1-2
pixel changes stay in a deadband, normal movement is 30% old plus 70% new,
high-confidence large movement snaps immediately, a low-confidence large jump
needs two nearby detections, and one missed frame reuses the last result. A
reused result has `found=1 detected=0 predicted=1`; it is never counted as a
real detection.

## Calibrated pose detector

`main/quarter_goal_pose.c` rectifies the selected pixels onto the ground plane,
estimates the quarter disk's symmetry bisector, and fits the two perpendicular
radius edges. It uses the 160x120 intrinsics, distortion and homography exported
on `test_cmh`:

```text
GOAL_POSE found=1 confidence=79 origin_raw=(97,31) origin_vehicle_mm=(-420,760) robot_field_mm=(350,790) heading_deg=-12 radius_mm=104
```

- `origin_raw` is the coloured pixel nearest the fitted corner.
- `origin_vehicle_mm` uses vehicle coordinates: +x right, +y forward.
- `robot_field_mm=(X,Y)` uses the requested field frame: origin at the
  top-left goal corner, +X down the reference image, +Y to the right.
- `heading_deg` is measured from field +X toward field +Y.
- The red overlay ray is field +X; the blue ray is field +Y.

At the post-line T region the vehicle initially points approximately toward
field +Y. Reset the board at that pose: the odometry origin is the boot pose.
The 20 ms odometry task rotates each current visual goal position back into
that remembered frame. After applying the mounted sensor's 180-degree
raw-to-first-person rotation, a reliable remembered lateral position at least
80 mm on the negative side votes for the lower goal, while one at least 80 mm
on the positive side votes for the upper goal. Five consecutive reliable
frames lock the identity for the run.
The upper goal vertex is fixed at global `(0, 0)` mm and opens toward `+X/+Y`.
The lower goal vertex is fixed at global `(900, 0)` mm and opens toward
`-X/+Y`. Once the identity is known, both fitted straight edges define the
global axes: every reliable visual frame recalculates the vehicle's global
`X/Y/heading`, correcting accumulated odometry drift. Between reliable visual
frames, encoder odometry propagates forward from the latest correction. The
TFT origin cross is green for the upper goal, magenta for the lower goal, and
white while identity is unknown.

`POST_ODOM` reports relative `x_mm`, `y_mm`, `heading_deg` and all three raw
encoder counts every 200 ms. `GOAL_POSE` reports the same pose in `odom_pose`
and the compensated lateral position in `remembered_x_mm`. Do not lift or
manually rotate the chassis during an identity run: any motion not observed by
the encoders breaks the memory. Resetting the board clears both odometry and
the locked goal identity.

After one reliable identified visual observation, the detector also remembers
that goal and its field-coordinate transform in the boot reference frame. A
later missing frame keeps `found=0` but reports `position_valid=1`,
`field_pose_valid=1`, and `predicted=1`; `goal_vehicle_mm`, `robot_field_mm`
and `heading_deg` are then propagated from the wheel odometry. Before the first
reliable anchor, missing frames report `position_valid=0` because wheel motion
alone cannot reveal an unknown goal's location. A visual shape with unknown
upper/lower identity can still have a valid vehicle-relative position, but its
global field pose remains invalid.

Wheel diameter 55 mm and wheel-contact radius 90 mm are measured values.
Counts/revolution 406, all three encoder signs, ideal 120-degree wheel position
and drive angles, effective rolling radius under load, and floor slip remain
provisional in `main/post_line_odometry_config.h`; verify them from floor-test
logs before treating the reported pose as an accurate global measurement.

The goal radius is `GOAL_RADIUS_MM` near the top of `quarter_goal_pose.c` and is
currently 100 mm. The calibration uses the measured 80 mm distance from the
vehicle centre to the bottom-centre ground point in the first-person image and
is valid only for the camera mounting pose used by `test_cmh`. Recalibration
requires replacing the constants in that file.

The advanced frame assumes the uniquely coloured quarter circle is the
top-left field goal and that its coloured sector extends into +X/+Y. If an
identical marker is placed at a different field corner, shape geometry alone
cannot identify which global corner it is; use a distinct colour/notch or an
external pose prior.
