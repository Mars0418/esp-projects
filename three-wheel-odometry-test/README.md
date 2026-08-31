# Three-wheel odometry test

This is an independent, read-only encoder and relative-pose test. It does not
start the camera or line-following controller. All motor control pins, including
TB6612 STBY, are held LOW.

The startup pose is `(x, y, heading) = (0 mm, 0 mm, 0 deg)`. In a top view,
`+x` points right, `+y` points up and matches the initial vehicle heading, and
positive heading is counter-clockwise.

The firmware samples signed cumulative A/B/D encoder counts every 20 ms and
reports every 200 ms:

```text
ODOM x_mm=... y_mm=... heading_mdeg=... countA=... countB=... countD=...
```

Do not take the absolute value of encoder counts. The module subtracts the
previous cumulative values and handles signed 32-bit wraparound.

All unmeasured parameters are isolated in
`main/three_wheel_odometry_config.h`. Current geometry and encoder signs are
placeholders for a symmetric 120-degree omni-wheel chassis and are not valid
distance or angle calibration values.

Build and monitor with:

```powershell
cd C:\esp-projects-team\three-wheel-odometry-test
. C:\esp\v6.1-beta1\esp-idf\export.ps1
idf.py build
idf.py -p COM11 flash monitor
```
