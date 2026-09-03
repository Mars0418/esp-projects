# Three-wheel odometry test

This is an independent, read-only encoder and relative-pose test. It does not
start the camera or line-following controller. All motor control pins, including
TB6612 STBY, are held LOW.

The startup pose is `(x, y, heading) = (0 mm, 0 mm, 0 deg)`. In a top view,
`+x` points right, `+y` points up and matches the initial vehicle heading, and
positive heading is counter-clockwise.

The firmware samples signed cumulative A/B/D encoder counts every 20 ms and
reports the pose every 100 ms:

```text
x_mm=0.0,y_mm=0.0,heading_deg=0.0
```

Do not take the absolute value of encoder counts. The module subtracts the
previous cumulative values and handles signed 32-bit wraparound.

Parameters are isolated in `main/three_wheel_odometry_config.h`. Wheel diameter
is 55 mm, wheel-contact radius is 90 mm and encoder resolution is currently 406
counts/revolution. Encoder signs and ideal 120-degree wheel angles still need
verification from this floor test.

Build and monitor with:

```powershell
cd C:\esp-projects-team\three-wheel-odometry-test
. C:\esp\v6.1-beta1\esp-idf\export.ps1
idf.py -B build-local-6.1-beta1 build
idf.py -B build-local-6.1-beta1 -p COM15 flash monitor
```
