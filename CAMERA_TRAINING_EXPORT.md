# Camera training export

`camera_training_exporter.py` converts fully annotated camera sessions into a
single compact dataset without changing source images or `path_labels.jsonl`.
All exported image and target coordinates use the vehicle first-person view.

## Default export

Run from the repository root:

```powershell
.\start_camera_training_exporter.ps1
```

The exporter discovers every fully annotated `camera-datasets/session_*`
directory and creates a new timestamped directory under
`camera-training-export`. It refuses to overwrite an existing output path.

Run dependency and algorithm checks without exporting:

```powershell
.\start_camera_training_exporter.ps1 -SelfTest
```

## Single-frame input

`frames.npz` contains NCHW arrays:

- `mask`: `uint8 [N,1,108,106]`, values 0 or 1. This is the default INT8 model
  input.
- `luma`: `uint8 [N,1,108,106]`, values 0 through 255. This is retained for
  shadow-rejection experiments, not required by the default model.

Use `-WithoutLuma` to export only the binary mask. RGB is deliberately omitted
because it triples input memory and first-layer computation compared with one
channel.

## Targets

`targets.npz` contains:

- `scene_class`: `path=0`, `finish_t=1`, `invalid=2`.
- `path_xy_mm [N,24,2]`: path points in vehicle ground-plane millimeters.
- `path_s_mm [N,24]`: arc distance from the nearest annotated point.
- `path_valid [N,24]`: valid-point mask.
- `path_length_mm [N]`: full annotated path length, including any portion past
  the 24-point capacity.
- `corner_* [N,3,...]`: up to three ordered corners with position, path
  distance, signed angle, direction and validity.
- `finish_* [N,...]`: T-junction position, crossbar length, stem/crossbar angle
  and validity.

Paths are sampled every 25 mm. Twenty-four points cover 575 mm while requiring
only 96 bytes when deployment coordinates are quantized to signed 16-bit
millimeters.

The 24 points are retained for visualization and backward compatibility. The
runtime model target is the fitted line representation:

- `segment_rho_mm [N,4]`: signed line offset in the vehicle ground plane.
- `segment_direction_xy [N,4,2]`: near-to-far unit direction.
- `segment_fit_rmse_mm [N,4]`: fit residual for data auditing.
- `segment_valid [N,4]`: valid fitted-segment mask.

Segments are split at manually labeled corners and fitted with robust total
least squares, so different point spacing on the same straight line does not
create different model targets. The controller consumes only segment zero and
the nearest corner, then re-evaluates after each maneuver.

## Temporal data

`index.jsonl` preserves session, sequence, timestamp, split and temporal-run
membership for every frame. `clips.jsonl` contains sliding five-frame index
windows; images are referenced rather than duplicated. A new run starts when:

- the session changes;
- the device sequence is not consecutive; or
- the time gap exceeds 1000 ms.

The clips are intended for temporal evaluation and output-level filtering. The
ESP32 runtime should still perform single-frame perception, then smooth the
small path/corner/finish result vectors. Do not feed five complete images to an
LSTM or multi-frame CNN by default.

## Session splits

Assign whole sessions, never individual adjacent frames, to validation and
test sets:

```powershell
.\start_camera_training_exporter.ps1 `
  -ValidationSession session_20260902_090000 `
  -TestSession session_20260903_090000
```

Every unlisted session is assigned to `development`. `audit.json` reports
coverage, geometry distributions, sequence counts, split counts and data-risk
warnings. At least three independent sessions are required for meaningful
development, validation and final test results.

## Compact model training

Train the pooled-mask multi-task model after export:

```powershell
.\start_camera_model_trainer.ps1 `
  -Dataset camera-training-export\export_YYYYMMDD_HHMMSS
```

The trainer uses only records whose split is `development` for gradient
updates. If a validation split is present, it selects the checkpoint by
validation loss. Test records are deliberately ignored. The model inverts the
exported mask so black-line pixels are one, pads the 106-pixel width to 108,
and applies 4x4 average pooling to produce a 27x27 input. A small shared model
then predicts scene, current line, nearest corner and T-finish outputs.
`model_weights.npz` is
the deployment-oriented weight file; INT8 conversion is performed only after
validation selects a checkpoint.
