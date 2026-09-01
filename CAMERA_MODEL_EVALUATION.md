# Camera model validation report

## Data split

- Development: 328 frames from the original and T-supplement sessions.
- Validation: 100 frames from `session_20260901_143055`.
- Test: sealed and not read by model training or validation.

Whole sessions are assigned to one split. Adjacent frames are never randomly
distributed across development and validation.

## Target representation

Annotated ground paths are split at labeled corners. Each part is fitted with
robust total least squares and represented by:

- signed line offset `rho_mm`;
- near-to-far unit direction `direction_xy`;
- fit validity and residual.

The runtime model consumes only the current (nearest) fitted line and nearest
corner. The controller must re-evaluate after every maneuver. The legacy 24
resampled path points remain in the export for visualization only.

The course has five corners in a fixed order. With positive angles meaning a
right turn and negative angles meaning a left turn, the commanded heading
changes are `+120`, `-30`, `-90`, `-90`, and `+90` degrees. The controller,
not the image model, owns this route state.

## Superseded multi-task candidate

Artifact: `camera-models/model_line_constrained_20260901_1700`

- Architecture: two-layer tiny CNN on a 27x27 pooled binary mask.
- Parameters: 16,029.
- The corner head predicts one signed coordinate along the current line. Its
  2D position is reconstructed from the predicted line, so the corner cannot
  be geometrically detached from that line.
- Validation scene accuracy: 78.0%.
- Current-line offset MAE: 41.6 mm.
- Current-line heading MAE: 22.6 degrees.
- Corner detection precision: 78.1%.
- Corner detection recall: 62.5%.
- Corner type accuracy: 62.5%.
- Corner position MAE: 95.1 mm.
- T threshold: 0.85.
- T precision/recall at that threshold: 75% / 75% (only four validation T
  frames, so this estimate has high uncertainty).

These values are not sufficient for direct motor control from learned corner
geometry.

## Superseded position-and-angle ablation

Artifact: `camera-models/model_corner_only_20260901_1730`

This experiment used an incorrect four-class turn assumption. Its position
results remain useful as an ablation, but all turn-class metrics are discarded.

Across three fixed random seeds:

- best epoch: 96 to 102;
- validation corner position MAE: 58.2, 59.2, and 69.3 mm;
Removing unrelated tasks substantially improves corner position compared with
the multi-task model's 95.1 mm.

## Selected corner-position candidate

Artifact: `camera-models/model_corner_position_20260901_1800`

This 15,654-parameter model is trained only on the 81 development frames with
a labeled corner. It outputs ground-plane corner coordinates `(x, y)` and has
no scene, line, T-finish, corner-presence, or turn-angle head. `line_vision`
must establish that a corner exists before this model output is used.

Across three fixed random seeds:

- best epoch: 93 to 346;
- validation corner position MAE: 56.9, 50.3, and 59.1 mm;
- validation x-coordinate MAE: 22.6, 20.6, and 23.2 mm;
- validation y-coordinate MAE: 47.6, 41.8, and 50.6 mm.

The default fixed-seed artifact is retained instead of selecting the best seed
on validation performance. The sealed test set remains unread.

## Hand-written vision baseline

On the same validation labels, the existing `line_vision` result achieved:

- corner precision: 61.8%;
- corner recall: 85.0%;
- left/right direction accuracy on matched corners: 91.2%.

Its inferred direction can be retained as a consistency check, but the fixed
course state supplies the actual turn command.

## Deployment decision

Use a hybrid controller for the first vehicle test:

1. Keep `line_vision` for black-line geometry and corner presence.
2. Use the corner-position model only after `line_vision` opens the corner gate.
3. Command the fixed course turns `+120`, `-30`, `-90`, `-90`, and `+90`
   degrees. Advance the route index only after the turn completes and the
   outgoing line is reacquired.
4. Use the tiny model only as an additional T-finish probability signal.
5. Require at least two T-positive frames in the latest three frames before
   stopping. This assumes the T remains visible for at least three frames.
6. Keep the existing safety stop and temporal smoothing.

Do not deploy learned corner position as the sole corner-presence signal with
the current dataset.
