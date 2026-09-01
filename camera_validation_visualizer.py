"""Build an inline validation-set inspection visualization."""

from __future__ import annotations

import argparse
import base64
from io import BytesIO
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import torch

from camera_model_trainer import (
    CORNER_TYPE_NAMES,
    FINISH_SCALE,
    SCENE_NAMES,
    SEGMENT_RHO_SCALE,
    TinyCnnPerceptionModel,
    decode_corner_xy,
    load_dataset,
)


def data_url(image: Image.Image, image_format: str, **options: object) -> str:
    target = BytesIO()
    image.save(target, format=image_format, **options)
    mime = "image/jpeg" if image_format == "JPEG" else "image/png"
    return f"data:{mime};base64,{base64.b64encode(target.getvalue()).decode()}"


def angular_error(left: np.ndarray, right: np.ndarray) -> float:
    left = left / max(1e-9, float(np.linalg.norm(left)))
    right = right / max(1e-9, float(np.linalg.norm(right)))
    return math.degrees(math.acos(float(np.clip(np.dot(left, right), -1.0, 1.0))))


def build(args: argparse.Namespace) -> None:
    data, splits = load_dataset(args.dataset)
    records = [json.loads(line) for line in (args.dataset / "index.jsonl").read_text().splitlines()]
    validation_indices = [index for index, split in enumerate(splits) if split == "validation"]
    if not validation_indices:
        raise ValueError("dataset has no validation frames")

    model = TinyCnnPerceptionModel()
    model.load_state_dict(torch.load(args.model / "model.pt", weights_only=True))
    model.eval()
    with torch.no_grad():
        output = model(data["features"])
        scene_probabilities = output["scene"].softmax(dim=-1).numpy()
        line_probabilities = output["segment_valid"].sigmoid().numpy()
        corner_probabilities = output["corner_valid"].sigmoid().numpy()
        corner_type_probabilities = output["corner_type"].softmax(dim=-1).numpy()
        predicted_rho = output["segment_rho"].numpy() * SEGMENT_RHO_SCALE
        predicted_direction = torch.nn.functional.normalize(
            output["segment_direction"], dim=-1
        ).numpy()
        predicted_corner = decode_corner_xy(output).numpy()

    with np.load(args.calibration) as calibration:
        ground_x = calibration["x_mm"]
        ground_y = calibration["y_mm"]
        ground_valid = calibration["valid"].astype(bool)
    roi_x_min, roi_y_min, roi_width, roi_height = 27, 4, 106, 108
    roi_ground_x = ground_x[roi_y_min : roi_y_min + roi_height, roi_x_min : roi_x_min + roi_width]
    roi_ground_y = ground_y[roi_y_min : roi_y_min + roi_height, roi_x_min : roi_x_min + roi_width]
    roi_ground_valid = ground_valid[roi_y_min : roi_y_min + roi_height, roi_x_min : roi_x_min + roi_width]
    valid_y, valid_x = np.nonzero(roi_ground_valid)
    valid_ground = np.column_stack((roi_ground_x[roi_ground_valid], roi_ground_y[roi_ground_valid]))

    def ground_to_pixel(point: np.ndarray, maximum_error: float = 40.0) -> list[float] | None:
        distances = np.sum((valid_ground - point) ** 2, axis=1)
        nearest = int(np.argmin(distances))
        if float(distances[nearest]) > maximum_error**2:
            return None
        return [float(valid_x[nearest]), float(valid_y[nearest])]

    model_config = json.loads((args.model / "model_config.json").read_text(encoding="utf-8"))
    t_threshold = float(model_config["finish_t_calibration"]["threshold"])
    validation_metrics = model_config["validation_metrics"]
    frame_data: list[dict[str, object]] = []
    label_cache: dict[Path, dict[str, dict[str, object]]] = {}
    for frame_index in validation_indices:
        record = records[frame_index]
        raw_path = Path(record["raw_image"])
        session = raw_path.parent.parent
        if session not in label_cache:
            labels = [json.loads(line) for line in (session / "path_labels.jsonl").read_text(encoding="utf-8").splitlines()]
            label_cache[session] = {str(label["sample_id"]): label for label in labels}
        label = label_cache[session][str(record["sample_id"])]
        raw_image = Image.open(raw_path).convert("RGB").transpose(Image.Transpose.ROTATE_180)
        mask_image = Image.open(record["mask_image"]).convert("L").transpose(Image.Transpose.ROTATE_180)

        truth_scene = int(data["scene"][frame_index])
        predicted_scene = int(np.argmax(scene_probabilities[frame_index]))
        true_direction = data["segment_direction"][frame_index, 0].numpy()
        model_direction = predicted_direction[frame_index, 0]
        true_rho = float(data["segment_rho"][frame_index, 0])
        model_rho = float(predicted_rho[frame_index, 0])
        line_valid = bool(data["segment_valid"][frame_index, 0])
        heading_error = angular_error(true_direction, model_direction) if line_valid else None
        rho_error = abs(model_rho - true_rho) if line_valid else None

        normal = np.asarray((model_direction[1], -model_direction[0]))
        closest = model_rho * normal
        corner_ground = predicted_corner[frame_index, 0]
        corner_along_line = float(np.dot(corner_ground, model_direction))
        model_line_pixels: list[list[float]] = []
        line_distances = np.unique(
            np.append(np.linspace(-150.0, 850.0, 81), corner_along_line)
        )
        for distance in line_distances:
            pixel = ground_to_pixel(closest + distance * model_direction, 28.0)
            if pixel is not None and (not model_line_pixels or pixel != model_line_pixels[-1]):
                model_line_pixels.append(pixel)

        corner_valid = bool(data["corner_valid"][frame_index, 0])
        model_corner_pixel = ground_to_pixel(corner_ground, 55.0)
        model_corner_type = int(np.argmax(corner_type_probabilities[frame_index, 0]))
        turn_angle_degrees = (-60.0, -90.0, 60.0, 90.0)[model_corner_type]
        turn_angle = math.radians(turn_angle_degrees)
        rotation = np.asarray(
            (
                (math.cos(turn_angle), -math.sin(turn_angle)),
                (math.sin(turn_angle), math.cos(turn_angle)),
            )
        )
        next_direction = rotation @ model_direction
        model_turn_pixels: list[list[float]] = []
        for distance in np.linspace(0.0, 350.0, 36):
            pixel = ground_to_pixel(
                corner_ground + distance * next_direction, 28.0
            )
            if pixel is not None and (
                not model_turn_pixels or pixel != model_turn_pixels[-1]
            ):
                model_turn_pixels.append(pixel)
        human_polyline = label.get("polyline", []) if label.get("valid") else []
        human_corner_indices = label.get("corner_indices", [])
        human_corners = [human_polyline[index] for index in human_corner_indices if index < len(human_polyline)]
        finish = label.get("finish", {})
        human_finish = [finish[key] for key in ("junction", "crossbar_left", "crossbar_right") if key in finish]
        corner_position_error = None
        if corner_valid:
            corner_position_error = float(
                np.linalg.norm(
                    corner_ground - data["corner_xy"][frame_index, 0].numpy()
                )
            )

        frame_data.append(
            {
                "number": len(frame_data) + 1,
                "sampleId": record["sample_id"],
                "raw": data_url(raw_image, "JPEG", quality=58, optimize=True),
                "mask": data_url(mask_image, "PNG", optimize=True),
                "truthScene": SCENE_NAMES[truth_scene],
                "predictedScene": SCENE_NAMES[predicted_scene],
                "sceneProbabilities": [round(float(value), 4) for value in scene_probabilities[frame_index]],
                "humanPolyline": human_polyline,
                "humanCorners": human_corners,
                "humanFinish": human_finish,
                "modelLine": model_line_pixels,
                "modelLineProbability": round(float(line_probabilities[frame_index, 0]), 4),
                "modelCorner": model_corner_pixel,
                "modelTurn": model_turn_pixels,
                "modelCornerProbability": round(float(corner_probabilities[frame_index, 0]), 4),
                "modelCornerType": CORNER_TYPE_NAMES[model_corner_type],
                "modelCornerTypeProbability": round(float(corner_type_probabilities[frame_index, 0, model_corner_type]), 4),
                "tProbability": round(float(scene_probabilities[frame_index, 1]), 4),
                "tAlert": bool(scene_probabilities[frame_index, 1] >= t_threshold),
                "sceneError": truth_scene != predicted_scene,
                "cornerTruth": corner_valid,
                "cornerMissed": corner_valid and corner_probabilities[frame_index, 0] < 0.5,
                "rhoError": None if rho_error is None else round(rho_error, 1),
                "headingError": None if heading_error is None else round(heading_error, 1),
                "cornerPositionError": None if corner_position_error is None else round(corner_position_error, 1),
                "worstLine": bool(line_valid and (rho_error > 60.0 or heading_error > 30.0)),
            }
        )

    payload = json.dumps(
        frame_data,
        ensure_ascii=True,
        separators=(",", ":"),
        default=lambda value: value.item()
        if isinstance(value, np.generic)
        else str(value),
    )
    replacements = {
        "__FRAME_DATA__": payload,
        "__T_THRESHOLD__": f"{t_threshold:.2f}",
        "__SCENE_ACCURACY__": f"{100.0 * float(validation_metrics['scene_accuracy']):.1f}%",
        "__LINE_RHO__": f"{float(validation_metrics['segment_rho_mae_mm']):.1f}",
        "__LINE_HEADING__": f"{float(validation_metrics['segment_heading_mae_deg']):.1f}",
        "__CORNER_PRECISION__": f"{100.0 * float(validation_metrics['corner_detection_precision']):.1f}%",
        "__CORNER_RECALL__": f"{100.0 * float(validation_metrics['corner_detection_recall']):.1f}%",
    }
    html = TEMPLATE
    for source, target in replacements.items():
        html = html.replace(source, target)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html, encoding="utf-8")
    print(f"Wrote {len(frame_data)} validation frames to {args.output}")


TEMPLATE = r'''<div id="camera-validation-viz">
<style>
#camera-validation-viz{color:var(--foreground);font-family:inherit;display:grid;gap:14px}
#camera-validation-viz h2,#camera-validation-viz p{margin:0}
#camera-validation-viz .cv-stats{display:flex;gap:18px;flex-wrap:wrap;color:var(--muted-foreground)}
#camera-validation-viz .cv-stats strong{color:var(--foreground);font-weight:500}
#camera-validation-viz .cv-controls{display:grid;grid-template-columns:minmax(150px,220px) 1fr auto;gap:12px;align-items:end}
#camera-validation-viz .cv-control{display:grid;gap:5px}
#camera-validation-viz .cv-stage{display:grid;grid-template-columns:1fr 1fr;gap:14px}
#camera-validation-viz figure{margin:0;display:grid;gap:6px;min-width:0}
#camera-validation-viz canvas{width:100%;aspect-ratio:106/108;border:1px solid var(--border);display:block;image-rendering:pixelated;background:var(--muted)}
#camera-validation-viz figcaption{color:var(--muted-foreground)}
#camera-validation-viz .cv-detail{display:flex;flex-wrap:wrap;gap:8px 18px;padding-top:2px;border-top:1px solid var(--border)}
#camera-validation-viz .cv-detail span{white-space:nowrap}
#camera-validation-viz .cv-legend{display:flex;gap:14px;flex-wrap:wrap;color:var(--muted-foreground)}
#camera-validation-viz .cv-key{display:inline-flex;align-items:center;gap:6px}
#camera-validation-viz .cv-swatch{width:18px;height:3px;background:var(--viz-series-1)}
#camera-validation-viz .cv-swatch.model{background:var(--viz-series-2)}
#camera-validation-viz .cv-swatch.corner{height:10px;width:10px;border-radius:50%;background:var(--viz-series-3)}
#camera-validation-viz .cv-empty{padding:30px 0;color:var(--muted-foreground)}
@media(max-width:620px){#camera-validation-viz .cv-controls{grid-template-columns:1fr}#camera-validation-viz .cv-stage{grid-template-columns:1fr}}
</style>
<h2>验证集逐帧结果</h2>
<div class="cv-stats" aria-label="验证集整体指标">
  <span>场景准确率 <strong>__SCENE_ACCURACY__</strong></span>
  <span>当前线误差 <strong>__LINE_RHO__ mm / __LINE_HEADING__°</strong></span>
  <span>角点检测 <strong>P __CORNER_PRECISION__ / R __CORNER_RECALL__</strong></span>
  <span>T阈值 <strong>__T_THRESHOLD__</strong></span>
</div>
<div class="cv-controls viz-controls">
  <label class="cv-control form-label">筛选
    <select id="cv-filter" class="form-select">
      <option value="all">全部100帧</option><option value="sceneError">场景分类错误</option>
      <option value="cornerTruth">真实角点帧</option><option value="cornerMissed">漏检角点</option>
      <option value="trueT">真实T终点</option><option value="tAlert">T告警</option>
      <option value="worstLine">直线误差较大</option>
    </select>
  </label>
  <label class="cv-control form-label"><span id="cv-range-label">帧</span>
    <input id="cv-range" class="form-range" type="range" min="0" max="99" value="0">
  </label>
  <span id="cv-count" class="text-muted text-nowrap"></span>
</div>
<div id="cv-content">
  <div class="cv-stage">
    <figure><canvas id="cv-raw" width="424" height="432" role="img" aria-label="原图及人工标注与模型结果"></canvas><figcaption>原图与叠加结果</figcaption></figure>
    <figure><canvas id="cv-mask" width="424" height="432" role="img" aria-label="二值图及人工标注与模型结果"></canvas><figcaption>模型输入二值图</figcaption></figure>
  </div>
  <div class="cv-legend">
    <span class="cv-key"><span class="cv-swatch"></span>人工路径</span>
    <span class="cv-key"><span class="cv-swatch model"></span>模型当前线（虚线）</span>
    <span class="cv-key"><span class="cv-swatch corner"></span>人工角点 / T标记</span>
  </div>
  <div id="cv-detail" class="cv-detail" aria-live="polite"></div>
</div>
<div id="cv-empty" class="cv-empty" hidden>此筛选条件下没有帧。</div>
<script>
(()=>{const root=document.getElementById('camera-validation-viz');const frames=__FRAME_DATA__;const filter=root.querySelector('#cv-filter');const range=root.querySelector('#cv-range');const rangeLabel=root.querySelector('#cv-range-label');const count=root.querySelector('#cv-count');const content=root.querySelector('#cv-content');const empty=root.querySelector('#cv-empty');const detail=root.querySelector('#cv-detail');const raw=root.querySelector('#cv-raw');const mask=root.querySelector('#cv-mask');let visible=frames;
const color=n=>{const probe=document.createElement('span');probe.style.color=`var(${n})`;probe.hidden=true;root.appendChild(probe);const value=getComputedStyle(probe).color;probe.remove();return value};
function draw(canvas,url,frame){const ctx=canvas.getContext('2d');const img=new Image();img.onload=()=>{ctx.clearRect(0,0,canvas.width,canvas.height);ctx.imageSmoothingEnabled=false;ctx.drawImage(img,0,0,canvas.width,canvas.height);const sx=canvas.width/106,sy=canvas.height/108;ctx.lineCap='round';ctx.lineJoin='round';
function path(points,stroke,width,dash=[]){if(!points||points.length<2)return;ctx.beginPath();ctx.setLineDash(dash);points.forEach((p,i)=>{const x=p[0]*sx,y=p[1]*sy;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.strokeStyle=stroke;ctx.lineWidth=width;ctx.stroke();ctx.setLineDash([])}
function dot(point,fill,r=5){if(!point)return;ctx.beginPath();ctx.arc(point[0]*sx,point[1]*sy,r,0,Math.PI*2);ctx.fillStyle=fill;ctx.fill()}
path(frame.humanPolyline,color('--viz-series-1'),3);path(frame.modelLine,color('--viz-series-2'),3,[9,6]);if(frame.modelCornerProbability>=.5)path(frame.modelTurn,color('--viz-series-2'),3,[3,5]);frame.humanCorners.forEach(p=>dot(p,color('--viz-series-3'),6));frame.humanFinish.forEach(p=>dot(p,color('--viz-series-4'),6));if(frame.modelCornerProbability>=.5)dot(frame.modelCorner,color('--viz-series-2'),5)};img.src=url}
function show(){if(!visible.length){content.hidden=true;empty.hidden=false;count.textContent='0 帧';return}content.hidden=false;empty.hidden=true;const index=Math.min(Number(range.value),visible.length-1);const f=visible[index];range.value=index;rangeLabel.textContent=`帧 ${f.number} · ${f.sampleId}`;count.textContent=`${index+1} / ${visible.length}`;draw(raw,f.raw,f);draw(mask,f.mask,f);const fmt=v=>v==null?'—':v;detail.innerHTML=`<span>真实/预测：<strong>${f.truthScene}</strong> / <strong>${f.predictedScene}</strong></span><span>T概率：<strong>${(f.tProbability*100).toFixed(1)}%</strong></span><span>线有效：<strong>${(f.modelLineProbability*100).toFixed(1)}%</strong></span><span>ρ误差：<strong>${fmt(f.rhoError)} mm</strong></span><span>航向误差：<strong>${fmt(f.headingError)}°</strong></span><span>角点概率：<strong>${(f.modelCornerProbability*100).toFixed(1)}%</strong></span><span>角点类型：<strong>${f.modelCornerType}</strong></span>`}
function apply(){const value=filter.value;visible=frames.filter(f=>value==='all'||(value==='trueT'?f.truthScene==='finish_t':Boolean(f[value])));range.min=0;range.max=Math.max(0,visible.length-1);range.value=0;show()}
filter.addEventListener('change',apply);range.addEventListener('input',show);apply();
})();
</script>
</div>'''


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    build(parse_args())
