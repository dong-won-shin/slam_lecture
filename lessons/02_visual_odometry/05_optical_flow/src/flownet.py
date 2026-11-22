"""Run pretrained FlowNetS on an image pair or consecutive video frames."""
import argparse
import math
import os
import sys
import time
from pathlib import Path

try:
    import cv2
    import numpy as np
    import torch
except ImportError as exc:
    sys.exit(f"{exc}. Install requirements-flownet.txt with the selected Python interpreter.")

from flownets_model import estimate_flow, load_model

LESSON = Path(__file__).resolve().parents[1]
TITLE = "FlowNetS optical flow"


def overview(previous, field, label):
    dx, dy = cv2.split(field)
    magnitude, angle = cv2.cartToPolar(dx, dy, angleInDegrees=True)
    hsv = np.empty_like(previous)
    hsv[:, :, 0] = (angle * 0.5).astype(np.uint8) % 180
    hsv[:, :, 1] = 255
    hsv[:, :, 2] = np.clip(magnitude * (255 / 32), 0, 255).astype(np.uint8)
    colors = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    arrows = previous.copy()
    for y in range(12, arrows.shape[0], 24):
        for x in range(12, arrows.shape[1], 24):
            end = (round(x + float(dx[y, x])), round(y + float(dy[y, x])))
            cv2.arrowedLine(arrows, (x, y), end, (40, 220, 255), 1, cv2.LINE_AA, tipLength=0.25)

    canvas = np.full((458, 1040, 3), (29, 26, 24), dtype=np.uint8)

    def text(value, x, y):
        cv2.putText(canvas, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)

    for offset, image, title in ((12, arrows, "Previous / sampled flow vectors"),
                                 (526, colors, "FlowNetS / dense flow")):
        text(title, offset, 26)
        h, w = image.shape[:2]
        scale = min(502 / w, 378 / h)
        size = (max(1, round(w * scale)), max(1, round(h * scale)))
        x, y = offset + (502 - size[0]) // 2, 35 + (378 - size[1]) // 2
        canvas[y:y + size[1], x:x + size[0]] = cv2.resize(image, size)
    text(label, 850, 26)
    for i, name in enumerate(("Right", "Down", "Left", "Up")):
        color = cv2.cvtColor(np.uint8([[[i * 45, 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
        x = 12 + 90 * i
        cv2.rectangle(canvas, (x, 431), (x + 12, 443), tuple(map(int, color)), -1)
        text(name, x + 19, 443)
    text("Brightness: 0 to 32 px", 526, 443)
    return canvas


def play_video(path, output, model, device, display):
    if output and (output.resolve() == path.resolve() or
                   (output.exists() and path.exists() and output.samefile(path))):
        raise ValueError("Choose an output file different from the input video.")
    capture = cv2.VideoCapture(str(path))
    writer = None
    count = 0
    try:
        ok, previous = capture.read()
        if not ok:
            raise ValueError(f"Could not read video: {path}")
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            fps = 30.0
        while True:
            ok, current = capture.read()
            if not ok:
                break
            if previous.shape != current.shape:
                raise ValueError("Frame sizes differ.")
            start = time.monotonic()
            field = estimate_flow(model, previous, current, device)
            canvas = overview(previous, field, f"Frames {count + 1} -> {count + 2}")
            if output:
                if writer is None:
                    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps,
                                             (canvas.shape[1], canvas.shape[0]))
                    if not writer.isOpened():
                        raise OSError(f"Could not create video: {output}")
                writer.write(canvas)
            count += 1
            if display:
                cv2.imshow(TITLE, canvas)
                delay = max(1, round(1000 * (1 / fps - (time.monotonic() - start))))
                key = cv2.waitKey(delay) & 255
                if key in (ord("q"), 27) or cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1:
                    break
            previous = current
        if not count:
            raise ValueError("The video needs at least two frames.")
        print(f"Processed {count} frame pairs ({fps:g} fps).")
    finally:
        capture.release()
        if writer is not None:
            writer.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", nargs="*", type=Path, metavar="IMAGE")
    parser.add_argument("--video", type=Path, nargs="?", const=LESSON / "data/freiburg1_xyz.mp4")
    parser.add_argument("--output", type=Path, help="Save the rendered PNG (pair) or MP4 (video).")
    parser.add_argument("--device", choices=["auto", "cpu", "mps", "cuda"], default="auto")
    args = parser.parse_args()
    if len(args.images) not in (0, 2) or (args.images and args.video):
        parser.error("Pass two images or --video [path].")
    device = args.device
    if device == "auto":
        device = "cpu"
        if torch.cuda.is_available():
            device = "cuda"
        elif torch.backends.mps.is_available():
            device = "mps"
    display = not os.environ.get("SLAM_HEADLESS")
    print(f"FlowNetS / {device}", flush=True)
    try:
        model = load_model(LESSON / "build/model-cache", device)
        if args.video:
            play_video(args.video, args.output, model, device, display)
        else:
            paths = args.images or [LESSON / "data/1.png", LESSON / "data/2.png"]
            if args.output and args.output.exists() and any(path.exists() and path.samefile(args.output) for path in paths):
                raise ValueError("Choose an output file different from the input images.")
            previous, current = [cv2.imread(str(path)) for path in paths]
            if previous is None or current is None or previous.shape != current.shape:
                raise ValueError("Provide two readable images of the same size.")
            field = estimate_flow(model, previous, current, device)
            dx, dy = np.median(field, axis=(0, 1))
            print(f"Median flow (dx, dy) in pixels: {dx:.2f}, {dy:.2f}")
            canvas = overview(previous, field, "Image pair")
            if args.output or not display:
                output = args.output or LESSON / "build/flownet_flow.png"
                if not cv2.imwrite(str(output), canvas):
                    raise OSError(f"Could not write {output}")
            if display:
                cv2.imshow(TITLE, canvas)
                cv2.waitKey(0)
    except (ValueError, RuntimeError, OSError, cv2.error) as exc:
        parser.exit(1, f"FlowNetS: {exc}\n")
    finally:
        if display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
