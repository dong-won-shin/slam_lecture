"""Detect and match pretrained SuperPoint features in two views of a scene."""
import argparse
from pathlib import Path

import cv2
import numpy as np
import torch
from transformers import SuperPointForKeypointDetection, SuperPointImageProcessor

LESSON = Path(__file__).resolve().parents[1]
MODEL = "magic-leap-community/superpoint"
REVISION = "734450e9ffe229074f5998494ddc615475cdb20a"


@torch.inference_mode()
def extract_features(images):
    cache = LESSON / "build" / "model-cache"
    processor = SuperPointImageProcessor.from_pretrained(MODEL, revision=REVISION, cache_dir=cache)
    model = SuperPointForKeypointDetection.from_pretrained(
        MODEL, revision=REVISION, cache_dir=cache, use_safetensors=True
    ).eval()
    rgb = [cv2.cvtColor(image, cv2.COLOR_BGR2RGB) for image in images]
    inputs = processor(images=rgb, return_tensors="pt")
    outputs = model(**inputs)
    # Remove batch padding and map coordinates back to each original image.
    sizes = [image.shape[:2] for image in images]
    features = processor.post_process_keypoint_detection(outputs, sizes)
    return [(f["keypoints"].numpy(), f["descriptors"].numpy()) for f in features]


def match_features(first, second):
    points1, descriptors1 = first
    points2, descriptors2 = second
    if not len(points1) or not len(points2):
        return [], []
    matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = sorted(matcher.match(descriptors1, descriptors2), key=lambda m: m.distance)
    if len(matches) < 8:
        return matches, []
    xy1 = np.float32([points1[m.queryIdx] for m in matches])
    xy2 = np.float32([points2[m.trainIdx] for m in matches])
    cv2.setRNGSeed(0)
    fundamental, mask = cv2.findFundamentalMat(xy1, xy2, cv2.FM_RANSAC, 1.5, 0.99)
    if fundamental is None or mask is None:
        return matches, []
    return matches, [m for m, keep in zip(matches, mask.ravel()) if keep]


def display_tile(image, points):
    height, width = image.shape[:2]
    scale = min(440 / width, 264 / height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    tile = np.full((264, 440, 3), 24, np.uint8)
    offset = np.array([(440 - size[0]) // 2, (264 - size[1]) // 2])
    x, y = offset
    tile[y:y + size[1], x:x + size[0]] = cv2.resize(
        image, size, interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    )
    return tile, np.int32(np.rint(points * (np.array(size) / [width, height]) + offset))


def overview(images, features, matches, inliers):
    (left, xy1), (right, xy2) = [
        display_tile(image, points) for image, (points, _) in zip(images, features)
    ]
    gap = 8
    gutter = np.full((left.shape[0], gap, 3), 24, np.uint8)
    pair = np.hstack((left, gutter, right))
    keypoints, lines = pair.copy(), pair.copy()
    xy2 = xy2 + (left.shape[1] + gap, 0)
    for point in np.vstack((xy1, xy2)):
        cv2.circle(keypoints, tuple(point), 2, (45, 210, 255), 1, cv2.LINE_AA)
    shown = inliers[:50]
    for m in shown:
        a, b = tuple(xy1[m.queryIdx]), tuple(xy2[m.trainIdx])
        cv2.line(lines, a, b, (90, 215, 245), 1, cv2.LINE_AA)
        for endpoint in (a, b):
            cv2.circle(lines, endpoint, 3, (45, 210, 255), -1, cv2.LINE_AA)

    def caption(text):
        bar = np.full((32, pair.shape[1], 3), 24, np.uint8)
        cv2.putText(bar, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (235, 235, 235), 1, cv2.LINE_AA)
        return bar

    return np.vstack((
        caption(f"SuperPoint  |  View 1: {len(xy1)} keypoints    View 2: {len(xy2)} keypoints"),
        keypoints,
        caption(f"Mutual matches: {len(matches)}    RANSAC inliers: {len(inliers)}    Shown: {len(shown)}"),
        lines,
    ))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image1", type=Path, nargs="?", default=LESSON / "data/1.png")
    parser.add_argument("image2", type=Path, nargs="?", default=LESSON / "data/2.png")
    parser.add_argument("--output", type=Path, default=LESSON / "build/superpoint_matches.png")
    parser.add_argument("--show", action="store_true", help="Open the result in one window")
    args = parser.parse_args()

    images = []
    for path in (args.image1, args.image2):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            parser.error(f"Cannot read image: {path}")
        images.append(image)
    features = extract_features(images)
    matches, inliers = match_features(*features)
    result = overview(images, features, matches, inliers)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), result):
        raise OSError(f"Could not save {args.output}")
    for i, (points, descriptors) in enumerate(features, 1):
        print(f"View {i}: {len(points)} keypoints, descriptors {descriptors.shape}")
    print(f"Mutual matches: {len(matches)}; RANSAC inliers: {len(inliers)}")
    print(f"Saved {args.output}", flush=True)
    if args.show:
        cv2.imshow("SuperPoint matching", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
