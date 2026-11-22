"""Build the lecture graph from the KITTI odometry ground-truth poses/00.txt."""
import argparse
from pathlib import Path

import cv2
import numpy as np


# Frame pairs verified by the images in lesson 13.
REVISITS = ((505, 3499), (732, 3670), (920, 3830), (15, 4464))


def pose_fields(transform):
    rotation_vector = cv2.Rodrigues(transform[:3, :3])[0].ravel()
    angle = np.linalg.norm(rotation_vector)
    scale = np.sin(angle / 2) / angle if angle else 0.5
    xyz = scale * rotation_vector
    values = [*transform[:3, 3], *xyz, np.cos(angle / 2)]
    return " ".join(f"{value:.10g}" for value in values)


def edge_line(i, j, relative, diagonal):
    information = np.diag(diagonal)[np.triu_indices(6)]
    weights = " ".join(str(value) for value in information)
    return f"EDGE_SE3:QUAT {i} {j} {pose_fields(relative)} {weights}"


def build_graph(source, output):
    raw = np.loadtxt(source).reshape(-1, 3, 4)
    frame_ids = sorted(set(range(15, 4465, 10)).union(*REVISITS))
    # Express both world and local camera frames in a right-handed Y-up basis.
    basis = np.diag([1, -1, -1, 1])
    reference = {}
    for frame in frame_ids:
        transform = np.eye(4)
        transform[:3] = raw[frame]
        reference[frame] = basis @ transform @ basis

    # Integrate the relative transforms, starting at the first keyframe.
    odometry = np.eye(4)
    vertices = [f"VERTEX_SE3:QUAT 0 {pose_fields(odometry)}"]
    edges = []
    for j, (a, b) in enumerate(zip(frame_ids, frame_ids[1:]), start=1):
        relative = np.linalg.solve(reference[a], reference[b])
        distance = np.linalg.norm(relative[:3, 3])
        yaw_bias = np.deg2rad(0.007 * distance)
        relative[:3, :3] = relative[:3, :3] @ cv2.Rodrigues(np.array([0, yaw_bias, 0]))[0]
        odometry = odometry @ relative
        vertices.append(f"VERTEX_SE3:QUAT {j} {pose_fields(odometry)}")
        edges.append(edge_line(j - 1, j, relative, [25] * 3 + [10000] * 3))

    for a, b in REVISITS:
        relative = np.linalg.solve(reference[a], reference[b])
        edges.append(edge_line(frame_ids.index(a), frame_ids.index(b), relative,
                               [100] * 3 + [40000] * 3))

    header = [
        "# KITTI odometry 00, frames 15-4464. Derived from KITTI ground truth.",
        "# A. Geiger, P. Lenz, C. Stiller, R. Urtasun. CC BY-NC-SA 3.0.",
        "# Simulated odometry yaw drift and four reference-derived revisit measurements.",
    ]
    output.write_text("\n".join(header + vertices + ["FIX 0"] + edges) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("poses", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    build_graph(args.poses, args.output)
