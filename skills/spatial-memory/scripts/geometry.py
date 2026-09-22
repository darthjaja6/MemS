#!/usr/bin/env python3
"""Small metric geometry tools. No device access; metres and optical camera axes."""

import argparse
import json
import sys
from itertools import combinations
import numpy as np


def unit(vector):
    v = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(v)
    if v.shape != (3,) or not np.isfinite(v).all() or norm < 1e-12:
        raise ValueError("Expected a finite nonzero 3-vector")
    return v / norm


def transform_points(T, points):
    p = np.asarray(points, dtype=float)
    T = np.asarray(T, dtype=float)
    return p @ T[:3, :3].T + T[:3, 3]


def plane(points, normal=None):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError("Supply finite 3D points")
    if normal is None:
        if len(p) < 3:
            raise ValueError("An unknown plane requires three non-collinear 3D points")
        _, s, vh = np.linalg.svd(p - p.mean(axis=0), full_matrices=False)
        if s[1] < 1e-9:
            raise ValueError("Points are collinear")
        n = vh[-1]
        if n[2] < 0:
            n = -n
    else:
        n = unit(normal)
    offset = -float(n @ p.mean(axis=0))
    residuals = p @ n + offset
    return {
        "normal": n.tolist(),
        "offset_m": offset,
        "fit_rms_m": float(np.sqrt(np.mean(residuals**2))),
    }


def ray(pixel, K, T_base_camera, distortion=None):
    pixel = np.asarray(pixel, dtype=float)
    K = np.asarray(K, dtype=float)
    T = np.asarray(T_base_camera, dtype=float)
    if distortion is not None and np.any(distortion):
        import cv2

        xy = cv2.undistortPoints(
            pixel.reshape(1, 1, 2), K, np.asarray(distortion, dtype=float)
        ).reshape(2)
        local = np.r_[xy, 1.0]
    else:
        local = np.linalg.solve(K, np.r_[pixel, 1.0])
    return T[:3, 3], unit(T[:3, :3] @ local)


def ray_plane(origin, direction, normal, offset_m, height_m=0):
    origin = np.asarray(origin, dtype=float)
    d = unit(direction)
    raw_n = np.asarray(normal, dtype=float)
    n = unit(raw_n)
    offset = float(offset_m) / np.linalg.norm(raw_n) - float(height_m)
    denominator = float(n @ d)
    if abs(denominator) < 1e-9:
        raise ValueError("The ray is parallel to the plane")
    distance = -(float(n @ origin) + offset) / denominator
    if distance < 0:
        raise ValueError("The plane intersection is behind the ray origin")
    return {"point_m": (origin + distance * d).tolist(), "distance_m": distance}


def pixel_plane(pixel, K, T_base_camera, normal, offset_m, height_m=0, distortion=None):
    c, d = ray(pixel, K, T_base_camera, distortion)
    return ray_plane(c, d, normal, offset_m, height_m)


def triangulate(views):
    if len(views) < 2:
        raise ValueError("Need two translated camera views of the same feature")
    rays = [ray(**view) for view in views]
    matrices = [np.eye(3) - np.outer(d, d) for _, d in rays]
    A = np.vstack(matrices)
    b = np.concatenate([M @ c for M, (c, _) in zip(matrices, rays)])
    position, _, rank, singular = np.linalg.lstsq(A, b, rcond=None)
    if rank < 3 or singular[-1] < 1e-7:
        raise ValueError("Insufficient translation/parallax to locate this feature")
    if any((position - c) @ d <= 0 for c, d in rays):
        raise ValueError("Reconstructed feature lies behind a camera")
    distances = [
        np.linalg.norm(M @ (position - c)) for M, (c, _) in zip(matrices, rays)
    ]
    angles = [
        np.degrees(np.arccos(np.clip(a @ b, -1, 1)))
        for (_, a), (_, b) in combinations(rays, 2)
    ]
    return {
        "point_m": position.tolist(),
        "ray_rms_m": float(np.sqrt(np.mean(np.square(distances)))),
        "max_parallax_deg": float(max(angles)),
        "note": "Ray consistency depends on the supplied camera poses; it is not absolute accuracy.",
    }


def camera_pixels(points, K, distortion=None):
    """Project camera-frame points into the calibrated image's pixel coordinates."""
    points = np.asarray(points, dtype=float).reshape(-1, 3).copy()
    points[:, 2] = np.maximum(points[:, 2], 1e-6)
    K = np.asarray(K, dtype=float)
    if distortion is not None and np.any(distortion):
        import cv2
        pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K,
                                     np.asarray(distortion, dtype=float))
        return pixels.reshape(-1, 2)
    homogeneous = points @ K.T
    return homogeneous[:, :2] / homogeneous[:, 2:]


def project(point, K, T_base_camera, distortion=None):
    T = np.asarray(T_base_camera, dtype=float)
    local = T[:3, :3].T @ (np.asarray(point) - T[:3, 3])
    if local[2] <= 0:
        raise ValueError("Point is behind camera")
    return camera_pixels([local], K, distortion)[0].tolist()


def camera_looking_at(position, target, up=(0, 0, 1)):
    z = unit(np.asarray(target) - position)
    x = unit(np.cross(z, up))
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = position
    return T


def pose_matrix(pose):
    from scipy.spatial.transform import Rotation
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all():
        raise ValueError("Pose must be [x,y,z,qw,qx,qy,qz]")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(p[[4, 5, 6, 3]]).as_matrix()
    T[:3, 3] = p[:3]
    return T


def matrix_pose(T):
    from scipy.spatial.transform import Rotation
    T = np.asarray(T, dtype=float)
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    return [*T[:3, 3].tolist(), *q[[3, 0, 1, 2]].tolist()]


def tool_pose(target, approach, closing, point_in_tip=(0, 0, 0),
              local_approach=(1, 0, 0), local_closing=(0, 1, 0)):
    """Place a tool contact point while aligning approach and jaw-closing axes."""
    def basis(a, b):
        a, b = unit(a), unit(b)
        if abs(float(a @ b)) > 1e-5:
            raise ValueError("Approach and closing directions must be perpendicular")
        return np.column_stack([a, b, np.cross(a, b)])
    T = np.eye(4)
    T[:3, :3] = basis(approach, closing) @ basis(local_approach, local_closing).T
    T[:3, 3] = np.asarray(target) - T[:3, :3] @ np.asarray(point_in_tip)
    return {"pose": matrix_pose(T)}


def distance(a, b):
    return {"distance_m": float(np.linalg.norm(np.asarray(a) - np.asarray(b)))}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "operation",
        choices=["plane", "ray-plane", "pixel-plane", "triangulate", "transform",
                 "ray", "project", "tool-pose", "distance"],
    )
    parser.add_argument("--input", type=argparse.FileType("r"), default=sys.stdin)
    args = parser.parse_args()
    data = json.load(args.input)
    try:
        operation = {
            "plane": plane,
            "ray-plane": ray_plane,
            "pixel-plane": pixel_plane,
            "triangulate": triangulate,
            "transform": transform_points,
            "ray": lambda **kw: dict(zip(("origin", "direction"),
                                        [v.tolist() for v in ray(**kw)])),
            "project": project,
            "tool-pose": tool_pose,
            "distance": distance,
        }[args.operation]
        result = operation(**data)
        if isinstance(result, np.ndarray):
            result = result.tolist()
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    except (ValueError, np.linalg.LinAlgError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
