#!/usr/bin/env python3
"""URDF serial-chain FK / position IK for exploratory geometry, without hardware I/O."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def origin(element):
    T = np.eye(4)
    if element is not None:
        T[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        T[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return T


class Robot:
    def __init__(self, urdf):
        self.path = Path(urdf).resolve()
        root = ET.parse(self.path).getroot()
        self.name = root.get("name")
        self.links = {link.get("name"): link for link in root.findall("link")}
        self.joints = {}
        self.by_child = {}
        self._visual_points = None
        for element in root.findall("joint"):
            kind = element.get("type")
            if (
                kind not in ("fixed", "revolute", "continuous", "prismatic")
                or element.find("mimic") is not None
            ):
                raise ValueError(
                    "This small helper supports ordinary non-mimic serial joints; use the robot's adapter for other models"
                )
            axis_element = element.find("axis")
            axis = np.fromstring(
                axis_element.get("xyz", "1 0 0")
                if axis_element is not None
                else "1 0 0",
                sep=" ",
            )
            if kind != "fixed":
                axis = axis / np.linalg.norm(axis)
            limit = element.find("limit")
            limits = (
                [-np.pi, np.pi]
                if kind == "continuous"
                else [float(limit.get("lower", "0")), float(limit.get("upper", "0"))]
                if limit is not None
                else [0, 0]
            )
            joint = {
                "name": element.get("name"),
                "parent": element.find("parent").get("link"),
                "child": element.find("child").get("link"),
                "type": kind,
                "origin": origin(element.find("origin")),
                "axis": axis,
                "limits": limits,
            }
            self.joints[joint["name"]] = joint
            self.by_child[joint["child"]] = joint
        roots = set(self.links) - set(self.by_child)
        if len(roots) != 1:
            raise ValueError("Expected one robot root")
        self.root = roots.pop()

    def chain(self, tip):
        chain = []
        while tip != self.root:
            joint = self.by_child[tip]
            chain.append(joint)
            tip = joint["parent"]
        return list(reversed(chain))

    def frames(self, q=None):
        q = q or {}
        frames = {self.root: np.eye(4)}
        pending = list(self.joints.values())
        while pending:
            next_pending = []
            for joint in pending:
                if joint["parent"] not in frames:
                    next_pending.append(joint)
                    continue
                M = np.eye(4)
                value = float(q.get(joint["name"], 0))
                if joint["type"] in ("revolute", "continuous"):
                    M[:3, :3] = Rotation.from_rotvec(joint["axis"] * value).as_matrix()
                elif joint["type"] == "prismatic":
                    M[:3, 3] = joint["axis"] * value
                frames[joint["child"]] = frames[joint["parent"]] @ joint["origin"] @ M
            if len(next_pending) == len(pending):
                raise ValueError("Disconnected or cyclic joint tree")
            pending = next_pending
        return frames

    def visual_points(self, q=None, links=None):
        """World vertices of binary STL visuals, merged per link (metres)."""
        if self._visual_points is None:
            grouped = {}
            for visual in self.description()["visuals"]:
                path = self.path.parent / visual["mesh"]
                data = path.read_bytes()
                count = int.from_bytes(data[80:84], "little")
                if len(data) != 84 + count * 50:
                    raise ValueError(f"Expected a binary STL: {path}")
                dtype = np.dtype(
                    [
                        ("normal", "<f4", (3,)),
                        ("vertices", "<f4", (3, 3)),
                        ("attribute", "<u2"),
                    ]
                )
                points = np.frombuffer(data, dtype=dtype, count=count, offset=84)[
                    "vertices"
                ].reshape(-1, 3)
                points = np.unique(points, axis=0) * np.asarray(visual["scale"])
                T = np.asarray(visual["origin"])
                grouped.setdefault(visual["link"], []).append(
                    points @ T[:3, :3].T + T[:3, 3]
                )
            self._visual_points = {
                link: np.concatenate(parts) for link, parts in grouped.items()
            }
        selected = set(links) if links is not None else set(self._visual_points)
        if not selected <= self.links.keys():
            raise ValueError("Unknown link requested for visual geometry")
        frames = self.frames(q)
        return {
            link: points @ frames[link][:3, :3].T + frames[link][:3, 3]
            for link, points in self._visual_points.items()
            if link in selected
        }

    def path_plane_clearance(
        self,
        waypoints,
        normal,
        offset_m,
        *,
        links=None,
        extra_geometry=None,
        samples_per_segment=16,
    ):
        """Sample joint-linear paths against a plane, including local attachments.

        extra_geometry maps link names to local vertices for cameras/carried items.
        Returns sampled minimum signed distance per link; this is a geometry
        estimate with the supplied mesh and sampling resolution.
        """
        if not waypoints or samples_per_segment < 2:
            raise ValueError("Provide waypoints and at least two samples per segment")
        normal = np.asarray(normal, dtype=float)
        length = np.linalg.norm(normal)
        if normal.shape != (3,) or not np.isfinite(normal).all() or length < 1e-12:
            raise ValueError("A finite nonzero plane normal is required")
        normal = normal / length
        offset_m = float(offset_m) / length
        minimum = {}
        states, current = [], {}
        for waypoint in waypoints:
            current = {**current, **waypoint}
            states.append(current)
        pairs = list(zip(states, states[1:])) or [(states[0], states[0])]
        for start, changes in pairs:
            end = {**start, **changes}
            for t in np.linspace(0, 1, samples_per_segment):
                q = {
                    j: start.get(j, 0.0) + t * (v - start.get(j, 0.0))
                    for j, v in end.items()
                }
                clouds = self.visual_points(q, links)
                frames = self.frames(q) if extra_geometry else {}
                for link, points in (extra_geometry or {}).items():
                    T = frames[link]
                    clouds[f"{link}:attachment"] = (
                        np.asarray(points) @ T[:3, :3].T + T[:3, 3]
                    )
                for link, points in clouds.items():
                    distance = float(np.min(points @ normal + offset_m))
                    minimum[link] = min(minimum.get(link, float("inf")), distance)
        return minimum

    def ik(
        self,
        target,
        tip,
        seed=None,
        tool_z=None,
        joint_bounds=None,
        seed_weight=0.00005,
        orientation_axes=None,
        point_in_tip=None,
    ):
        seed = dict(seed or {})
        chain = [j for j in self.chain(tip) if j["type"] != "fixed"]
        names = [j["name"] for j in chain]
        lower, upper = np.array([j["limits"] for j in chain]).T
        for name, bounds in (joint_bounds or {}).items():
            if name not in names or len(bounds) != 2 or not np.isfinite(bounds).all():
                raise ValueError(
                    "Joint bounds require a chain joint and two finite radians/metres"
                )
            index = names.index(name)
            lower[index] = max(lower[index], bounds[0])
            upper[index] = min(upper[index], bounds[1])
        if np.any(lower >= upper) or not np.isfinite(seed_weight) or seed_weight < 0:
            raise ValueError(
                "Joint bounds must overlap model limits with positive width; seed weight must be nonnegative"
            )
        x0 = np.clip([seed.get(n, 0.0) for n in names], lower + 1e-8, upper - 1e-8)
        target = np.asarray(target, dtype=float)
        point_in_tip = np.asarray(
            point_in_tip if point_in_tip is not None else [0.0, 0.0, 0.0], dtype=float
        )
        if point_in_tip.shape != (3,) or not np.isfinite(point_in_tip).all():
            raise ValueError("point_in_tip must be a finite local 3-vector")
        # Tool axes are supplied by the gripper model; local z need not be
        # its approach direction, and local x need not be its closing axis.
        axes = []

        def normalized(value):
            value = np.asarray(value, dtype=float)
            if (
                value.shape != (3,)
                or not np.isfinite(value).all()
                or np.linalg.norm(value) < 1e-12
            ):
                raise ValueError("Orientation axes require finite nonzero 3-vectors")
            return value / np.linalg.norm(value)

        if tool_z is not None:
            axes.append(("tool_z", np.array([0.0, 0.0, 1.0]), normalized(tool_z), 10.0))
        for i, axis in enumerate(orientation_axes or []):
            tolerance = float(axis.get("tolerance_deg", 10.0))
            if not np.isfinite(tolerance) or not 0 < tolerance <= 180:
                raise ValueError("Axis tolerance must be in (0, 180] degrees")
            axes.append(
                (
                    axis.get("name", f"axis_{i}"),
                    normalized(axis["local"]),
                    normalized(axis["world"]),
                    tolerance,
                )
            )

        def angular_errors(T):
            return [
                float(
                    np.degrees(np.arccos(np.clip((T[:3, :3] @ local) @ world, -1, 1)))
                )
                for _, local, world, _ in axes
            ]

        def residual(x):
            T = self.frames({**seed, **dict(zip(names, x))})[tip]
            parts = [T[:3, :3] @ point_in_tip + T[:3, 3] - target]
            for name, local, world, tolerance in axes:
                delta = T[:3, :3] @ local - world
                if name != "tool_z":
                    # A task tolerance is a cone, not a demand for zero error.
                    # Leave a small margin for the final feasibility decision.
                    distance = np.linalg.norm(delta)
                    allowed = 2 * np.sin(np.radians(tolerance * 0.95) / 2)
                    delta = delta * max(0.0, 1 - allowed / max(distance, 1e-12))
                parts.append(0.04 * delta)
            parts.append(seed_weight * (x - x0))
            return np.concatenate(parts)

        candidates = [x0, (lower + upper) / 2]
        best = None
        for start in candidates:
            result = least_squares(
                residual,
                start,
                bounds=(lower, upper),
                max_nfev=100,
                ftol=1e-9,
                xtol=1e-9,
                gtol=1e-9,
            )
            if best is None or np.linalg.norm(residual(result.x)) < np.linalg.norm(
                residual(best.x)
            ):
                best = result
            T_best = self.frames({**seed, **dict(zip(names, best.x))})[tip]
            if np.linalg.norm(residual(best.x)[:3]) < 0.0005 and all(
                angle < axis[3] for angle, axis in zip(angular_errors(T_best), axes)
            ):
                break
        q = {**seed, **{n: float(v) for n, v in zip(names, best.x)}}
        T = self.frames(q)[tip]
        controlled_point = T[:3, :3] @ point_in_tip + T[:3, 3]
        error = float(np.linalg.norm(controlled_point - target))
        errors = angular_errors(T)
        angle = errors[0] if tool_z is not None else None
        return {
            "q_rad": q,
            "tip_position_m": T[:3, 3].tolist(),
            "position_error_m": error,
            "point_in_tip_m": point_in_tip.tolist(),
            "controlled_point_m": controlled_point.tolist(),
            "tool_z_error_deg": angle,
            "orientation_axes": [
                {"name": a[0], "error_deg": e, "tolerance_deg": a[3]}
                for a, e in zip(axes, errors)
            ],
            "reachable_in_model": error < 0.003
            and all(e < a[3] for e, a in zip(errors, axes)),
            "note": "Nominal geometry only; no trajectory or hardware command is produced.",
        }

    def description(self):
        visuals = []
        for name, link in self.links.items():
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    continue
                material = visual.find("material")
                visuals.append(
                    {
                        "link": name,
                        "mesh": mesh.get("filename"),
                        "scale": np.fromstring(
                            mesh.get("scale", "1 1 1"), sep=" "
                        ).tolist(),
                        "origin": origin(visual.find("origin")).tolist(),
                        "material": material.get("name")
                        if material is not None
                        else "default",
                    }
                )
        joints = [
            {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in j.items()}
            for j in self.joints.values()
        ]
        return {
            "name": self.name,
            "root": self.root,
            "joints": joints,
            "visuals": visuals,
        }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["inspect", "fk", "ik", "bounds", "clearance"])
    parser.add_argument("--input", type=argparse.FileType('r'))
    parser.add_argument("--links", nargs='+')
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--tip")
    parser.add_argument("--q", default="{}", help="Joint name to radians mapping")
    parser.add_argument("--target", type=float, nargs=3)
    parser.add_argument("--tool-z", type=float, nargs=3)
    parser.add_argument(
        "--joint-bounds",
        default="{}",
        help="Optional tighter joint ranges in radians/metres",
    )
    parser.add_argument("--seed-weight", type=float, default=0.00005)
    parser.add_argument(
        "--orientation-axes",
        default="[]",
        help="Local tool vectors and desired world directions",
    )
    parser.add_argument(
        "--point-in-tip", type=float, nargs=3, help="Local point to place at the target"
    )
    args = parser.parse_args()
    robot = Robot(args.urdf)
    q = json.loads(args.q)
    if args.operation == "inspect":
        result = robot.description()
    elif args.operation == 'bounds':
        result = {name: dict(min=points.min(axis=0).tolist(), max=points.max(axis=0).tolist())
                  for name, points in robot.visual_points(q, args.links).items()}
    elif args.operation == 'clearance':
        if args.input is None:
            parser.error('clearance requires --input')
        result = robot.path_plane_clearance(**json.load(args.input))
    elif args.operation == "fk":
        frames = robot.frames(q)
        result = (
            frames[args.tip].tolist()
            if args.tip
            else {k: v.tolist() for k, v in frames.items()}
        )
    else:
        if not args.tip or args.target is None:
            parser.error("ik requires --tip and --target")
        result = robot.ik(
            args.target,
            args.tip,
            q,
            args.tool_z,
            json.loads(args.joint_bounds),
            args.seed_weight,
            json.loads(args.orientation_axes),
            args.point_in_tip,
        )
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
