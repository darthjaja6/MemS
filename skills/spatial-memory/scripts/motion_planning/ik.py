"""World-frame tool goals solved with Drake from the measured joint configuration."""
from dataclasses import dataclass
from typing import Sequence
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import ArrayLike
from pydrake.all import (AddMultibodyPlantSceneGraph, CollisionFilterDeclaration, DiagramBuilder,
                        GeometrySet, InverseKinematics, Parser, RotationMatrix, SnoptSolver, Solve)
from scipy.spatial.transform import Rotation


@dataclass
class ToolGoal:
    """Endpoint constraint: contact position plus optional orientation/axes.

    quaternion (wxyz) is a preference unless fixed_orientation=True. Each axes
    pair constrains a tool-local direction to a world direction at the endpoint.
    These constraints do not require a constant orientation along the path.
    """
    position: ArrayLike
    quaternion: ArrayLike
    axes: Sequence[tuple[ArrayLike, ArrayLike]] = ()
    fixed_orientation: bool = False
    position_tolerance_m: float = .001
    angle_tolerance_rad: float = np.deg2rad(1.)


class CartesianIK:
    def __init__(self, urdf, srdf, names, robots, slices):
        builder = DiagramBuilder()
        self.plant, graph = AddMultibodyPlantSceneGraph(builder, 0.)
        Parser(self.plant).AddModels(str(urdf))
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName('planning_root'))
        self.plant.Finalize()
        for rule in ET.parse(srdf).getroot():
            a, b = [GeometrySet(self.plant.GetCollisionGeometriesForBody(self.plant.GetBodyByName(rule.get(k))))
                    for k in ('link1', 'link2')]
            graph.collision_filter_manager().Apply(CollisionFilterDeclaration().ExcludeBetween(a, b))
        self.diagram = builder.Build()
        self.root_context = self.diagram.CreateDefaultContext()
        self.context = self.plant.GetMyMutableContextFromRoot(self.root_context)
        self.indices = np.array([self.plant.GetJointByName(n).position_start() for n in names])
        if len(self.indices) != self.plant.num_positions():
            raise ValueError('Planning requires fixed-base robots with scalar joints')
        self.tools = {name: (self.plant.GetFrameByName(name + '_' + r['tip']), r['tool']['point_in_tip'])
                      for name, r in robots.items()}
        self.slices = slices

    def solve(self, current, goals, avoid_collision=False):
        q0 = np.zeros(self.plant.num_positions())
        q0[self.indices] = current
        self.plant.SetPositions(self.context, q0)
        ik = InverseKinematics(self.plant, self.context)
        if avoid_collision:
            ik.AddMinimumDistanceLowerBoundConstraint(0.)
        prog, q = ik.prog(), ik.q()
        world = self.plant.world_frame()
        movable = []
        for name, goal in goals.items():
            tip, point = self.tools[name]
            target = np.asarray(goal.position, dtype=float)
            rotation = RotationMatrix(Rotation.from_quat(goal.quaternion, scalar_first=True).as_matrix())
            ik.AddPositionConstraint(tip, point, world, target - goal.position_tolerance_m, target + goal.position_tolerance_m)
            if goal.fixed_orientation:
                ik.AddOrientationConstraint(world, rotation, tip, RotationMatrix(), goal.angle_tolerance_rad)
            for local, desired in goal.axes:
                ik.AddAngleBetweenVectorsConstraint(tip, local, world, desired, 0., goal.angle_tolerance_rad)
            ik.AddOrientationCost(world, rotation, tip, RotationMatrix(), 1.)
            movable.extend(self.indices[self.slices[name]])
        locked = np.setdiff1d(np.arange(len(q0)), movable)
        prog.AddBoundingBoxConstraint(q0[locked], q0[locked], q[locked])
        prog.AddQuadraticErrorCost(.05 * np.eye(len(q0)), q0, q)
        prog.SetInitialGuess(q, q0)
        prog.SetSolverOption(SnoptSolver.id(), 'Major optimality tolerance', 1e-4)
        result = Solve(prog)
        solution = result.GetSolution(q)[self.indices]
        if not result.is_success() or not np.isfinite(solution).all():
            raise ValueError(f'IK could not satisfy the tool goal: {result.get_solution_result()}')
        return solution
