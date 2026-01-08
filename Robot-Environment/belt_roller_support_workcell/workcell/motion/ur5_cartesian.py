from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.motion_generation import ArticulationKinematicsSolver


Vec3 = Sequence[float]
Quat = Sequence[float]


@dataclass
class CartesianGoal:
    pos: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]


class UR5Cartesian:
    """
    UR5 Cartesian motion controller (same style as UR10).
    """

    def __init__(self, robot_prim_path: str, ee_frame_name: Optional[str] = None):
        self.robot = Articulation(robot_prim_path)
        self.ik = ArticulationKinematicsSolver(self.robot)
        self.ee_frame_name = ee_frame_name
        self._goal: Optional[CartesianGoal] = None

    def set_goal(self, pos: Vec3, quat_xyzw: Quat) -> None:
        self._goal = CartesianGoal(tuple(pos), tuple(quat_xyzw))

    def clear_goal(self) -> None:
        self._goal = None

    def update(self) -> bool:
        if self._goal is None:
            return False

        success, joint_positions = self.ik.compute_inverse_kinematics(
            target_position=self._goal.pos,
            target_orientation=self._goal.quat,
            end_effector_frame_name=self.ee_frame_name if self.ee_frame_name else None,
        )

        if not success:
            return False

        self.robot.apply_action(ArticulationAction(joint_positions=joint_positions))
        return True