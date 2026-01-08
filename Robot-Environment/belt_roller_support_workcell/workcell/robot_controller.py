from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from __future__ import annotations

import numpy as np
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from omni.isaac.motion_generation.lula import RmpFlow

class URCartesianController:
    def __init__(self, prim_path: str, urdf_name: str):
        self.robot = Articulation(prim_path=prim_path)
        self.robot.initialize()

        self.ik = ArticulationKinematicsSolver(
            robot_articulation=self.robot,
            end_effector_frame_name="tool0",
            robot_description_path=urdf_name,
        )

        self.target_pos = None
        self.target_quat = None

    def set_goal(self, position, orientation_xyzw):
        self.target_pos = np.array(position)
        self.target_quat = np.array(orientation_xyzw)

    def update(self):
        if self.target_pos is None:
            return

        action, success = self.ik.compute_inverse_kinematics(
            target_position=self.target_pos,
            target_orientation=self.target_quat,
        )

        if success:
            self.robot.apply_action(action)