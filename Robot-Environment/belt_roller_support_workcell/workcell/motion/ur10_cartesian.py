from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from omni.isaac.core.utils.types import ArticulationAction

class UR10Cartesian:
    def __init__(self, prim_path: str):
        self.robot = Articulation(prim_path)
        self.ik = ArticulationKinematicsSolver(self.robot)

    def move_to(self, pos, quat):
        success, joints = self.ik.compute_inverse_kinematics(
            target_position=pos,
            target_orientation=quat
        )
        if success:
            self.robot.apply_action(
                ArticulationAction(joint_positions=joints)
            )