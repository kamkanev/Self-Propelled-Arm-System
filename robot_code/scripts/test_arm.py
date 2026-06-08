import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.arm_controller import ArmController
from control.gripper_controller import GripperController


def main():
    arm = ArmController()
    gripper = GripperController()
    arm.home()
    gripper.open()
    arm.pre_grasp()
    arm.grasp()
    gripper.close()
    arm.lift()
    arm.safe_pose()


if __name__ == "__main__":
    main()
