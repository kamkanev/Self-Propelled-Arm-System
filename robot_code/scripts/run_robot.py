import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.arm_controller import ArmController
from control.base_controller import BaseController
from control.gripper_controller import GripperController
from control.safety import SafetyController
from perception.camera import Camera
from perception.paper_detector import PaperDetector
from workflow.state_machine import RobotStateMachine


def main():
    camera = Camera()
    base = BaseController()
    arm = ArmController()
    gripper = GripperController()
    safety = SafetyController(base, arm)
    state_machine = RobotStateMachine(camera, PaperDetector(), base, arm, gripper)

    try:
        for _ in range(3):
            if not state_machine.step():
                break
    finally:
        camera.release()
        safety.stop_all()


if __name__ == "__main__":
    main()
