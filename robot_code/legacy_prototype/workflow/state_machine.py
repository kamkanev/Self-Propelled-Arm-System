class RobotStateMachine:
    """Tiny placeholder state machine for smoke testing module wiring."""

    def __init__(self, camera, paper_detector, base, arm, gripper):
        self.camera = camera
        self.paper_detector = paper_detector
        self.base = base
        self.arm = arm
        self.gripper = gripper
        self.state = "SEARCH_TARGET"

    def step(self):
        print(f"[state] {self.state}")
        frame = self.camera.read()
        target = self.paper_detector.detect(frame)

        if self.state == "SEARCH_TARGET":
            if target is None:
                self.base.turn_left()
            else:
                self.base.stop()
                self.state = "GRASP_TARGET"
        elif self.state == "GRASP_TARGET":
            self.base.stop()
            self.arm.pre_grasp()
            self.gripper.open()
            self.arm.grasp()
            self.gripper.close()
            self.arm.lift()
            self.state = "SAFE_STOP"
        elif self.state == "SAFE_STOP":
            self.base.stop()
            self.arm.safe_pose()
            return False

        return True

