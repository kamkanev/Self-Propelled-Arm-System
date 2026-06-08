class SafetyController:
    def __init__(self, base, arm=None):
        self.base = base
        self.arm = arm

    def stop_all(self):
        self.base.emergency_stop()
        if self.arm is not None:
            self.arm.safe_pose()

