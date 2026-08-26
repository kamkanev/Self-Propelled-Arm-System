class ArmController:
    """Mock arm controller with named poses."""

    def home(self):
        print("[arm] home")

    def pre_grasp(self):
        print("[arm] pre_grasp")

    def grasp(self):
        print("[arm] grasp")

    def lift(self):
        print("[arm] lift")

    def carry(self):
        print("[arm] carry")

    def release_pose(self):
        print("[arm] release_pose")

    def safe_pose(self):
        print("[arm] safe_pose")

