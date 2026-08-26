class BaseController:
    """Mock chassis controller for early software testing."""

    def set_velocity(self, linear, angular):
        print(f"[base] velocity linear={linear:.2f}, angular={angular:.2f}")

    def forward(self, speed=0.2):
        self.set_velocity(speed, 0.0)

    def backward(self, speed=0.2):
        self.set_velocity(-speed, 0.0)

    def turn_left(self, speed=0.25):
        self.set_velocity(0.0, speed)

    def turn_right(self, speed=0.25):
        self.set_velocity(0.0, -speed)

    def stop(self):
        print("[base] stop")

    def emergency_stop(self):
        print("[base] emergency stop")

