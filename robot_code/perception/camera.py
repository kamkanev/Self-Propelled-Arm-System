class Camera:
    """Small camera placeholder.

    Replace this with an OpenCV-backed implementation when testing on the robot.
    """

    def __init__(self, index=0):
        self.index = index

    def read(self):
        print(f"[camera] read frame from camera {self.index} (placeholder)")
        return None

    def release(self):
        print("[camera] release")

