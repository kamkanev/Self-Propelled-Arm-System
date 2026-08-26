import time

from jetbot.robot import Robot


SPEED = 0.15
MOVE_SECONDS = 0.3
PAUSE_SECONDS = 0.4


def run_step(name, action, robot):
    print(f"[demo] {name}")
    action(SPEED)
    time.sleep(MOVE_SECONDS)
    robot.stop()
    time.sleep(PAUSE_SECONDS)


def main():
    robot = Robot()

    try:
        robot.stop()
        time.sleep(PAUSE_SECONDS)

        run_step("forward", robot.forward, robot)
        run_step("turn left", robot.left, robot)
        run_step("forward again", robot.forward, robot)
        run_step("turn right", robot.right, robot)
        run_step("backward", robot.backward, robot)

        print("[demo] done")
    finally:
        robot.stop()
        print("[demo] stopped")


if __name__ == "__main__":
    main()
