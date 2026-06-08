import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.base_controller import BaseController


def main():
    base = BaseController()
    base.forward()
    base.turn_left()
    base.turn_right()
    base.stop()


if __name__ == "__main__":
    main()
