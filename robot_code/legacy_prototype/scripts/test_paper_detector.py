import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.paper_detector import PaperDetector


def main():
    detector = PaperDetector()
    detection = detector.detect(frame=None)
    print(f"detection={detection}")


if __name__ == "__main__":
    main()
