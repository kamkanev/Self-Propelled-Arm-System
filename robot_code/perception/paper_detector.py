from dataclasses import dataclass


@dataclass
class Detection:
    class_name: str
    confidence: float
    center_x: int
    bbox_height: int


class PaperDetector:
    """Placeholder paper detector.

    Later this should load best.pt with Ultralytics YOLO and return a Detection.
    """

    def __init__(self, model_path="../best.pt", confidence=0.5):
        self.model_path = model_path
        self.confidence = confidence

    def detect(self, frame):
        print(f"[paper_detector] no detection yet; model={self.model_path}")
        return None

