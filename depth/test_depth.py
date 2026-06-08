from PIL import Image
import cv2
import numpy as np

try:
    from ultralytics import YOLO  # type: ignore[import]
except ImportError as exc:
    raise ImportError(
        "The ultralytics package is required. Install it with `pip install ultralytics`."
    ) from exc

yolo_model = YOLO("../yolo11s.pt")

image_path = "data/dubai.png"

yolo_input = cv2.imread(image_path)

results = yolo_model(yolo_input)

person_boxes = []
for result in results:
    boxes = result.boxes.xyxy.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy()

    for box, cls in zip(boxes, classes):
        if result.names[int(cls)] == 'person':
            x1, y1, x2, y2 = map(int, box[:4])
            person_boxes.append((x1, y1, x2, y2))
            cv2.rectangle(yolo_input, (x1, y1), (x2, y2), (0, 255, 0), 2)

cv2.imshow("YOLOv11s Detection", yolo_input)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Midas Depth Perception
try:
    import torch
except ImportError as exc:
    raise ImportError(
        "The torch package is required. Install it with `pip install torch`."
    ) from exc

# Load Midas model
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
midas.to(device)
midas.eval()

# Load transform
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = midas_transforms.small_transform

# Prepare image for depth estimation
img_input = transform(yolo_input).to(device)

# Predict depth
with torch.no_grad():
    prediction = midas(img_input)
    prediction = torch.nn.functional.interpolate(
        prediction.unsqueeze(1),
        size=yolo_input.shape[:2],
        mode="bicubic",
        align_corners=False,
    ).squeeze()

depth_map = prediction.cpu().numpy()

# Create copy of original image for distance annotation
yolo_with_distance = cv2.imread(image_path)

# Add YOLO detections and depth information to the copy
for x1, y1, x2, y2 in person_boxes:
    # Draw bounding box
    cv2.rectangle(yolo_with_distance, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # Extract depth values within the bounding box
    box_depth = depth_map[y1:y2, x1:x2]
    
    # Calculate average depth and convert to distance in meters
    # MiDaS outputs inverse depth, so we take the reciprocal and scale
    avg_depth = np.mean(box_depth)
    distance_m = 1.0 / (avg_depth + 1e-6)  # Avoid division by zero
    
    # Display distance on the image
    text = f"Distance: {distance_m:.2f}m"
    cv2.putText(
        yolo_with_distance,
        text,
        (x1, y1 - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )

cv2.imshow("YOLO Detection with Distance", yolo_with_distance)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("output_detection_with_distance.png", yolo_with_distance)

# Normalize depth map for visualization
depth_normalized = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
depth_colormap = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)

cv2.imshow("Midas Depth Estimation", depth_colormap)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save detection output
# cv2.imwrite("output_detection.png", yolo_input)

# Create inverted depth map
depth_np_normalized = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
inv_depth_np_normalized = 1.0 - depth_np_normalized
inv_depth_colormap = cv2.applyColorMap((inv_depth_np_normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
cv2.imshow("Inverted Depth Colormap", inv_depth_colormap)
cv2.waitKey(0)
cv2.destroyAllWindows()

# # Save depth outputs
# cv2.imwrite("output_depth.png", depth_colormap)
# cv2.imwrite("output_depth_inverted.png", inv_depth_colormap)

