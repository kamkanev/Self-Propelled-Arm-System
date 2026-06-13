import numpy as np
import cv2
import os
import sys
import time
from aruco_dict import ARUCO_DICT, get_default_tag_size, get_max_marker_id, get_predefined_dictionary

def aruco_display(corners, ids, rejected, image):
    
    if len(corners) > 0:

        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):
            
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] ArUco marker ID: {}".format(markerID))
    return image

def draw_axis(image, camera_matrix, dist_coeffs, rvec, tvec, length=0.05):
    axis_points = np.float32([
        [0, 0, 0],
        [length, 0, 0],
        [0, length, 0],
        [0, 0, -length],
    ])
    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    origin = tuple(imgpts[0].ravel().astype(int))
    x_axis = tuple(imgpts[1].ravel().astype(int))
    y_axis = tuple(imgpts[2].ravel().astype(int))
    z_axis = tuple(imgpts[3].ravel().astype(int))
    cv2.line(image, origin, x_axis, (0, 0, 255), 3)
    cv2.line(image, origin, y_axis, (0, 255, 0), 3)
    cv2.line(image, origin, z_axis, (255, 0, 0), 3)
    return image


def estimate_pose(image, aruco_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = get_predefined_dictionary(aruco_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        marker_length = 0.05  # physical marker side length in meters
        object_points = np.array([
            [-marker_length / 2, marker_length / 2, 0.0],
            [marker_length / 2, marker_length / 2, 0.0],
            [marker_length / 2, -marker_length / 2, 0.0],
            [-marker_length / 2, -marker_length / 2, 0.0],
        ], dtype=np.float32)

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_corners = marker_corners.reshape((4, 2)).astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                marker_corners,
                matrix_coefficients,
                distortion_coefficients,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if success:
                image = draw_axis(image, matrix_coefficients, distortion_coefficients, rvec, tvec, length=marker_length * 0.5)
                cv2.putText(
                    image,
                    f"ID {int(marker_id)}",
                    tuple(marker_corners[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

    return image

aruco_type = "DICT_6X6_250"
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
matrix_coefficients = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
distortion_coefficients = np.zeros((5, 1), dtype=np.float32)

aruco_dict = get_predefined_dictionary(aruco_type)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

while cap.isOpened():
    ret, image = cap.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    corners, ids, rejected_img_points = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    output_image = aruco_display(corners, ids, rejected_img_points, image)
    cv2.imshow('Pose Estimation', output_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
