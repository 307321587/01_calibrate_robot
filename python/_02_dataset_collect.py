from realsense_camera import RealsenseCamera
import cv2

realsense_cam=RealsenseCamera()
while True:
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    key = cv2.waitKey(1)