import cv2.aruco
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
sys.path.append('/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/build/modules/pybind')
import robot
import json

all_status=[]

def get_intrinsics(profile):
    intr = profile.as_video_stream_profile().get_intrinsics()
    camera_matrix=np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    coeffs=intr.coeffs
    print(f"Camera Intrinsics:{camera_matrix}")
    print(f'coeffs:{coeffs}')
    return camera_matrix,coeffs

def detect_aruco(img,camera_matrix):
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(arucoDict)
    (corners, ids, rejected) = detector.detectMarkers(img)
    if ids is not None:
             
        # cv2.aruco.drawDetectedMarkers(img, corners, ids)
        rvecs, tvecs,_=cv2.aruco.estimatePoseSingleMarkers(corners, 0.06, camera_matrix, np.zeros(5))
        for rvec,tvec in zip(rvecs,tvecs):
            cv2.drawFrameAxes(img, camera_matrix, np.zeros(5), rvec, tvec, 0.1)
            # print(f"rotation: {rvec} translation:{tvec}") 
        return rvecs[0],tvecs[0]
    return None,None

def save_hand_eye_calibrate_data(num,rvec,tvec):
    robot_status=robot.get_status()
    camera_pose=np.hstack((rvec.squeeze(),tvec.squeeze())).tolist()
    print(camera_pose)
    print(robot_status)
    all_status.append({'num':num,'robot':robot_status,'camera':camera_pose})

if __name__ == "__main__":
    robot.init()

    num=0
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    color_profile = profile.get_stream(rs.stream.color)
    camera_matrix,coeff=get_intrinsics(color_profile)
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            rvec,tvec=detect_aruco(color_image,camera_matrix)
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            # 如果key为空格
            if key & 0xFF == ord(' '):
                cv2.imwrite(f'record/{num:06d}.jpg',color_image)
                save_hand_eye_calibrate_data(num,rvec,tvec)
                num=num+1
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        with open('record/record.json','w') as f:
            json.dump(all_status,f,indent=1)

        pipeline.stop()
        robot.log_out()