import cv2.aruco
from camera import create_camera
import numpy as np
import cv2
import sys
sys.path.append('/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/build/modules/pybind')
import robot
import json
import time
import os
from dataset_collect.inout import save_json,save_im
from scipy.spatial.transform import Rotation as R


def detect_aruco(img,camera_matrix):
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
    arucoParams = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(arucoDict)
    (corners, ids, rejected) = detector.detectMarkers(img)
    marker_length=0.1
    corners_3d=np.array([[-marker_length/2,marker_length/2,0],[marker_length/2,marker_length/2,0],[marker_length/2,-marker_length/2,0],[-marker_length/2,-marker_length/2,0]])
    if ids is not None:
        corners_detect=np.squeeze(corners[0])
        flag,rvecs,tvecs=cv2.solvePnP(corners_3d,corners_detect,camera_matrix,np.zeros(5))
        
        cv2.drawFrameAxes(img, camera_matrix, np.zeros(5), rvecs, tvecs, 0.1)
        repoj_corners_subpix, _ = cv2.projectPoints(corners_3d, rvecs, tvecs, camera_matrix, np.zeros(5))
        repoj_corners_subpix=np.squeeze(repoj_corners_subpix)
        repoj_error=np.mean(np.linalg.norm(corners_detect-repoj_corners_subpix,axis=1))
        show_corner=(int(corners_detect[0][0]),int(corners_detect[0][1]))
        show_corner_reproj=(int(repoj_corners_subpix[0][0]),int(repoj_corners_subpix[0][1]))
        cv2.circle(img,show_corner,4,(255,0,0))
        cv2.circle(img,show_corner_reproj,4,(0,0,255))
        cv2.drawFrameAxes(img, camera_matrix, np.zeros(5), rvecs, tvecs, 0.1)
        print(f'reproje error:{repoj_error}')
        # print(f"rotation: {rvec} translation:{tvec}") 
        return rvecs,tvecs
    return None,None


if __name__ == "__main__":

    time_str=time.strftime('%Y%m%d%H%M', time.localtime())
    save_path='record/effector_real_'+time_str
    os.makedirs(save_path,exist_ok=True)
    robot.init()

    num=0
    gts=[]
    # Configure depth and color streams
    realsense_cam=create_camera('oak',1920,1080)
    camera_matrix,coeff=realsense_cam.get_intrinsics()

    camera_parm={'camera_matrix':realsense_cam.get_intrinsics()[0].tolist(),'discoeffs':realsense_cam.get_intrinsics()[1].tolist()}
    save_json(os.path.join(save_path,"camera.json"),camera_parm)
    try:
        while True:
            
            color_image = realsense_cam.get_color_image()
            save_image=color_image.copy()
            rvec,tvec=detect_aruco(color_image,camera_matrix)
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            # 如果key为空格
            if key & 0xFF == ord(' '):
                cv2.imwrite(os.path.join(save_path,f'{num:06d}.png'),save_image)
                pose=robot.get_status()
                Rend2base=R.from_euler('xyz',pose[3:]).as_matrix().tolist()
                tend2base=np.array(pose[0:3]).tolist()

                gt={"index":num,"R_e2b":Rend2base,"t_e2b":tend2base}
                gts.append(gt)
                num=num+1
                print(f'保存位姿{num}')
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        save_json(os.path.join(save_path,"record.json"),gts)
        robot.log_out()