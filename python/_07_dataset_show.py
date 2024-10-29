import cv2.aruco
from camera import create_camera
import numpy as np
import cv2
import sys
import json
import time
import os
from dataset_collect.inout import save_json,load_json
from scipy.spatial.transform import Rotation as R
import glob

square_size = 0.01  # 标定板每个格子的尺寸（假设单位为米)

dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)  # 畸变系数  

if __name__ == "__main__":

    root_path='record/effector_real_202410262058'
    camera_path=os.path.join(root_path,'camera.json')
    robot_pose_path=os.path.join(root_path,'record.json')
    camera_matrix=np.array(load_json(camera_path)['camera_matrix'])
    calibration_save_path=os.path.join(root_path,'calibration_ax_zb.json')
    calibration_save=load_json(calibration_save_path)
    img_paths=glob.glob(os.path.join(root_path,'*.png'))
    img_paths=sorted(img_paths)
    robot_poses=load_json(robot_pose_path)

    Rbase2cam=np.array(calibration_save['Rbase2cam'])
    tbase2cam=np.array(calibration_save['tbase2cam'])
    Tbase2cam=np.vstack((np.hstack((Rbase2cam,tbase2cam[:,np.newaxis])),np.array([0,0,0,1])))



    num=0
    gts=[]
    # Configure depth and color streams
    realsense_cam=create_camera('oak',1920,1080)
    camera_matrix,coeff=realsense_cam.get_intrinsics()
    point=np.array([0,0,0,1])
    
    for robot_pose,img_path in zip(robot_poses,img_paths):
        color_image = cv2.imread(img_path)
        # Show images
        Rend2base=np.array(robot_pose['R_e2b'])
        tend2base=np.array(robot_pose['t_e2b'])
        Tend2base=np.vstack((np.hstack((Rend2base,tend2base[:,np.newaxis])),np.array([0,0,0,1])))
        Tend2cam=Tbase2cam@Tend2base
        point_proj=camera_matrix@Tend2cam[0:3,:]@point
        point_proj=point_proj[0:2]/point_proj[2]
        cv2.circle(color_image,(int(point_proj[0]),int(point_proj[1])),4,(0,255,0),thickness=10)

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        key = cv2.waitKey()
        # 如果key为空格
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break