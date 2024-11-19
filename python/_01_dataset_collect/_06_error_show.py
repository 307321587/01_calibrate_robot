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
from dataset_collect.inout import save_json,load_json
from scipy.spatial.transform import Rotation as R
from opengl_utils.OffScreenRender import PoseProcess

square_size = 0.01  # 标定板每个格子的尺寸（假设单位为米)

# dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)  # 畸变系数  

if __name__ == "__main__":

    robot.init()
    root_path='record/effector_real_202411080026'
    camera_path=os.path.join(root_path,'camera.json')
    camera_matrix=np.array(load_json(camera_path)['camera_matrix'])
    dist_coeffs=np.array(load_json(camera_path)['discoeffs'])
    calibration_save_path=os.path.join(root_path,'calibration_ax_zb.json')
    calibration_save=load_json(calibration_save_path)

    Rbase2cam=np.array(calibration_save['Rbase2cam'])
    tbase2cam=np.array(calibration_save['tbase2cam'])
    Tbase2cam=np.vstack((np.hstack((Rbase2cam,tbase2cam[:,np.newaxis])),np.array([0,0,0,1])))

    num=0
    gts=[]
    # Configure depth and color streams
    realsense_cam=create_camera('oak',1920,1080)
    camera_matrix,coeff=realsense_cam.get_intrinsics()
    color_image=realsense_cam.get_color_image()
    height,width,_=color_image.shape
    point=np.array([0,0,0,1])
    model_path='python/data/model/000000.obj'
    pose_render=PoseProcess(model_path,width,height,camera_matrix,1.0,2500.0,20.0,0.1,0.1,m2mm=False, opencv_flag=True)

    Rgri2end=np.array([[1,0,0],[0,1,0],[0,0,1]])
    Rgri2end=R.from_euler('xyz',[179.835327-180,-0.229587,-179.882217+180],degrees=True).as_matrix()
    # tgri2end=np.array([-0.041520+0.042,0.009143-0.00965,0.161])
    tgri2end=np.array([-0.042+0.042,0.009143-0.00965,0.157])
    # tgri2end=np.array([-0.041+0.042,0.009489-0.00965,0.159])

    # tgri2end=np.array([0,0,0.1635])
    Tgri2end_init=np.vstack((np.hstack((Rgri2end,tgri2end[:,np.newaxis])),np.array([0,0,0,1])))

    try:
        while True:
            
            color_image = realsense_cam.get_color_image()
            # Show images
            
            pose=robot.get_status()
            Rend2base=R.from_euler('xyz',pose[3:]).as_matrix()
            tend2base=np.array(pose[0:3])
            Tend2base=np.vstack((np.hstack((Rend2base,tend2base[:,np.newaxis])),np.array([0,0,0,1])))
            Tend2cam=Tbase2cam@Tend2base
            point_proj=camera_matrix@Tend2cam[0:3,:]@point
            point_proj=point_proj[0:2]/point_proj[2]
            color_image=cv2.undistort(color_image,camera_matrix,dist_coeffs)
            if True:
                Tgri2cam=Tbase2cam@Tend2base@Tgri2end_init
                Tgri2cam[0:3,3]*=1000
                edge_img=pose_render.PoseShow(Tgri2cam)
                # edge_img=cv2.undistort(edge_img,camera_matrix,dist_coeffs)
                edge_img=cv2.ximgproc.thinning(edge_img)
                color_image[edge_img>0]=[0,0,255]
           
            cv2.circle(color_image,(int(point_proj[0]),int(point_proj[1])),4,(0,255,0),thickness=10)

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            # 如果key为空格
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # realsense_cam.stop()
        robot.log_out()