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

square_size = 0.01  # 标定板每个格子的尺寸（假设单位为米)

dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)  # 畸变系数  

def get_intrinsics(profile):
    intr = profile.as_video_stream_profile().get_intrinsics()
    camera_matrix=np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    coeffs=intr.coeffs
    print(f"Camera Intrinsics:{camera_matrix}")
    print(f'coeffs:{coeffs}')
    return camera_matrix,coeffs

def get_RT_from_chessboard(img,camera_matrix):
    '''
    :param img_path: 读取图片路径
    :param chess_board_x_num: 棋盘格x方向格子数
    :param chess_board_y_num: 棋盘格y方向格子数
    :param K: 相机内参
    :param chess_board_len: 单位棋盘格长度,mm
    :return: 相机外参
    '''
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(img, (11, 8), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)  

    
    # print(object_points)

    if ret:  # 如果成功找到了角点  
        corners_subpix=cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        obj_points = np.zeros((11*8,3),dtype=np.float64)
        obj_points[:,:2] = np.mgrid[0:11,0:8].T.reshape(-1,2) * square_size
        # 通过角点坐标和标定板的实际尺寸来计算标定板的位姿  
        _,rvecs, tvecs,_ = cv2.solvePnPRansac(obj_points, corners_subpix, camera_matrix, dist_coeffs) 
        RT=np.column_stack(((cv2.Rodrigues(rvecs))[0],tvecs))
        RT = np.row_stack((RT, np.array([0, 0, 0, 1]))) 
        # 计算重投影误差
        repoj_corners_subpix, _ = cv2.projectPoints(obj_points, rvecs, tvecs, camera_matrix, dist_coeffs)
        repoj_error=np.mean(np.linalg.norm(corners_subpix-repoj_corners_subpix,axis=1))
        cv2.drawFrameAxes(img, camera_matrix, np.zeros(5), rvecs, tvecs, 0.1)
        print(f'reproje error:{repoj_error}')
        rvecs=np.squeeze(rvecs)
        tvecs=np.squeeze(tvecs)
        return rvecs,tvecs
    else:
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
            rvec,tvec=get_RT_from_chessboard(color_image,camera_matrix)
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
        # realsense_cam.stop()
        robot.log_out()