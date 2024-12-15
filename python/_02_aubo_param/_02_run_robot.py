import numpy as np
import sys
import time
sys.path.append('./build/modules/pybind')
print(sys.path)
import robot
import cv2
import os
from _01_dataset_collect.dataset_collect.inout import save_json,load_json
from _01_dataset_collect.camera import create_camera
from scipy.spatial.transform import Rotation as R
from _02_aubo_param._01_pbvs import pbvs_straight
square_size = 0.01  # 标定板每个格子的尺寸（假设单位为米)
dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)  # 畸变系数  

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

def detect_aruco(img,camera_matrix):
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
    arucoParams = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(arucoDict)
    (corners, ids, rejected) = detector.detectMarkers(img)
    marker_length=0.1
    corners_3d=np.array([[marker_length/2,marker_length/2,0],[-marker_length/2,marker_length/2,0],[-marker_length/2,-marker_length/2,0],[marker_length/2,-marker_length/2,0]])
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
        # print(f'reproje error:{repoj_error}')
        rvecs=np.squeeze(rvecs)
        tvecs=np.squeeze(tvecs)
        # print(f"rotation: {rvec} translation:{tvec}") 
        return rvecs,tvecs
    return None,None


def main():

    ################ 初始化相机 ################
    root_path='record/effector_real_202411271710'
    camera_path=os.path.join(root_path,'camera.json')
    camera_matrix=np.array(load_json(camera_path)['camera_matrix'])
    dist_coeffs=np.array(load_json(camera_path)['discoeffs'])
    calibration_save_path=os.path.join(root_path,'calibration_ax_zb.json')
    calibration_save=load_json(calibration_save_path)

    Rbase2cam=np.array(calibration_save['Rbase2cam'])
    tbase2cam=np.array(calibration_save['tbase2cam'])
    Tbase2cam=np.vstack((np.hstack((Rbase2cam,tbase2cam[:,np.newaxis])),np.array([0,0,0,1])))
    # Configure depth and color streams
    realsense_cam=create_camera('oak',1920,1080)
    camera_matrix,coeff=realsense_cam.get_intrinsics()
    color_image=realsense_cam.get_color_image()
    height,width,_=color_image.shape
    
    ################ 初始化夹爪矩阵 ################
    Rgri2end=np.array([[1,0,0],[0,1,0],[0,0,1]])
    Rgri2end=R.from_euler('xyz',[179.835327-180,-0.229587,-179.882217+180],degrees=True).as_matrix()
    tgri2end=np.array([-0.040872+0.042,0.009489-0.00965,0.162968])
    Tgri2end_init=np.vstack((np.hstack((Rgri2end,tgri2end[:,np.newaxis])),np.array([0,0,0,1])))

    ################ 机器人初始化 ##################
    aubo_robot=robot.vpRobotAuboRobots("192.168.123.96",False)
    aubo_robot.setPosition(robot.vpControlFrameType.JOINT_STATE,robot.vpColVector([np.deg2rad(-90),0,np.deg2rad(90),np.deg2rad(0),np.deg2rad(90),0]))
    aubo_robot.rtInit("192.168.123.96")

    lower_limits = np.array([-0.01, -0.01, -0.01, -0.01, -0.01, -0.01])
    upper_limits = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    start = time.time()
    last=time.time()
    first=True
    init_error=np.array([0,0,0,0,0,0])
    realsense_cam.start_recording("test")
    while True:
        # 记录时间用于启动
        now = time.time()
        duration = now - start
        cost=now-last
        
        last=now
        
        color_image = realsense_cam.get_color_image()
        save_image=color_image.copy()
        rvec,tvec=detect_aruco(color_image,camera_matrix)
        realsense_cam.set_record_frame(color_image)
        if rvec is not None:
            Rtar2cam,_=cv2.Rodrigues(rvec)
            ttar2cam=np.array(tvec)
            Ttar2cam=np.vstack((np.hstack((Rtar2cam,ttar2cam[:,np.newaxis])),np.array([0,0,0,1])))
            Toffset2tar=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-0.3],[0,0,0,1]])
            Toffsettar2cam=Ttar2cam@Toffset2tar

            realtime_pose=np.array(aubo_robot.getPositionP(robot.vpControlFrameType.END_EFFECTOR_FRAME))
            Tend2base=np.vstack((np.hstack((R.from_euler('xyz',realtime_pose[3:]).as_matrix(),realtime_pose[0:3][:,np.newaxis])),np.array([0,0,0,1])))
            Tend2cam=Tbase2cam@Tend2base
            
            error=pbvs_straight(Tend2cam,Toffsettar2cam)

            if first:
                init_error=error.copy()
                first=False
                continue

            cur_vel=0.1*error-0.1*init_error*np.exp(-2*duration)
            print(f'cost:{cost}  cur_vel:{cur_vel}')
            clipped_vel = np.maximum(np.minimum(cur_vel, upper_limits), lower_limits)
            aubo_robot.setVelocity(robot.vpControlFrameType.END_EFFECTOR_FRAME, robot.vpColVector(clipped_vel))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
    realsense_cam.release()