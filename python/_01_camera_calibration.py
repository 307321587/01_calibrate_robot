import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import glob
import os
from dataset_collect.inout import load_json,save_json

square_size = 0.02  # 标定板每个格子的尺寸（假设单位为米)
reproj_threhold=0.5
dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)  # 畸变系数  

def euler2rot(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    rotation_matrix = r.as_matrix()
    return rotation_matrix
 
def detect_aruco(img,camera_matrix):
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(arucoDict)
    (corners, ids, rejected) = detector.detectMarkers(img)
    marker_length=0.06
    corners_3d=np.array([[-marker_length/2,marker_length/2,0],[marker_length/2,marker_length/2,0],[marker_length/2,-marker_length/2,0],[-marker_length/2,-marker_length/2,0]])

    if ids is not None:
             
        # cv2.aruco.drawDetectedMarkers(img, corners, ids)
        # rvecs, tvecs,_=cv2.aruco.estimatePoseSingleMarkers(corners, 0.06, camera_matrix, np.zeros(5))
        flag,rvecs,tvecs=cv2.solvePnP(corners_3d,corners[0],camera_matrix,np.zeros(5))
        rvecs=np.squeeze(rvecs)
        tvecs=np.squeeze(tvecs)
        cv2.drawFrameAxes(img, camera_matrix, np.zeros(5), rvecs, tvecs, 0.1)
        cv2.imshow('axis',img)
        cv2.waitKey()
            # print(f"rotation: {rvec} translation:{tvec}") 
        return rvecs,tvecs
    return None,None


def get_points(img):
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
    ret, corners = cv2.findChessboardCorners(img, (11, 8), flags=cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_FAST_CHECK|cv2.CALIB_CB_NORMALIZE_IMAGE)  
    obj_points = np.zeros((11*8,3))
    obj_points[:,:2] = np.mgrid[0:11,0:8].T.reshape(-1,2) * square_size
    
    if ret:  # 如果成功找到了角点  
        corners_subpix=cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        return ret,corners_subpix,obj_points
    else:
        return False,None,None
 
if __name__=="__main__":
    # 标定不能使用4：3分辨率，尤其是640 480 会标定失败
    root_path='record/effector_real_202410242308'
    camera_path=os.path.join(root_path,'camera.json')
    camera_matrix=np.array(load_json(camera_path)['camera_matrix'])
    img_paths=glob.glob(os.path.join(root_path,'*.png'))
    img_paths=sorted(img_paths)
    calibration_save_path=os.path.join(root_path,'calibration.json')
    calibration_save={}

    
    # Arrays to store object points and image points from all the images.
    objpoints_s = [] # 3d point in real world space
    imgpoints_s = [] # 2d points in image plane.
    for index,img_path in enumerate(img_paths):
        img=cv2.imread(img_path)
        ret,corners_subpix,obj_points=get_points(img)
        if ret:
            objpoints_s.append(obj_points.astype(np.float32))
            imgpoints_s.append(corners_subpix.astype(np.float32))
        # cv2.drawChessboardCorners(img, (11, 8), corners_subpix, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(np.array(objpoints_s), np.array(imgpoints_s), img.shape[0:2][::-1], None, None)
    print(f'camera_matrix:{mtx}')