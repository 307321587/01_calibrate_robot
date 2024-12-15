import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import glob
import os
from dataset_collect.inout import load_json,save_json

square_size = 0.02  # 标定板每个格子的尺寸（假设单位为米)
reproj_threhold=0.5

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


def get_RT_from_chessboard(undistort_img,camera_matrix):
    '''
    :param img_path: 读取图片路径
    :param chess_board_x_num: 棋盘格x方向格子数
    :param chess_board_y_num: 棋盘格y方向格子数
    :param K: 相机内参
    :param chess_board_len: 单位棋盘格长度,mm
    :return: 相机外参
    '''
    gray = cv2.cvtColor(undistort_img, cv2.COLOR_BGR2GRAY)
    # size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(undistort_img, (11, 8), flags=cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_FAST_CHECK|cv2.CALIB_CB_NORMALIZE_IMAGE)  

    

    if ret:  # 如果成功找到了角点  
        corners_subpix=cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))

        obj_points = np.zeros((11*8,3),dtype=np.float64)
        obj_points[:,:2] = np.mgrid[0:11,0:8].T.reshape(-1,2) * square_size
        # print(object_points)
        # 通过角点坐标和标定板的实际尺寸来计算标定板的位姿  
        _,rvecs, tvecs,_ = cv2.solvePnPRansac(obj_points, corners_subpix, camera_matrix, np.array([0, 0, 0, 0, 0], dtype=np.float64)) 
        RT=np.column_stack(((cv2.Rodrigues(rvecs))[0],tvecs))
        RT = np.row_stack((RT, np.array([0, 0, 0, 1]))) 
        # 计算重投影误差
        repoj_corners_subpix, _ = cv2.projectPoints(obj_points, rvecs, tvecs, camera_matrix, np.array([0, 0, 0, 0, 0], dtype=np.float64))
        repoj_error=np.mean(np.linalg.norm(corners_subpix-repoj_corners_subpix,axis=1))
        if repoj_error>reproj_threhold:
            return None,None
        cv2.drawFrameAxes(undistort_img, camera_matrix, np.zeros(5), rvecs, tvecs, 0.1)
        # cv2.imshow('axis',img)
        # cv2.waitKey()
        # print(f'reprojet error:{repoj_error}')
        rvecs=np.squeeze(rvecs)
        tvecs=np.squeeze(tvecs)
        return rvecs,tvecs
    else:
        return None,None
 
def cal_mean_diff(Rbase2gri_s,tbase2gri_s,Rcam2tar_s,tcam2tar_s,Rbase2cam,tbase2cam,Rgri2tar,tgri2tar,img_paths,camera_matrix):
    Tgri2tar=np.vstack((np.hstack((Rgri2tar,tgri2tar)),np.array([0,0,0,1])))
    Tbase2cam=np.vstack((np.hstack((Rbase2cam,tbase2cam)),np.array([0,0,0,1])))
    error=[]
    point=np.array([0,0,0,1])
    # 重投影计算误差
    for Rbase2gri,tbase2gri,Rcam2tar,tcam2tar,img_path in zip(Rbase2gri_s,tbase2gri_s,Rcam2tar_s,tcam2tar_s,img_paths):
        Tbase2gri=np.vstack((np.hstack((Rbase2gri,tbase2gri[:,np.newaxis])),np.array([0,0,0,1])))

        Tcam2tar=np.vstack((np.hstack((Rcam2tar,tcam2tar[:,np.newaxis])),np.array([0,0,0,1])))
        Tbase2gri_cal=np.linalg.inv(Tgri2tar)@Tcam2tar@Tbase2cam
        
        diff=np.linalg.inv(Tbase2gri_cal)@Tbase2gri
        diff_rads,_=cv2.Rodrigues(diff[0:3,0:3])
        diff_angle=np.linalg.norm(diff_rads*180/np.pi)

        t_diff_norm=np.linalg.norm(diff[0:3,3])

        error.append(np.array([diff_angle,t_diff_norm]))
        # 展示末端投影情况
        img=cv2.imread(img_path)
        Tgri2cam=Tbase2cam@np.linalg.inv(Tbase2gri)
        point_proj=camera_matrix@Tgri2cam[0:3,:]@point
        point_proj=point_proj[0:2]/point_proj[2]
        cv2.circle(img,(int(point_proj[0]),int(point_proj[1])),4,(0,255,0))
        cv2.imshow('reproj',img)
        cv2.waitKey()

    # 平均位姿
    error_avg=np.mean(np.array(error),axis=0)
    

    return error_avg


if __name__=="__main__":
    # 标定不能使用4：3分辨率，尤其是640 480 会标定失败 effector_real_202410242209 effector_real_202410261700
    root_path='record/effector_real_202411241638'
    camera_path=os.path.join(root_path,'camera.json')
    camera_matrix=np.array(load_json(camera_path)['camera_matrix'])
    dist_coeffs=np.array(load_json(camera_path)['discoeffs'])
    img_paths=glob.glob(os.path.join(root_path,'*.png'))
    img_paths=sorted(img_paths)
    calibration_save_path=os.path.join(root_path,'calibration_ax_zb.json')
    calibration_save={}

    Rcam2tar_s=[]
    tcam2tar_s=[]
    delete_index=[]
    deleted_img_paths=[]
    for index,img_path in enumerate(img_paths):
        img=cv2.imread(img_path)
        undistort_img=cv2.undistort(img,camera_matrix,dist_coeffs)
        rvec,tvec=get_RT_from_chessboard(undistort_img,camera_matrix)
        if rvec is not None:
            Rtar2cam,_=cv2.Rodrigues(rvec)
            ttar2cam=np.array(tvec)
            Rcam2tar=Rtar2cam.T
            tcam2tar=-Rcam2tar@ttar2cam
            # print(f'{index:06d}:\n{Rtar2cam}')
            Rcam2tar_s.append(Rcam2tar)
            tcam2tar_s.append(tcam2tar)
            deleted_img_paths.append(img_path)
        else:
            delete_index.append(index)


    datas=load_json(os.path.join(root_path,'record.json'))

    Rbase2gri_s=[]
    tbase2gri_s=[]
    
    
    for index,data in enumerate(datas):
        if index in delete_index:
            continue
        robot_rot=np.array(data['R_e2b'])
        robot_t=np.array(data['t_e2b'])
        # Tgri2base=np.vstack((np.hstack((robot_rot,robot_t[:,np.newaxis])),np.array([0,0,0,1])))
        # print(R.from_matrix(robot_rot).as_euler('xyz',degrees=True))
        # Tbase2gri=np.linalg.inv(Tgri2base)
        Rbase2gri=robot_rot.T
        tbase2gri=-Rbase2gri@robot_t

        Rbase2gri_s.append(Rbase2gri)
        tbase2gri_s.append(tbase2gri)

    # Rcam2base,tcam2base=cv2.calibrateHandEye(Rbase2gri_s,tbase2gri_s,Rtar2cam_s,ttar2cam_s,cv2.CALIB_HAND_EYE_TSAI)
    Rbase2cam,tbase2cam,Rgri2tar,tgri2tar=cv2.calibrateRobotWorldHandEye(Rcam2tar_s,tcam2tar_s,Rbase2gri_s,tbase2gri_s)
    print(f'基座到相机旋转：\n{Rbase2cam}')
    print(f'基座到相机平移：\n{tbase2cam}')
    print(f'末端到标定板旋转：\n{Rgri2tar}')
    print(f'末端到标定板平移：\n{tgri2tar}')
    calibration_save['Rbase2cam']=np.squeeze(Rbase2cam).tolist()
    calibration_save['tbase2cam']=np.squeeze(tbase2cam).tolist()

    error_avg=cal_mean_diff(Rbase2gri_s,tbase2gri_s,Rcam2tar_s,tcam2tar_s,Rbase2cam,tbase2cam,Rgri2tar,tgri2tar,deleted_img_paths,camera_matrix)
    if error_avg[0]<0.5 and error_avg[1]<0.005:
        print("标定成功")
        print(f'平均角度误差(度):{error_avg[0]}')
        print(f'平均平移误差(米):{error_avg[1]}')
    else:
        print("标定失败，重新采集数据集")
        print(f'平均角度误差(度):{error_avg[0]}')
        print(f'平均平移误差(米):{error_avg[1]}')
    

    save_json(calibration_save_path,calibration_save)