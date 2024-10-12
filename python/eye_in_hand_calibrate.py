import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
square_size = 0.02  # 标定板每个格子的尺寸（假设单位为米）  
dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)  # 畸变系数  
camera_matrix=np.array([[918.27160645  ,0.,643.14483643],
 [  0.,918.02313232 ,357.28491211],
 [  0.,0.,   1.        ]])

print(cv2.__version__)

def euler2rot(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    rotation_matrix = r.as_matrix()
    return rotation_matrix

def get_RT_from_chessboard(img_path):
    '''
    :param img_path: 读取图片路径
    :param chess_board_x_num: 棋盘格x方向格子数
    :param chess_board_y_num: 棋盘格y方向格子数
    :param K: 相机内参
    :param chess_board_len: 单位棋盘格长度,mm
    :return: 相机外参
    '''
    # img=cv2.imread(image)
    image = cv2.imread(img_path)
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(image, (11, 8), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)  


    obj_points = np.zeros((11*8,3),dtype=np.float64)
    obj_points[:,:2] = np.mgrid[0:11,0:8].T.reshape(-1,2) * square_size
    # print(object_points)

    if ret:  # 如果成功找到了角点  
        # 通过角点坐标和标定板的实际尺寸来计算标定板的位姿  
        _,rvecs, tvecs,_ = cv2.solvePnPRansac(obj_points, corners, camera_matrix, dist_coeffs) 
        RT=np.column_stack(((cv2.Rodrigues(rvecs))[0],tvecs))
        RT = np.row_stack((RT, np.array([0, 0, 0, 1]))) 
        print(RT)
        return RT
    else:
        return None
 
if __name__=="__main__":


    with open('record/hand_eye2/record.json','r') as f:
        datas=json.load(f)

    Rgri2base_s=[]
    tgri2base_s=[]
    Rtar2cam_s=[]
    ttar2cam_s=[]
    
    for data in datas:
        robot_t_euler=data['robot']

        robot_rot=euler2rot(robot_t_euler[3:6])
        robot_t=np.array(robot_t_euler[0:3])
        Tgri2base=np.vstack((np.hstack((robot_rot,robot_t[:,np.newaxis])),np.array([0,0,0,1])))


        Ttar2cam=get_RT_from_chessboard(f'record/hand_eye2/{data["num"]:06d}.jpg')

        Rgri2base_s.append(Tgri2base[0:3,0:3])
        tgri2base_s.append(Tgri2base[0:3,3])
        Rtar2cam_s.append(Ttar2cam[0:3,0:3])
        ttar2cam_s.append(Ttar2cam[0:3,3])
    Rcam2gri,tcam2gri=cv2.calibrateHandEye(Rgri2base_s,tgri2base_s,Rtar2cam_s,ttar2cam_s,cv2.CALIB_HAND_EYE_ANDREFF)
    print(Rcam2gri)
    print(tcam2gri)
    # 重投影计算误差
    for Rgri2base,tgri2base,Rtar2cam,ttar2cam in zip(Rgri2base_s,tgri2base_s,Rtar2cam_s,ttar2cam_s):
        gri2base=np.eye(4)
        gri2base[0:3,0:3]=Rgri2base
        gri2base[0:3,3]=tgri2base
        tar2cam=np.eye(4)
        tar2cam[0:3,0:3]=Rtar2cam
        tar2cam[0:3,3]=ttar2cam
        cam2gri=np.eye(4)
        cam2gri[0:3,0:3]=Rcam2gri
        cam2gri[0:3,3]=tcam2gri.squeeze()
        tar2base=gri2base@cam2gri@tar2cam
        print(" ")
        print(tar2base)

