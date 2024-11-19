import cv2
import numpy as np
import glob
from math import *
import os

# K=np.array([[1534.5, 0, 957.0639],  
#             [0, 1534.3, 538.1748],  
#             [0, 0, 1]],dtype=np.float64)#大恒相机内参
# chess_board_x_num=12#棋盘格x方向格子数
# chess_board_y_num=9#棋盘格y方向格子数
# chess_board_len=20#单位棋盘格长度,mm


# #用于根据欧拉角计算旋转矩阵
# def myRPY2R_robot(x, y, z):
#     Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
#     Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
#     Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
#     R = Rz@Ry@Rx
#     return R

# #用于根据位姿计算变换矩阵
# def pose_robot(x, y, z, Tx, Ty, Tz):
#     thetaX = x / 180 * pi
#     thetaY = y / 180 * pi
#     thetaZ = z / 180 * pi
#     R = myRPY2R_robot(thetaX, thetaY, thetaZ)
#     t = np.array([[Tx], [Ty], [Tz]])
#     RT1 = np.column_stack([R, t])  # 列合并
#     RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
#     # RT1=np.linalg.inv(RT1)
#     return RT1

# #用来从棋盘格图片得到相机外参
# def get_RT_from_chessboard(img_path,chess_board_x_num,chess_board_y_num,K,chess_board_len):
#     '''
#     :param img_path: 读取图片路径
#     :param chess_board_x_num: 棋盘格x方向格子数
#     :param chess_board_y_num: 棋盘格y方向格子数
#     :param K: 相机内参
#     :param chess_board_len: 单位棋盘格长度,mm
#     :return: 相机外参
#     '''
#     img=cv2.imread(img_path)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     size = gray.shape[::-1]
#     ret, corners = cv2.findChessboardCorners(gray, (chess_board_x_num, chess_board_y_num), None)
#     # print(corners)
#     corner_points=np.zeros((2,corners.shape[0]),dtype=np.float64)
#     for i in range(corners.shape[0]):
#         corner_points[:,i]=corners[i,0,:]
#     # print(corner_points)
#     object_points=np.zeros((3,chess_board_x_num*chess_board_y_num),dtype=np.float64)
#     flag=0
#     for i in range(chess_board_y_num):
#         for j in range(chess_board_x_num):
#             object_points[:2,flag]=np.array([(11-j-1)*chess_board_len,(8-i-1)*chess_board_len])
#             flag+=1
#     # print(object_points)

#     retval,rvec,tvec  = cv2.solvePnP(object_points.T,corner_points.T, K, distCoeffs=None)
#     # print(rvec.reshape((1,3)))
#     # RT=np.column_stack((rvec,tvec))
#     RT=np.column_stack(((cv2.Rodrigues(rvec))[0],tvec))
#     RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
#     # RT=pose(rvec[0,0],rvec[1,0],rvec[2,0],tvec[0,0],tvec[1,0],tvec[2,0])
#     # print(RT)

#     # print(retval, rvec, tvec)
#     # print(RT)
#     # print('')
#     return RT

# folder = r"calib"#棋盘格图片存放文件夹
# # files = os.listdir(folder)
# # file_num=len(files)
# # RT_all=np.zeros((4,4,file_num))

# # print(get_RT_from_chessboard('calib/2.bmp', chess_board_x_num, chess_board_y_num, K, chess_board_len))
# # '''
# # 这个地方很奇怪的特点，有些棋盘格点检测得出来，有些检测不了，可以通过函数get_RT_from_chessboard的运行时间来判断
# # '''
# good_picture=[1,3,5,6,7,8,10,11,12,13,14,15]#存放可以检测出棋盘格角点的图片
# # good_picture=[1,3,10,11,12]
# file_num=len(good_picture)

# #计算board to cam 变换矩阵
# R_all_chess_to_cam_1=[]
# T_all_chess_to_cam_1=[]
# for i in good_picture:
#     # print(i)
#     image_path=folder+'/'+str(i)+'.bmp'
#     RT=get_RT_from_chessboard(image_path, chess_board_x_num, chess_board_y_num, K, chess_board_len)

#     # RT=np.linalg.inv(RT)

#     R_all_chess_to_cam_1.append(RT[:3,:3])
#     T_all_chess_to_cam_1.append(RT[:3, 3].reshape((3,1)))
# # print(T_all_chess_to_cam.shape)

# #计算end to base变换矩阵
# file_address='calib/机器人基坐标位姿.xlsx'#从记录文件读取机器人六个位姿
# sheet_1 = pd.read_excel(file_address)
# R_all_end_to_base_1=[]
# T_all_end_to_base_1=[]
# # print(sheet_1.iloc[0]['ax'])
# for i in good_picture:
#     # print(sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dx'],
#     #                                   sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dz'])
#     RT=pose_robot(sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dx'],
#                                       sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dz'])
#     # RT=np.column_stack(((cv2.Rodrigues(np.array([[sheet_1.iloc[i-1]['ax']],[sheet_1.iloc[i-1]['ay']],[sheet_1.iloc[i-1]['az']]])))[0],
#     #                    np.array([[sheet_1.iloc[i-1]['dx']],
#     #                                   [sheet_1.iloc[i-1]['dy']],[sheet_1.iloc[i-1]['dz']]])))
#     # RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
#     # RT = np.linalg.inv(RT)

#     R_all_end_to_base_1.append(RT[:3, :3])
#     T_all_end_to_base_1.append(RT[:3, 3].reshape((3, 1)))

# # print(R_all_end_to_base_1)
# R,T=cv2.calibrateHandEye(R_all_end_to_base_1,T_all_end_to_base_1,R_all_chess_to_cam_1,T_all_chess_to_cam_1)#手眼标定
# RT=np.column_stack((R,T))
# RT = np.row_stack((RT, np.array([0, 0, 0, 1])))#即为cam to end变换矩阵
# print('相机相对于末端的变换矩阵为：')
# print(RT)

# #结果验证，原则上来说，每次结果相差较小
# for i in range(len(good_picture)):

#     RT_end_to_base=np.column_stack((R_all_end_to_base_1[i],T_all_end_to_base_1[i]))
#     RT_end_to_base=np.row_stack((RT_end_to_base,np.array([0,0,0,1])))
#     # print(RT_end_to_base)

#     RT_chess_to_cam=np.column_stack((R_all_chess_to_cam_1[i],T_all_chess_to_cam_1[i]))
#     RT_chess_to_cam=np.row_stack((RT_chess_to_cam,np.array([0,0,0,1])))
#     # print(RT_chess_to_cam)

#     RT_cam_to_end=np.column_stack((R,T))
#     RT_cam_to_end=np.row_stack((RT_cam_to_end,np.array([0,0,0,1])))
#     # print(RT_cam_to_end)

#     RT_chess_to_base=RT_end_to_base@RT_cam_to_end@RT_chess_to_cam#即为固定的棋盘格相对于机器人基坐标系位姿
#     RT_chess_to_base=np.linalg.inv(RT_chess_to_base)
#     print('第',i,'次')
#     print(RT_chess_to_base[:3,:])
#     print('')
# ————————————————

#                             版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
                        
# 原文链接：https://blog.csdn.net/qq_43607792/article/details/121257352




# import numpy as np
# import cv2
  
# # 步骤 1: 收集数据  
# # 1.1 机器人末端执行器位姿列表  
# pose_list = []  # 存储机器人末端执行器的位姿  
  
# # 假设有一个机器人末端执行器的运动学模型robot_kinematics，可以通过该模型获取机器人末端执行器的位姿  
# for i in range(num_poses):  # 假设有num_poses个位姿  
#     pose = robot_kinematics.get_pose(i)  # 获取第i个位姿  
#     pose_list.append(pose)  # 将位姿添加到列表中  
  
# # 1.2 相机视觉检测到的标定板位姿列表  
# pattern_pose_list = []  # 存储相机视觉检测到的标定板位姿  
  

# 读取相机标定参数  
# 这里假设您已经完成了相机的标定，并且已经获得了相机的内参矩阵camera_matrix和畸变系数dist_coeffs  
camera_matrix = np.array([[1534.5, 0, 957.0639],  
             [0, 1534.3, 538.1748],  
             [0, 0, 1]],dtype=np.float64)  # 相机内参矩阵  
dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float64)  # 畸变系数  
  
# 读取标定板的实际尺寸  
# 这里假设您已经知道了标定板的实际尺寸，例如棋盘格的每个格子的尺寸square_size  
square_size = 20  # 标定板每个格子的尺寸（假设单位为米）  
  
# 读取标定板图像  
# 假设您已经拍摄了一张包含标定板的图像，并且已经将该图像读取为OpenCV格式的图像对象image  
img_path ='D:/study/02code/mpdepth/dataset/20240130/rgb/0000.png'

# 查找标定板的角点  
# 这里假设您使用了棋盘格作为标定板，使用cv2.findChessboardCorners函数来查找棋盘格的角点  

# if ret:  # 如果成功找到了角点  
#     # 通过角点坐标和标定板的实际尺寸来计算标定板的位姿  
#     ret, rvecs, tvecs = cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)  
#     cv2.drawChessboardCorners(image, (11, 8), corners, ret)
#     cv2.imshow('image', image)
#     cv2.waitKey(0)
    # rvecs为旋转向量，tvecs为平移向量，它们分别表示了标定板相对于相机的位姿  
    # 在这里，您可以将rvecs和tvecs添加到pattern_pose_list中，以保存检测到的标定板位姿  
# else:  
#     print("未找到标定板角点")  

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


    # ret, corners = cv2.findChessboardCorners(gray, (11, 8), None)
    # print(corners)
    # corner_points=np.zeros((2,corners.shape[0]),dtype=np.float64)
    # for i in range(corners.shape[0]):
    #     corner_points[:,i]=corners[i,0,:]
    # print(corner_points)
    # object_points=np.zeros((3,11*8),dtype=np.float64)
    obj_points = np.zeros((11*8,3),dtype=np.float64)
    obj_points[:,:2] = np.mgrid[0:11,0:8].T.reshape(-1,2) * square_size
    # print(object_points)

    if ret:  # 如果成功找到了角点  
        # 通过角点坐标和标定板的实际尺寸来计算标定板的位姿  
        _,rvecs, tvecs,_ = cv2.solvePnPRansac(obj_points, corners, camera_matrix, dist_coeffs) 
        RT=np.column_stack(((cv2.Rodrigues(rvecs))[0],tvecs))
        RT = np.row_stack((RT, np.array([0, 0, 0, 1]))) 
        # cv2.drawChessboardCorners(image, (11, 8), corners, ret)
        # cv2.imshow('image', image)
        # cv2.waitKey(0)
        print(RT)
        return RT
    else:
        return None

    # print(rvec.reshape((1,3)))
    # RT=np.column_stack((rvec,tvec))


    
    # print(retval, rvec, tvec)
    # print(RT)
    # print('')
    # return RT
    

def myRPY2R_robot(z1, y, z2):
    Rz1 = np.array([[cos(z1), -sin(z1), 0], [sin(z1), cos(z1), 0], [0, 0, 1]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz2 = np.array([[cos(z2), -sin(z2), 0], [sin(z2), cos(z2), 0], [0, 0, 1]])
    R = Rz1@Ry@Rz2
    return R

def pose_robot(Tx, Ty, Tz, z1, y, z2):

    thetaX = z1 / 180 * pi
    thetaY = y / 180 * pi
    thetaZ = z2 / 180 * pi
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[Tx], [Ty], [Tz]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
    # RT1=np.linalg.inv(RT1)
    return RT1

pose_robot(418.756,334.608,407.816,80.6349,-142.413,-48.2772)

R_all_chess_to_cam_1=[]
T_all_chess_to_cam_1=[]

R_all_end_to_base_1=[]
T_all_end_to_base_1=[]

#第一次拍摄15张,rgb
# robot_pose=np.array([[-30.65, -659.55, 244.69,108.18,-169.20,-84.74],
#                     [-35.91,-637.20,297.31,117.48,-165.57,-73.43],
#                     [119.63,-587.46,370.23,55.31,-159.20,-127.59],
#                     [115.06,-594.40,322.15,46.76,-161.69,-136.54],
#                     [180.70,-621.53,299.02,37.38,-157.24,-152.37],
#                     [37.29,-520.15,365.03,88.13,-156.21,-94.97],
#                     [-45.07,-683.92,384.79,99.67,-169.97,-74.21],
#                     [-251.60,-636.99,356.79,147.91,-155.27,-27.69],  
#                     [-317.51,-557.01,312.48,146.68,-146.47,-23.76],
#                     [-0.82,-500.25,320.98,91.09,-150.94,-90.22],
#                     [136.78,-684.62,322.66,14.07,-164.81,-168.08],
#                     [296.16,-636.34,307.46,26.68,-155.61,-156.72],
#                     [179.78,-594.88,383.79,49.68,-161.69,-134.65],
#                     [164.85,-524.70,374.02,57.31,-156.35,-127.39],                                     
#                     [-205.15,-432.16,330.00,114.16,-145.56,-57.41]])


#第二次拍摄9张,rgb2
# robot_pose=np.array([[13.80, -579.35, 416.05,95.08,-164.26,-85.84],
#                     [131.45,-590.68,351.06,65.67,-159.00,-118.13],
#                     [93.12,-624.21,339.29,68.11,-164.34,-115.95],
#                     [100.54,-620.94,386.81,63.26,-169.23,-120.73],
#                     [4.22,-679.10,356.06,92.19,-173.11,-89.23],
#                     [7.62,-574.28,320.60,92.35,-166.73,-89.11],
#                     [-178.03,-542.59,300.12,135.96,-151.84,-41.27],
#                     [-177.23,-592.91,327.44,141.36,-159.58,-39.47],
#                     [-175.48,-643.15,321.41,153.55,-160.54,-23.30]])

#第三次拍摄20张,rgb3
robot_pose=np.array([[-32.28,-662.71,342.53,119.83,-172.83,-63.27],
                    [-58.94,-695.05,266.24,102.12,-174.38,-79.19],
                    [-4.98,-696.05,265.66,107.50,-174.62,-78.24],
                    [47.59,-689.97,270.88,111.88,-174.63,-78.21],
                    [93.52,-687.79,279.43,109.55,-172.14,-84.32],
                    [25.37,-653.19,289.98,112.99,-175.35,-75.26],
                    [-12.91,-677.23,261.80,103.37,-172.13,-81.35],
                    [1.22,-672.28,279.12,108.61,-170.99,-77.03],  
                    [-65.42,-668.76,279.10,101.72,-170.88,-78.27],
                    [-60.71,-689.67,289.43,100.94,-170.02,-79.82],
                    [-54.73,-664.36,334.98,98.66,-168.25,-82.46],
                    [17.72,-658.37,337.20,105.81,-168.79,-81.52],
                    [63.22,-650.04,337.92,110.75,-169.47,-80.59],
                    [58.55,-592.80,364.58,112.69,-170.84,-78.60],
                    [-26.21,-592.39,375.06,100.68,-168.51,-82.41],
                    [-74.35,-588.62,359.63,98.04,-168.36,-80.11],
                    [37.28,-585.84,375.28,107.46,-166.88,-81.53],
                    [30.07,-636.99,379.13,103.23,-163.06,-85.08],
                    [-39.67,-636.95,368.87,98.51,-164.47,-83.47],
                    [-87.13,-632.86,358.81,94.31,-164.31,-83.27],])




for index in range(6,18):
    # if index ==0 :
    #     continue
    RT=get_RT_from_chessboard(os.path.join('D:/study/02code/mpdepth/dataset/20240130/rgb3','{:04d}.png'.format(index)))
    if RT is None:
        continue

    R_all_chess_to_cam_1.append(RT[:3,:3])
    T_all_chess_to_cam_1.append(RT[:3, 3].reshape((3,1)))

    RT = pose_robot(robot_pose[index,0],robot_pose[index,1],robot_pose[index,2],robot_pose[index,3],robot_pose[index,4],robot_pose[index,5])


    R_all_end_to_base_1.append(RT[:3, :3])
    T_all_end_to_base_1.append(RT[:3, 3].reshape((3, 1)))

R,T=cv2.calibrateHandEye(R_all_end_to_base_1,T_all_end_to_base_1,R_all_chess_to_cam_1,T_all_chess_to_cam_1)#手眼标定

print("手眼矩阵分解得到的旋转矩阵")
print(R)
print("\n")


print("手眼矩阵分解得到的平移矩阵")
print(T)

RT=np.column_stack((R,T))
RT = np.row_stack((RT, np.array([0, 0, 0, 1])))#即为cam to end变换矩阵
print("\n")
print('相机相对于末端的变换矩阵为：')
print(RT)

# #结果验证，原则上来说，每次结果相差较小
for index,_ in enumerate(R_all_chess_to_cam_1):
    b2c=np.column_stack((R_all_chess_to_cam_1[index],T_all_chess_to_cam_1[index]))
    rt_b2c=np.row_stack((b2c,np.array([0,0,0,1])))
    
    e2b=np.column_stack((R_all_end_to_base_1[index],T_all_end_to_base_1[index]))
    rt_e2b=np.row_stack((e2b,np.array([0,0,0,1])))

    board2base=rt_e2b@RT@rt_b2c

    print('index:{}'.format(index))
    print(board2base)


# # 问题
# # 1. 手眼标定需要先获得标定板的位姿，标定板的位姿是否需要多张图片进行
# # 2. 机器人末端执行器的位姿列表如何输入
# # 3. 
# #现在需要首先获取机器人末端执行器的位姿列表和标定板的位姿列表