from camera import create_camera
import cv2
import sys
sys.path.append('./build/modules/pybind')
import robot
import numpy as np
from dataset_collect.sample import shell_section,random_walk
from scipy.spatial.transform import Rotation as R
from dataset_collect.inout import save_json,save_im
import os
import time


def generate_trajectory():
    '''
    首先设定边界和步长,线性生成xyz点集,然后把xyz点集的顺序变成除草机式,然后对于每个点xyz增加几组设定的rxryrz
    :return: new_points
    '''

    # ================设定空间轨迹参数================
    x_stride, y_stride, z_stride = 0.3, 0.2, 0.1
    x_min, x_max, y_min, y_max, z_min, z_max = -0.5, 0.2, -0.65, -0.4, 0.4, 0.6

    # ================初始化点集================
    points = []
    
    # ================xyz方向的个数================
    nx = (x_max - x_min) // x_stride + 1
    ny = (y_max - y_min) // y_stride + 1
    nz = (z_max - z_min) // z_stride + 1
    
    # ================xyz================
    x = np.linspace(x_min, x_max, int(nx))
    y = np.linspace(y_min, y_max, int(ny))
    z = np.linspace(z_min, z_max, int(nz))
    
    # ================生成三维网格坐标================
    xx, yy, zz = np.meshgrid(x, y, z)
   
    # ================将三个坐标数组转换为一维数组================
    xx = xx.reshape(-1)
    yy = yy.reshape(-1)
    zz = zz.reshape(-1)
    
    # ================将三个一维数组合并成一个二维数组================
    points = np.column_stack((xx, yy, zz))
    
    eulers = np.array([[-180,0,90],
            [-176.01,29.85,97.42],
            [-178.21,-9.01,96.51],
            [178.52,20.30,66.18],
            [179.24,22.97,104.22],
            [151.32,0.95,90.87],
            [-156.92,1.14,90.3]])/180*np.pi

    # ================将每个点增加n组姿态,组成点(x,y,z,rx,ry,rz)================
    new_points = []
    for point in points:
        for euler in eulers:
            new_points.append(point.tolist() + euler.tolist())
    new_points = np.array(new_points)
    return new_points


def compute_end_matrix(end_pos, end_tar, cam_up_vector):
    y_vector=np.array(end_tar-end_pos)/np.linalg.norm(end_tar-end_pos)
    x_vector=np.cross(cam_up_vector,y_vector)/np.linalg.norm(np.cross(cam_up_vector,y_vector))
    z_vector=np.cross(x_vector,y_vector)/np.linalg.norm(np.cross(x_vector,y_vector))

    Tend2world=np.array([x_vector,y_vector,z_vector,np.array(end_pos)]).transpose()
    Tend2world=np.concatenate([Tend2world,[[0,0,0,1]]],axis=0)

    return Tend2world

if __name__=="__main__":

    points=generate_trajectory()

    time_str=time.strftime('%Y%m%d%H%M', time.localtime())
    save_path='record/effector_real_'+time_str
    os.makedirs(save_path,exist_ok=True)

    robot.init()
    robot.movej(np.deg2rad([-90,0,90,0,90,0]))
    current_end_pose=robot.get_status()
    end_pos=current_end_pose[0:3]
    end_ori=current_end_pose[3:6]
    up_vector=np.array([0,0,1])
    distance2end=0.7
    camera_pos=end_pos-np.array([0,distance2end,0])

    target_distance=0.2
    target_pos=end_pos-np.array([0,target_distance,0])
    # end_poses=shell_section(target_pos,target_distance,target_distance,1,-30,30,10,45,145,50) 
    # 1280,720 1.88 0.0096 640 480 2.54 0.0095
    # end_poses=shell_section(target_pos,target_distance,target_distance,1,-45,30,10,40,120,50) 3.8 0.01
    # end_poses=shell_section(target_pos,target_distance,target_distance,1,-60,30,10,40,120,50)
    end_poses=shell_section(camera_pos,distance2end,distance2end,1,-15,10,15,60,110,25)
    realsense_cam=create_camera('oak',1920,1080)

    length=end_poses.shape[0]
    end_rot_angle=np.random.rand(length,1)*np.pi #夹爪随机角度

    camera_parm={'camera_matrix':realsense_cam.get_intrinsics()[0].tolist(),'discoeffs':realsense_cam.get_intrinsics()[1].tolist()}
    save_json(os.path.join(save_path,"camera.json"),camera_parm)



    gts=[]
    for index,action in enumerate(points):
        # robot.movel_random_effctor(action,end_rot_angle[index])
        robot.movel_random_effctor(action,-np.pi/2)
        # robot.movel(action)

        color_image=realsense_cam.get_color_image()
        pose=robot.get_status()
        
        Rend2base=R.from_euler('xyz',pose[3:]).as_matrix().tolist()
        tend2base=np.array(pose[0:3]).tolist()

        gt={"index":index,"R_e2b":Rend2base,"t_e2b":tend2base}
        gts.append(gt)
        img_path=os.path.join(save_path,f'{index:06d}.png')
        save_im(img_path,color_image[:,:,::-1])
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)
    
    save_json(os.path.join(save_path,"record.json"),gts)