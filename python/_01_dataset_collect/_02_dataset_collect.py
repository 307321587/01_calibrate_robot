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



def compute_end_matrix(end_pos, end_tar, cam_up_vector):
    y_vector=np.array(end_tar-end_pos)/np.linalg.norm(end_tar-end_pos)
    x_vector=np.cross(cam_up_vector,y_vector)/np.linalg.norm(np.cross(cam_up_vector,y_vector))
    z_vector=np.cross(x_vector,y_vector)/np.linalg.norm(np.cross(x_vector,y_vector))

    Tend2world=np.array([x_vector,y_vector,z_vector,np.array(end_pos)]).transpose()
    Tend2world=np.concatenate([Tend2world,[[0,0,0,1]]],axis=0)

    return Tend2world

if __name__=="__main__":
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

    target_distance=0.4
    target_pos=end_pos-np.array([0,target_distance,0])

    end_poses=shell_section(camera_pos,distance2end,distance2end,1,-15,10,10,60,110,50)
    realsense_cam=create_camera('oak',1920,1080)

    length=end_poses.shape[0]
    end_rot_angle=np.random.rand(length,1)*np.pi #夹爪随机角度

    gts=[]
    for index,end_pose in enumerate(end_poses):

        Tend2world=compute_end_matrix(end_pose,target_pos,up_vector)
        # rand_R=np.array(mathutils.Matrix.Rotation(end_rot_angle[index], 3, end_rot_axis[index]))
        # euler=R.from_matrix(rand_R@Tend2world[:3,:3]).as_euler('xyz')
        euler=R.from_matrix(Tend2world[:3,:3]).as_euler('xyz')
        trans=Tend2world[:3,3]
        action=np.concatenate((trans,euler))
        robot.movel_random_effctor(action,end_rot_angle[index])
        # robot.movel_random_effctor(action,-np.pi/2)
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
    
    camera_parm={'camera_matrix':realsense_cam.get_intrinsics()[0].tolist(),'discoeffs':realsense_cam.get_intrinsics()[1].tolist()}
    save_json(os.path.join(save_path,"camera.json"),camera_parm)
    save_json(os.path.join(save_path,"record.json"),gts)