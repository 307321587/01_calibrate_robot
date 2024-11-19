from camera import create_camera
import cv2
import sys
sys.path.append('./build/modules/pybind')
import robot
import numpy as np
from dataset_collect.sample import shell_section,random_walk
from scipy.spatial.transform import Rotation as R
from dataset_collect.inout import save_json,save_im,load_json
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

    copy_path='record/effector_real_202411072045'
    datas=load_json(os.path.join(copy_path,'record.json'))

    robot.init()

    realsense_cam=create_camera('oak',1920,1080)

    camera_parm={'camera_matrix':realsense_cam.get_intrinsics()[0].tolist(),'discoeffs':realsense_cam.get_intrinsics()[1].tolist()}
    save_json(os.path.join(save_path,"camera.json"),camera_parm)

    gts=[]
    for index,data in enumerate(datas):
        
        tend2base=np.array(data['t_e2b'])
        Rend2base=np.array(data['R_e2b'])
        euler=R.from_matrix(Rend2base).as_euler('xyz')
        end_pose=tend2base
        action=np.concatenate((end_pose,euler))
        # robot.movel_random_effctor(action,end_rot_angle[index])
        # robot.movel_random_effctor(action,-np.pi/2)
        robot.movel(action)
        time.sleep(5)
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
        cv2.waitKey(30)
    
    save_json(os.path.join(save_path,"record.json"),gts)