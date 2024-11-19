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
    time_str=time.strftime('TCP%Y%m%d%H%M', time.localtime())
    save_path='record/effector_real_'+time_str
    os.makedirs(save_path,exist_ok=True)

    robot.init()
    num=0
    gts=[]
    while True:
        key=input("Press Enter to save pose")
        # 如果key为空格
        if key==' ':
            pose=robot.get_status()
            Rend2base=R.from_euler('xyz',pose[3:]).as_matrix().tolist()
            tend2base=np.array(pose[0:3]).tolist()

            gt={"index":num,"R_e2b":Rend2base,"t_e2b":tend2base}
            gts.append(gt)
            num=num+1
            print(f'保存位姿{num}')
        # Press esc or 'q' to close the image window
        if key=='q':
            break
    rot_diff=np.zeros((0,3))
    trans_diff=np.zeros(0)

    if len(gts)>=4:
        for i in range(len(gts)-1):
            rot_diff=np.append(rot_diff,np.array(gts[i]['R_e2b'])-np.array(gts[i+1]['R_e2b']),axis=0)
            trans_diff=np.append(trans_diff,np.array(gts[i+1]['t_e2b'])-np.array(gts[i]['t_e2b']))
        X = np.linalg.lstsq(np.array(rot_diff), np.array(trans_diff), rcond=-1)
            
    
    print(X)
    save_json(os.path.join(save_path,"record.json"),gts)