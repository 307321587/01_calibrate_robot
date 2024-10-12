import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
 
 
def euler2rot(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    rotation_matrix = r.as_matrix()
    return rotation_matrix
 
 
if __name__=="__main__":
    with open('record/record.json','r') as f:
        datas=json.load(f)

    Rbase2gri_s=[]
    tbase2gri_s=[]
    Rtar2cam_s=[]
    ttar2cam_s=[]
    
    for data in datas:
        robot_t_euler=data['robot']
        camera_axis_angle_t=data['camera']

        robot_rot=euler2rot(robot_t_euler[3:6])
        robot_t=np.array(robot_t_euler[0:3])
        Tbase2gri=np.vstack((np.hstack((robot_rot,robot_t[:,np.newaxis])),np.array([0,0,0,1])))
        Tbase2gri=np.linalg.inv(Tbase2gri)


        camera_rot,_=cv2.Rodrigues(np.array(camera_axis_angle_t[0:3]))
        camera_t=np.array(camera_axis_angle_t[3:6])

        Rbase2gri_s.append(Tbase2gri[0:3,0:3])
        tbase2gri_s.append(Tbase2gri[0:3,3])
        Rtar2cam_s.append(camera_rot)
        ttar2cam_s.append(camera_t)
    Rcam2base,tcam2base=cv2.calibrateHandEye(Rbase2gri_s,tbase2gri_s,Rtar2cam_s,ttar2cam_s,cv2.CALIB_HAND_EYE_ANDREFF)
    print(Rcam2base)
    print(tcam2base)
    # 重投影计算误差
    for Rbase2gri,tbase2gri,Rtar2cam,ttar2cam in zip(Rbase2gri_s,tbase2gri_s,Rtar2cam_s,ttar2cam_s):
        base2gri=np.eye(4)
        base2gri[0:3,0:3]=Rbase2gri
        base2gri[0:3,3]=tbase2gri
        tar2cam=np.eye(4)
        tar2cam[0:3,0:3]=Rtar2cam
        tar2cam[0:3,3]=ttar2cam
        cam2base=np.eye(4)
        cam2base[0:3,0:3]=Rcam2base
        cam2base[0:3,3]=tcam2base.squeeze()
        gri2tar=base2gri@cam2base@tar2cam
        print(gri2tar)

