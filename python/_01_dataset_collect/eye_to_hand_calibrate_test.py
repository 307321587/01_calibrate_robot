import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import glob
import os
from dataset_collect.inout import load_json
from dataset_collect.sample import shell_section,random_walk
import sys
sys.path.append('./build/modules/pybind')
import robot
 
def euler2rot(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    rotation_matrix = r.as_matrix()
    return rotation_matrix
 
def detect_aruco(img,camera_matrix):
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(arucoDict)
    (corners, ids, rejected) = detector.detectMarkers(img)
    if ids is not None:
             
        # cv2.aruco.drawDetectedMarkers(img, corners, ids)
        rvecs, tvecs,_=cv2.aruco.estimatePoseSingleMarkers(corners, 0.06, camera_matrix, np.zeros(5))
        for rvec,tvec in zip(rvecs,tvecs):
            cv2.drawFrameAxes(img, camera_matrix, np.zeros(5), rvec, tvec, 0.1)
            # cv2.imshow('axis',img)
            # cv2.waitKey()
            # print(f"rotation: {rvec} translation:{tvec}") 
        return rvecs[0],tvecs[0]
    return None,None

def compute_end_matrix(end_pos, end_tar, cam_up_vector):
    y_vector=np.array(end_tar-end_pos)/np.linalg.norm(end_tar-end_pos)
    x_vector=np.cross(cam_up_vector,y_vector)/np.linalg.norm(np.cross(cam_up_vector,y_vector))
    z_vector=np.cross(x_vector,y_vector)/np.linalg.norm(np.cross(x_vector,y_vector))

    Tend2world=np.array([x_vector,y_vector,z_vector,np.array(end_pos)]).transpose()
    Tend2world=np.concatenate([Tend2world,[[0,0,0,1]]],axis=0)

    return Tend2world 

if __name__=="__main__":

    root_path='record/effector_real_202410220901'
    camera_path=os.path.join(root_path,'camera.json')
    camera_matrix=np.array(load_json(camera_path)['camera_matrix'])
    img_paths=glob.glob(os.path.join(root_path,'*.png'))
    img_paths=sorted(img_paths)
    Rtar2cam_s=[]
    ttar2cam_s=[]

    for img_path in img_paths:
        img=cv2.imread(img_path)
        rvec,tvec=detect_aruco(img,camera_matrix)
        Rtar2cam,_=cv2.Rodrigues(rvec)
        ttar2cam=np.array(tvec)
        Rtar2cam_s.append(Rtar2cam)
        ttar2cam_s.append(ttar2cam)


    datas=load_json(os.path.join(root_path,'record.json'))

    Rbase2gri_s=[]
    tbase2gri_s=[]
    
    robot.init()
    robot.movej(np.deg2rad([-90,0,90,0,90,0]))
    current_end_pose=robot.get_status()
    end_pos=current_end_pose[0:3]
    end_ori=current_end_pose[3:6]
    up_vector=np.array([0,0,1])
    distance2end=0.7
    camera_pos=end_pos-np.array([0,distance2end,0])

    end_poses=shell_section(camera_pos,distance2end,distance2end,1,-15,10,10,60,110,50)
    for index,end_pose in enumerate(end_poses):
        if index%40!=0:
            continue

        Tend2world=compute_end_matrix(end_pose,camera_pos,up_vector)
        print(Tend2world)
        Tbase2gri=np.linalg.inv(Tend2world)

        Rbase2gri_s.append(Tbase2gri[0:3,0:3])
        tbase2gri_s.append(Tbase2gri[0:3,3])

    Rcam2base,tcam2base=cv2.calibrateHandEye(Rbase2gri_s,tbase2gri_s,Rtar2cam_s,ttar2cam_s,cv2.CALIB_HAND_EYE_TSAI)
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

