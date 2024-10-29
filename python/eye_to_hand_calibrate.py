import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import glob
import os
from dataset_collect.inout import load_json
reproj_threhold=0.5 

def euler2rot(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    rotation_matrix = r.as_matrix()
    return rotation_matrix
 
def detect_aruco(img,camera_matrix):
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
    arucoParams = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(arucoDict)
    (corners, ids, rejected) = detector.detectMarkers(img)
    marker_length=0.1
    corners_3d=np.array([[-marker_length/2,marker_length/2,0],[marker_length/2,marker_length/2,0],[marker_length/2,-marker_length/2,0],[-marker_length/2,-marker_length/2,0]])

    if ids is not None:
        corners_detect=np.squeeze(corners[0])
        flag,rvecs,tvecs=cv2.solvePnP(corners_3d,corners_detect,camera_matrix,np.zeros(5))
        
        repoj_corners_subpix, _ = cv2.projectPoints(corners_3d, rvecs, tvecs, camera_matrix, np.zeros(5))
        repoj_corners_subpix=np.squeeze(repoj_corners_subpix)
        repoj_error=np.mean(np.linalg.norm(corners_detect-repoj_corners_subpix,axis=1))
        if repoj_error>reproj_threhold:
            return None,None
        rvecs=np.squeeze(rvecs)
        tvecs=np.squeeze(tvecs)
        # cv2.drawFrameAxes(img, camera_matrix, np.zeros(5), rvecs, tvecs, 0.1)
        # cv2.imshow('axis',img)
        # cv2.waitKey()
            # print(f"rotation: {rvec} translation:{tvec}") 
        return rvecs,tvecs
    return None,None

def cal_mean_diff(Rbase2gri_s,tbase2gri_s,Rtar2cam_s,ttar2cam_s,Rcam2base,tcam2base):
    Tgri2tar_s=[]
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
        Tgri2tar_s.append(gri2tar)
        print(gri2tar)

    # 平均位姿
    Ttar2base_avg=np.mean(np.array(Tgri2tar_s),axis=0)
    Rtar2base_avg=Ttar2base_avg[0:3,0:3]
    ttar2base_avg=Ttar2base_avg[0:3,3]

    mean_angle=[]
    mean_trans=[]
    # 计算与平均位姿之间的差值
    for Tgri2tar in Tgri2tar_s:
        R_diff=Rtar2base_avg.transpose()@Tgri2tar[0:3,0:3]
        t_diff=ttar2base_avg-Tgri2tar[0:3,3]

        R_diff_rads,_=cv2.Rodrigues(R_diff)
        R_diff_angle=np.linalg.norm(R_diff_rads*180/np.pi)

        t_diff_norm=np.linalg.norm(t_diff)

        mean_angle.append(R_diff_angle)
        mean_trans.append(t_diff_norm)
    
    mean_angle=np.mean(mean_angle)
    mean_trans=np.mean(mean_trans)
    return mean_angle,mean_trans,Rtar2base_avg,ttar2base_avg,Tgri2tar_s
 

if __name__=="__main__":

    root_path='record/effector_real_202410261006'
    camera_path=os.path.join(root_path,'camera.json')
    camera_matrix=np.array(load_json(camera_path)['camera_matrix'])
    img_paths=glob.glob(os.path.join(root_path,'*.png'))
    img_paths=sorted(img_paths)
    Rtar2cam_s=[]
    ttar2cam_s=[]
    delete_index=[]
    for index,img_path in enumerate(img_paths):
        img=cv2.imread(img_path)
        rvec,tvec=detect_aruco(img,camera_matrix)
        if rvec is not None:
            Rtar2cam,_=cv2.Rodrigues(rvec)
            ttar2cam=np.array(tvec)
            Rtar2cam_s.append(Rtar2cam)
            ttar2cam_s.append(ttar2cam)
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
        Tgri2base=np.vstack((np.hstack((robot_rot,robot_t[:,np.newaxis])),np.array([0,0,0,1])))
        print(R.from_matrix(robot_rot).as_euler('xyz',degrees=True))
        Tbase2gri=np.linalg.inv(Tgri2base)

        Rbase2gri_s.append(Tbase2gri[0:3,0:3])
        tbase2gri_s.append(Tbase2gri[0:3,3])

    Rcam2base,tcam2base=cv2.calibrateHandEye(Rbase2gri_s,tbase2gri_s,Rtar2cam_s,ttar2cam_s,cv2.CALIB_HAND_EYE_TSAI)
    print(Rcam2base)
    print(tcam2base)
    mean_angle,mean_trans,Rtar2base_avg,ttar2base_avg,Tgri2tar_s=cal_mean_diff(Rbase2gri_s,tbase2gri_s,Rtar2cam_s,ttar2cam_s,Rcam2base,tcam2base)
    
    # 取小于平均值的位姿再进行标定
    opt_Rtar2cam_s=[]
    opt_ttar2cam_s=[]
    opt_Rbase2gri_s=[]
    opt_tbase2gri_s=[]

    for index,Tgri2tar in enumerate(Tgri2tar_s):
        R_diff=Rtar2base_avg.transpose()@Tgri2tar[0:3,0:3]
        t_diff=ttar2base_avg-Tgri2tar[0:3,3]

        R_diff_rads,_=cv2.Rodrigues(R_diff)
        R_diff_angle=np.linalg.norm(R_diff_rads*180/np.pi)

        t_diff_norm=np.linalg.norm(t_diff)

        if(R_diff_angle<mean_angle and t_diff_norm<mean_trans):
            opt_Rbase2gri_s.append(Rbase2gri_s[index])
            opt_tbase2gri_s.append(tbase2gri_s[index])
            opt_Rtar2cam_s.append(Rtar2cam_s[index])
            opt_ttar2cam_s.append(ttar2cam_s[index])
    opt_Rcam2base,opt_tcam2base=cv2.calibrateHandEye(opt_Rbase2gri_s,opt_tbase2gri_s,opt_Rtar2cam_s,opt_ttar2cam_s,cv2.CALIB_HAND_EYE_TSAI)
    print(opt_Rcam2base)
    print(opt_tcam2base)

    mean_angle,mean_trans,Rtar2base_avg,ttar2base_avg,Tgri2tar_s=cal_mean_diff(opt_Rbase2gri_s,opt_tbase2gri_s,opt_Rtar2cam_s,opt_ttar2cam_s,opt_Rcam2base,opt_tcam2base)

    if mean_angle<0.5 and mean_trans<0.005:
        print("标定成功")
        print(f'平均角度误差(度):{mean_angle}')
        print(f'平均平移误差(米):{mean_trans}')
    else:
        print("标定失败，重新采集数据集")
        print(f'平均角度误差(度):{mean_angle}')
        print(f'平均平移误差(米):{mean_trans}')