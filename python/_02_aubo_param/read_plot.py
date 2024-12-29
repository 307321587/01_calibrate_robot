
import numpy as np
import matplotlib.pyplot as plt

with open("/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/joints_data.txt","r") as file:
    joints_data=np.zeros((0,7))
    for line in file:
        joint_current=np.fromstring(line,sep=',')
        if joint_current.shape[0]!=7:
            continue
        joints_data=np.append(joints_data,joint_current[np.newaxis,:],axis=0)
        # print(joint_current)
    # 计算速度
    time=joints_data[:,0][:,np.newaxis]
    time=np.tile(time,(1,6))
    joints_data=joints_data[:,1:]
    joints_speed=np.diff(joints_data,axis=0)
    diff_time=np.diff(time,axis=0)
    time_speed=time[1:,:]
    # joints_speed=joints_speed/diff_time*1e6
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(time,joints_data)
    plt.legend(["j1","j2","j3","j4","j5","j6"])
    plt.subplot(2,1,2)
    plt.plot(time_speed,joints_speed)
    plt.legend(["j1","j2","j3","j4","j5","j6"])


with open("/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/record_line.offt","r") as file:
    joints_data=np.zeros((0,7))
    time=0
    for line in file:
        joint=np.fromstring(line,sep=',')
        joint_current=np.zeros(7)
        joint_current[0]=time
        joint_current[1:]=joint
        joints_data=np.append(joints_data,joint_current[np.newaxis,:],axis=0)
        time+=500
        # print(joint_current)
    # 计算速度
    time=joints_data[:,0][:,np.newaxis]
    time=np.tile(time,(1,6))
    joints_data=joints_data[:,1:]
    joints_speed=np.diff(joints_data,axis=0)
    diff_time=np.diff(time,axis=0)
    time_speed=time[1:,:]
    # joints_speed=joints_speed/diff_time*1e6
    
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(time,joints_data)
    plt.legend(["j1","j2","j3","j4","j5","j6"])
    plt.subplot(2,1,2)
    plt.plot(time_speed,joints_speed)
    plt.legend(["j1","j2","j3","j4","j5","j6"])
    plt.show()