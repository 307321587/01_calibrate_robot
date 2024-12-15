import numpy as np
import sys
import time
from scipy.spatial.transform import Rotation as R
from _01_pbvs import pbvs_straight
sys.path.append('./build/modules/pybind')
import robot
aubo_robot=robot.vpRobotAuboRobots("192.168.123.96",False)
aubo_robot.setPosition(robot.vpControlFrameType.JOINT_STATE,robot.vpColVector([np.deg2rad(-90),0,np.deg2rad(90),np.deg2rad(0),np.deg2rad(90),0]))
aubo_robot.rtInit("192.168.123.96")



acc_sum_count = 40
target_vel = np.array([-0.01, 0, 0, 0.01, 0, 0])
max_acc = target_vel / acc_sum_count
cur_vel = np.zeros(6)
count = 0
count_sum = 200

stop = False


# 模拟机器人设置速度的函数
time.sleep(0.1)
position=np.array(aubo_robot.getPositionP(robot.vpControlFrameType.END_EFFECTOR_FRAME))
Tcur2base=np.vstack((np.hstack((R.from_euler('xyz',position[3:]).as_matrix(),position[0:3][:,np.newaxis])),np.array([0,0,0,1])))

transform_r=R.from_euler('xyz',[np.pi/6,0,0]).as_matrix()
transform_t=np.array([0.01,0.01,0.01])
transform_mat=np.vstack((np.hstack((transform_r,transform_t[:,np.newaxis])),np.array([0,0,0,1])))
Ttar2base=Tcur2base@transform_mat

print(R.from_euler('xyz',[np.pi/6,0,0]).as_rotvec())
error=pbvs_straight(Tcur2base,Ttar2base)
init_error=error.copy()
print(error)

lower_limits = np.array([-0.01, -0.01, -0.01, -0.01, -0.01, -0.01])
upper_limits = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])

start = time.time()
# 输入不为q
while np.linalg.norm(error)>1e-4:
    now = time.time()
    duration = now - start
    # if duration>1:
    #     break

    position=np.array(aubo_robot.getPositionP(robot.vpControlFrameType.END_EFFECTOR_FRAME))
    Tcur2base=np.vstack((np.hstack((R.from_euler('xyz',position[3:]).as_matrix(),position[0:3][:,np.newaxis])),np.array([0,0,0,1])))
    error=pbvs_straight(Tcur2base,Ttar2base)

    cur_vel=0.1*error-0.1*init_error*np.exp(-2*duration)
    print(f'cur_vel:{cur_vel}')
    clipped_vel = np.maximum(np.minimum(cur_vel, upper_limits), lower_limits)
    aubo_robot.setVelocity(robot.vpControlFrameType.END_EFFECTOR_FRAME, robot.vpColVector(clipped_vel))
    print(f"clipped_vel:{clipped_vel}")
    time.sleep(0.01)  # 休眠 5000 微秒
# test=robot.vpMatrix([[0,0,0]])
# # print(test.numpy())
# robot.print_vpMatrix(test)
# print(test.numpy())