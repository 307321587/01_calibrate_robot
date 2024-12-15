import numpy as np
import sys
import time
from scipy.spatial.transform import Rotation as R
sys.path.append('./build/modules/pybind')
import robot
aubo_robot=robot.vpRobotAuboRobots("192.168.123.96",False)
# aubo_robot.setPosition(robot.vpControlFrameType.JOINT_STATE,robot.vpColVector([np.deg2rad(90),0,np.deg2rad(-90),0,np.deg2rad(-90),0]))
aubo_robot.rtInit("192.168.123.96")



acc_sum_count = 40
target_vel = np.array([-0.01, 0, 0, 0.01, 0, 0])
max_acc = target_vel / acc_sum_count
cur_vel = np.zeros(6)
count = 0
count_sum = 200

stop = False
last = time.time()

# 模拟机器人设置速度的函数
time.sleep(0.01)
position=np.array(aubo_robot.getPositionP(robot.vpControlFrameType.END_EFFECTOR_FRAME))
Tcur2base=np.vstack((np.hstack((R.from_euler('xyz',position[3:]).as_matrix(),position[0:3][:,np.newaxis])),np.array([0,0,0,1])))

transform_r=R.from_euler('xyz',[np.pi/6,0,0])
transform_t=np.array([0.01,0.01,0.01])
transform_mat=np.vstack((np.hstack((transform_r,transform_t[:,np.newaxis])),np.array([0,0,0,1])))

Ttar2base=Tcur2base@transform_mat

# 输入不为q
while not stop:
    now = time.time()
    duration = now - last
    cost = duration * 1e6  # 转换为微秒
    # print(f"cost: {cost} us")
    last = now

    if count == count_sum + acc_sum_count - 1:
        stop = True

    if count % count_sum == 0:
        target_vel = -target_vel
        max_acc = -max_acc

    if np.linalg.norm(target_vel - cur_vel) > 1e-6:
        cur_vel += max_acc

    count += 1
    # print(f"cur: {cur_vel}")
    # aubo_robot.setVelocity(robot.vpControlFrameType.END_EFFECTOR_FRAME, robot.vpColVector(cur_vel))
    position=np.array(aubo_robot.getPositionP(robot.vpControlFrameType.END_EFFECTOR_FRAME))
    cur_mat=np.vstack((np.hstack((R.from_euler('xyz',position[3:]).as_matrix(),position[0:3][:,np.newaxis])),np.array([0,0,0,1])))
    print(cur_mat)
    time.sleep(0.005)  # 休眠 5000 微秒
# test=robot.vpMatrix([[0,0,0]])
# # print(test.numpy())
# robot.print_vpMatrix(test)
# print(test.numpy())