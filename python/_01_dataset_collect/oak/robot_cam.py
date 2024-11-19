
import argparse
import os.path
import depthai as dai
import numpy as np
import cv2
import time
from tqdm import trange
import shutil

import os, sys
import threading, time
import signal
from tqdm import tqdm

sys.path.append("robot/out/build/x64-Release")
import cdrflex as cd

# --------------------------------------------------------


def signal_handler(sig, frame):
    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX signal_handler")
    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX signal_handler")
    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX signal_handler")
    global g_node
    publisher = g_node.create_publisher(RobotStop, "stop", 10)

    msg = RobotStop()
    msg.stop_mode = 1

    publisher.publish(msg)
    # sys.exit(0)
    rclpy.shutdown()

def sampler_half(n):
    a=np.zeros((0,3))
    phi=(np.sqrt(5)-1)/2
    for i in range(int(n/2),int(n/5*4)):
        # i=i+1
        # z=(2*i-1)/n-1
        z=i/n # 半球
        theta=(2*np.pi*i*phi)%(np.pi/2)+np.pi/4
        x=np.sqrt(1-z**2)*np.cos(theta)
        y=np.sqrt(1-z**2)*np.sin(theta)
        a=np.append(a,np.array([[x,y,z]]),axis=0)
    return a

def location_to_point_matrix(location,poi):

    # 此处通过指向向量完成旋转采样
    front = poi - location

    norm = np.linalg.norm(front)  # 求二范数
    front_ = front / norm

    world_z = np.array([0, 0, 1])
    x = np.cross(front_, world_z)
    norm = np.linalg.norm(x)
    x_ = x/ norm

    y = np.cross(x_, front_)
    norm = np.linalg.norm(y)
    y_ = y/ norm
    R=np.array([[x_[0],y_[0],front_[0]], [x_[1],y_[1],front_[1]], [x_[2],y_[2],front_[2]]])
    alpha=np.arctan2(R[1,2],R[0,2])
    beta=np.arctan2(np.sqrt(1-R[2,2]*R[2,2]),R[2,2])
    gamma=np.arctan2(R[2,1],(-R[2,0]))
    return alpha,beta,gamma

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-r','--root',type=str,default='dataset')
    parser.add_argument('-n','--name',type=str,default='202401201609')
    parser.add_argument('-l','--laser_power',type=int,default=0)
    parser.add_argument('-b','--baseline',type=float,default=49.890)
    parser.add_argument('-t','--time',type=int,default=20)

    args = parser.parse_args()

    root_path = args.root
    scene_name = args.name
    laser_power = args.laser_power
    baseline = args.baseline
    acquisition_time = args.time

    if os.path.exists(os.path.join(root_path, scene_name, "realsense")):
        shutil.rmtree(os.path.join(root_path, scene_name, "realsense"))

    if not os.path.exists(os.path.join(root_path,scene_name,"realsense")):
        # 创建文件夹
        os.makedirs(os.path.join(root_path,scene_name,"realsense"))
        os.makedirs(os.path.join(root_path,scene_name,"realsense","depth"))
        os.makedirs(os.path.join(root_path,scene_name,"realsense","color"))
        os.makedirs(os.path.join(root_path,scene_name,"realsense","ir_left"))
        os.makedirs(os.path.join(root_path,scene_name,"realsense","ir_right"))
        # 初始化相机
    
    pipeline = dai.Pipeline()

    # Define source and output
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")

    # Properties
    camRgb.setPreviewSize(1920, 1080)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    # Linking
    camRgb.preview.link(xoutRgb.input)

    # 连接机器人
    cd.tmain(50)

    with dai.Device(pipeline) as device:
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        # movel([369.7288471949993, -446.82105231307867, 399.0, -145.59785596510403, 131.6823253933395, 180.0],  vel=100, acc=60)
        cd.movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], 75, 75)

        # 设置采样点
        n=1800
        print("n:", n,n-2*int(n/3))
        sample_points=sampler_half(n)
        for sample_point in tqdm(sample_points):
            timestamp = int(time.time() * 1000)
            # 提取后6位数字
            timestamp = str(timestamp)[-6:]
            r=600
            x, y, z = r * sample_point[0], r * sample_point[1] -950, r * sample_point[2]-50

            alpha,beta,gamma = location_to_point_matrix(np.array([x,y,z]),np.array([0,-950,-50]))
            
            xx=[x,y,z,np.rad2deg(alpha),np.rad2deg(beta),np.rad2deg(gamma)]
            # print(xx)
            cd.movel(xx,  85, 85)
            inRgb = qRgb.get()
            cv2.imwrite(os.path.join(root_path,scene_name,"realsense","color","color_"+timestamp+".png"),inRgb.getCvFrame())

    # # 写入parameter(内参矩阵+baseline(mm单位))
    # profile = cfg.get_stream(rs.stream.infrared)
    # intrinsics = profile.as_video_stream_profile().get_intrinsics()
    # with open(os.path.join(root_path,scene_name,"realsense","parameter.txt"), 'w') as file:
    #     # 写入内容到文件
    #     content = str(intrinsics.fx)+" 0 "+str(intrinsics.ppx)+" 0 "+str(intrinsics.fy)+" "+str(intrinsics.ppy)+" 0 0 1 "+str(baseline)
    #     file.write(content)

    # try:
    #     device = pipeline.get_active_profile().get_device()
    #     depth_sensor = device.query_sensors()[0]
    #     laser_range = depth_sensor.get_option_range(rs.option.laser_power)
    #     print("laser power range = ", laser_range.min, "~", laser_range.max)
    #     # 设置laser强度(0-360)
    #     depth_sensor.set_option(rs.option.laser_power, 0)
    #     laser_pwr = depth_sensor.get_option(rs.option.laser_power)
    #     print("current laser power = ", laser_power)

    #     for _ in trange(30*acquisition_time):
    #         # 等待获取帧数据
    #         frames = pipeline.wait_for_frames()

    #         # 获取当前的毫秒级时间戳
    #         timestamp = int(time.time() * 1000)
    #         # 提取后6位数字
    #         timestamp = str(timestamp)[-6:]

    #         # 获取深度图像和彩色图像
    #         depth_frame = frames.get_depth_frame()
    #         color_frame = frames.get_color_frame()
    #         ir_frame_left = frames.get_infrared_frame(1)
    #         ir_frame_right = frames.get_infrared_frame(2)


    #         # 将帧数据转换为NumPy数组
    #         depth_image = np.asanyarray(depth_frame.get_data())
    #         color_image = np.asanyarray(color_frame.get_data())
    #         ir_image_left = np.asanyarray(ir_frame_left.get_data())
    #         ir_image_right = np.asanyarray(ir_frame_right.get_data())

    #         cv2.imwrite(os.path.join(root_path,scene_name,"realsense","depth","depth_"+timestamp+".png"),depth_image)
    #         cv2.imwrite(os.path.join(root_path,scene_name,"realsense","color","color_"+timestamp+".png"),color_image)
    #         cv2.imwrite(os.path.join(root_path,scene_name,"realsense","ir_left","ir_left_"+timestamp+".png"),ir_image_left)
    #         cv2.imwrite(os.path.join(root_path,scene_name,"realsense","ir_right","ir_right_"+timestamp+".png"),ir_image_right)


    #         # # 显示深度图像和彩色图像
    #         # cv2.imshow('Depth Image', depth_image)
    #         # cv2.imshow('Color Image', color_image)
    #         # cv2.imshow('ir left Image', ir_image_left)
    #         # cv2.imshow('ir right Image', ir_image_right)
    #         #
    #         # # 按下'q'键退出循环
    #         # if cv2.waitKey(1) & 0xFF == ord('q'):
    #         #     break

    # finally:
    #     # 停止相机并关闭窗口
    #     pipeline.stop()
    #     cv2.destroyAllWindows()