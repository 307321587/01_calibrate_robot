import pyrealsense2 as rs
import numpy as np
import depthai as dai
import gxipy as gx
from time import sleep
import cv2
class RealsenseCamera:
    def __init__(self,width=1280,height=720):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        # config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)
        self.color_profile = profile.get_stream(rs.stream.color)
        

    def get_intrinsics(self):
        intr = self.color_profile.as_video_stream_profile().get_intrinsics()
        camera_matrix=np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        coeffs=intr.coeffs
        return camera_matrix,np.array(coeffs)

    def get_color_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        return color_image
    
    def stop(self):
        self.pipeline.stop()

class OAKCamera:
    def __init__(self,width=1280,height=720):
        self.width=width
        self.height=height
        pipeline = dai.Pipeline()

        # Define source and output
        camRgb = pipeline.create(dai.node.ColorCamera)
        xoutRgb = pipeline.create(dai.node.XLinkOut)

        # https://docs.oakchina.cn/projects/api/samples/Camera/camera_undistort.html 使用video节点，用于拍照
        xoutRgb.setStreamName("still") 

        # Properties
        camRgb.setPreviewSize(width, height)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Linking
        camRgb.preview.link(xoutRgb.input)

        self.device=dai.Device(pipeline)

        qRgb = self.device.getOutputQueue(name="still", maxSize=1, blocking=False)
        color_image=qRgb.get().getCvFrame()

    def get_color_image(self):
        qRgb = self.device.getOutputQueue(name="still", maxSize=4, blocking=False)
        color_image=qRgb.get()
        color_image=color_image.getCvFrame()
        return color_image
    
    def get_intrinsics(self):
        calibData = self.device.readCalibration()
        # camera_matrix = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.width, self.height))
        camera_matrix=np.array([[1543.96624464076, 0.0, 958.115323380734], [0.0, 1543.86647915387, 535.854123578914], [0.0, 0.0, 1.0]])
        print(f"RGB Camera resized intrinsics... {self.width} x {self.height}:\n {camera_matrix}")
        # coeffs=np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
        # print("Coefficients...")
        # [print(name+": "+value) for (name, value) in zip(["k1","k2","p1","p2","k3","k4","k5","k6","s1","s2","s3","s4","τx","τy"],[str(data) for data in coeffs])]
        coeffs=np.array([0.108291528595848,-0.202884682353348,0,0,0])
        return camera_matrix,coeffs
    

class DahengCamera:
    def __init__(self,width=1920,height=1080):
        '''
        param: index
        '''
        self.width=width
        self.height=height
        device_manager = gx.DeviceManager()
        self.device_manager = device_manager
        dev_num, dev_info_list = device_manager.update_device_list()
        
        if dev_num == 0:
            raise Exception("can't find camera device")

        # 打开设备
        # 获取设备基本信息列表
        self.cam = device_manager.open_device_by_index(1)
        # str_sn = dev_info_list[index].get("sn")
        # # 通过序列号打开设备
        # self.cam = device_manager.open_device_by_sn(str_sn)
        self.cam.TriggerMode.set(gx.GxSwitchEntry.OFF)

            # set exposure
        self.cam.ExposureTime.set(20000.0)

        # set gain
        self.cam.Gain.set(24.0)
        # 开始采集 
        self.cam.stream_on()
        sleep(1)
        # self.cam = cam
        pass

    def get_color_image(self):
        raw_image = self.cam.data_stream[0].get_image()
        if raw_image is None:
            print("Getting image failed.")
            return raw_image
        rgb_image = raw_image.convert("RGB")

        if rgb_image is None:
            return rgb_image

        # 从 RGB 图像数据创建 numpy 数组
        numpy_image = rgb_image.get_numpy_array()
        numpy_image = cv2.cvtColor(numpy_image, cv2.COLOR_BGR2RGB)
        
        numpy_image=cv2.resize(numpy_image, (raw_image.get_width()//3, raw_image.get_height()//3))
        # numpy_image = numpy_image[:, :, ::-1]
        return numpy_image

    def get_intrinsics(self):
        camera_matrix=np.array([[2220.81514776138, 0.0, 925.523302556001], [0.0, 2221.30320458244, 602.728783535491], [0.0, 0.0, 1.0]])
        coeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        return camera_matrix,coeffs

    def close(self):
        self.cam.stream_off()
        self.cam.close_device()
        self.device_manager.del_manager()

def create_camera(cam_type,height,width):
    if cam_type=="realsense":
        return RealsenseCamera(height,width)
    elif cam_type=="oak":
        return OAKCamera(height,width)
    elif cam_type=="daheng":
        return DahengCamera(height,width)