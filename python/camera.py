import pyrealsense2 as rs
import numpy as np
import depthai as dai

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
        camera_matrix = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.width, self.height))
        print(f"RGB Camera resized intrinsics... {self.width} x {self.height}:\n {camera_matrix}")
        coeffs=np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
        print("Coefficients...")
        [print(name+": "+value) for (name, value) in zip(["k1","k2","p1","p2","k3","k4","k5","k6","s1","s2","s3","s4","τx","τy"],[str(data) for data in coeffs])]
        return camera_matrix,coeffs

def create_camera(cam_type,height,width):
    if cam_type=="realsense":
        return RealsenseCamera(height,width)
    elif cam_type=="oak":
        return OAKCamera(height,width)