import pyrealsense2 as rs
import numpy as np
class RealsenseCamera:
    def __init__(self,width=1280,height=720):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)
        self.color_profile = profile.get_stream(rs.stream.color)
        

    def get_intrinsics(self):
        intr = self.color_profile.as_video_stream_profile().get_intrinsics()
        camera_matrix=np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        coeffs=intr.coeffs
        return camera_matrix,coeffs

    def get_color_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        return color_image