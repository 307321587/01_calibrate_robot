#!/usr/bin/env python3
from camera import OAKCamera
import cv2
import time

camera=OAKCamera(1920,1080)
camera_matrix,discoeff=camera.get_intrinsics()
# img=camera.get_color_image()
# codec = cv2.VideoWriter_fourcc(*'XVID')
# video_file_name = 'test.avi'
# # output_video = cv2.VideoWriter(video_file_name, codec, 30, (1920, 1080))
# camera.start_recording("test")
# last=time.time()
# while True:
#     now=time.time()
#     duration=now-last
#     print(f"cost:{duration}")
#     last=now
#     img=camera.get_color_image()
#     # img=cv2.undistort(img,camera_matrix,discoeff)
#     # output_video.write(img)
#     camera.set_record_frame(img)
#     cv2.imshow("img",img)
#     key=cv2.waitKey(1)
#     if key & 0xFF == ord('q'):
#         break
# camera.release()
# output_video.release()
