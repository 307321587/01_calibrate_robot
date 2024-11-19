"""
camera.py

"""


from OpenGL.GL import *
from opengl_utils.glutils import *
import numpy as np
import math

YAW=0
PITCH=0
SPEED=20
SENSITIVITY=0.5
ZOOM=45
class Camera:
    def __init__(self,position=[0,0,0],up=[0,0,1],yaw=YAW,pitch=PITCH):
        self.front_=np.array([-1.0,0.0,0.0])
        self.movementSpeed_=SPEED
        self.mouseSensitivity_=SENSITIVITY
        self.zoom_=ZOOM

        self.position_=np.array(position)
        self.world_up_=np.array(up)
        self.yaw_=yaw
        self.pitch_=pitch

        self.UpdataCameraVectors()

    def ProcessKeyboard(self,direction,delta_time):

        # Camera controls
        cameraSpeed =self.movementSpeed_ * delta_time
        # 前
        if (direction==0):
            self.position_ += cameraSpeed * self.front_
        # 后
        if (direction==1):
            self.position_ -= cameraSpeed * self.front_
        # 左
        if (direction==2):
            # normalize up vector
            self.position_ -= cameraSpeed * self.right_
        # 右
        if (direction==3):
            self.position_ += cameraSpeed * self.right_

    def ProcessMouseMovement(self,xoffset,yoffset,constrainPitch=True):

        xoffset *= self.mouseSensitivity_
        yoffset *= self.mouseSensitivity_

        self.yaw_-=xoffset
        self.pitch_+=yoffset

        if(constrainPitch):
            if(self.pitch_>89):
                self.pitch_=89
            if(self.pitch_<-89):
                self.pitch_=-89

        self.UpdataCameraVectors()

    def ProcessMouseScroll(self,yoffset):

        self.zoom_-=yoffset
        if(self.zoom_<1):
            self.zoom_=1
        if(self.zoom_>45):
            self.zoom_=45

    def UpdataCameraVectors(self):

        front = np.array([-math.cos(math.radians(self.pitch_)) * math.cos(math.radians(self.yaw_)),
                -math.cos(math.radians(self.pitch_))*math.sin(math.radians(self.yaw_)),
                math.sin(math.radians(self.pitch_))])

        norm = np.linalg.norm(front) # 求二范数
        self.front_=front/norm

        self.right_=np.cross(self.front_,self.world_up_)
        norm=np.linalg.norm(self.right_)
        self.right_=self.right_/norm

        self.up_ = np.cross(self.right_, self.front_)
        norm = np.linalg.norm(self.up_)
        self.up_ = self.up_ / norm

    def GetViewMatrix(self):

        view_matrix=lookAt(self.position_, self.position_ + self.front_, self.up_)

        return view_matrix