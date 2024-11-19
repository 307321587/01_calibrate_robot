import os
from opengl_utils.glutils import *
from opengl_utils.shader_utils import Shader
from opengl_utils.model import ModelProcess
import glfw
import sys

class FrameBuffer:
    def __init__(self, width, height, cbo_type, cbo_format,color_format=GL_RED, color_type=GL_UNSIGNED_BYTE):
        """
        初始化framebuffer
        :param width: 图像宽度
        :param height: 图像高度
        :param cbo_type: 判断颜色附件的类型:0为texture buffer,1为render buffer
        """

        self.scr_width_ = width
        self.scr_height_ = height
        self.cbo_type_ = cbo_type
        self.cbo_format_=cbo_format
        self.color_format_ = color_format
        self.color_type_ = color_type

        # 位置缓冲帧设置
        self.fbo_ = glGenFramebuffers(1)
        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo_)

        # 渲染深度缓冲,不使用该缓冲会显示模型内部
        self.dbo_ = glGenRenderbuffers(1)
        glBindRenderbuffer(GL_RENDERBUFFER, self.dbo_)
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, self.scr_width_, self.scr_width_)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, self.dbo_)
        glBindRenderbuffer(GL_RENDERBUFFER, 0)

        # cbo_type_用于最后显示的部分,最后显示的为1,之前的纹理为0
        # 0:render buffer, 忽略纹理
        # 1:texture buffer,特别针对纹理
        # 如果需要一个纯净的offscreen渲染,推荐render buffer
        if (not self.cbo_type_):
            self.cbo_ = glGenRenderbuffers(1)
            glBindRenderbuffer(GL_RENDERBUFFER, self.cbo_)

            glRenderbufferStorage(GL_RENDERBUFFER, self.cbo_format_, self.scr_width_, self.scr_width_)

            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, self.cbo_)
            glBindRenderbuffer(GL_RENDERBUFFER, 0)

        else:

            # 创建纹理图像,适用于最终不需要显示的buffer
            self.cbo_ = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, self.cbo_)

            glTexImage2D(GL_TEXTURE_2D, 0, self.cbo_format_, self.scr_width_, self.scr_height_, 0, GL_RGB, GL_FLOAT, None)

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT)
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, self.cbo_, 0)

        glBindTexture(GL_TEXTURE_2D, 0)  # 解绑纹理缓冲
        glBindFramebuffer(GL_FRAMEBUFFER, 0)  # 解绑帧缓冲

    def ClearBuffer(self):
        """
        清空Buffer,使用清屏函数
        :return:
        """
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    def BindTexture(self):
        """
        绑定该framebuffer的texture buffer
        :return:
        """
        glBindTexture(GL_TEXTURE_2D, self.cbo_)

    def Bind(self):
        """
        绑定该framebuffer
        :return:
        """

        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo_)

    def ReadBuffer(self):
        """
        读取该buffer的内容
        :return: 返回buffer数组
        """
        if (not self.cbo_type_):
            self.Bind()
            glReadBuffer(GL_COLOR_ATTACHMENT0)
            array = glReadPixels(0, 0, self.scr_width_, self.scr_height_, self.color_format_, self.color_type_)
        else:
            glBindTexture(GL_TEXTURE_2D, self.cbo_)
            array = glGetTexImage(GL_TEXTURE_2D, 0, self.color_format_, self.color_type_)

        # 使用frombuffer读取数据

        # if(self.color_type_==GL_UNSIGNED_BYTE):
        #     nparray = np.frombuffer(array, np.uint8).reshape(self.scr_height_, self.scr_width_, -1)
        #     if (nparray.shape[2] == 1):
        #         nparray = np.squeeze(nparray)
        # elif(self.color_type_==GL_FLOAT):
        #     nparray = np.frombuffer(array, np.float).reshape(self.scr_height_, self.scr_width_, -1)
        #     if (nparray.shape[2] == 1):
        #         nparray = np.squeeze(nparray)
        # else:
        # 读入的数据为1280*720,重排为720*1280
        # 输入类型为GL_FLOAT等时,会直接返回数组
        nparray = array.reshape(self.scr_height_, self.scr_width_, -1)
        if (nparray.shape[2] == 1):
            nparray = np.squeeze(nparray)

        return nparray

    def SetFullViewPort(self):

        glViewport(0,0,self.scr_width_,self.scr_height_)

class OneStageRender:
    def __init__(self, type: str, width, height, camera_matrix,transformation_matrix,near,far,texture_flag,opencv_flag=True,m2mm=True):
        """

        :param type: 渲染器类型(RGB/Depth/Position/Normal/Mask)
        :param width: 图像宽度
        :param height: 图像高度
        :param camera_matrix: 相机内参矩阵
        :param transformation_matrix: 相机外参矩阵
        :param near: 近点
        :param far: 原点
        :param texture_flag: 该渲染是否为texture,是为1,renderbuffer为0
        :param opencv_flag: True为opencv 转换矩阵 False为 opengl转换矩阵
        """

        # 根据渲染的不同类型进行设置

        # RGB需要纹理图像,但目前ModelProcess暂时无法读入纹理图像
        if (type == "RGB"):
            self.cbo_format_ = GL_RGB
            self.color_format_ = GL_BGR
            self.color_type_ = GL_UNSIGNED_INT
            self.shader = Shader([os.path.join('shaders', "RGB_vertex.vs")],
                                 [os.path.join('shaders', "RGB_fragment.fs")])
        # 深度图像
        elif (type == "Depth"):
            self.cbo_format_ = GL_R32F
            self.color_format_ = GL_RED
            self.color_type_ = GL_FLOAT
            self.shader = Shader([os.path.join('shaders', "Depth_vertex.vs")],
                                 [os.path.join('shaders', "Depth_fragment.fs")])
        # 位置图像,用于Egde渲染
        elif (type == "Position"):
            self.cbo_format_ = GL_RGB32F
            self.color_format_ = GL_RGB
            self.color_type_ = GL_FLOAT
            self.shader = Shader([os.path.join('shaders', "position_shader.vs")],
                                 [os.path.join('shaders', "position_shader.fs")])
        # 法线图像,用于Edge渲染
        elif (type == "Normal"):
            self.cbo_format_ = GL_RGB32F
            self.color_format_ = GL_RGB
            self.color_type_ = GL_FLOAT
            self.shader = Shader([os.path.join('shaders', "Normal_vertex.vs")],
                                 [os.path.join('shaders', "Normal_fragment.fs")])
        # 掩膜图像
        elif (type == "Mask"):
            self.cbo_format_ = GL_RGB
            self.color_format_ = GL_RED
            self.color_type_ = GL_UNSIGNED_INT
            self.shader = Shader([os.path.join('shaders', "Mask_vertex.vs")],
                                 [os.path.join('shaders', "Mask_fragment.fs")])
        elif(type=="Ori_Position"):
            self.cbo_format_ = GL_RGB32F
            self.color_format_ = GL_RGB
            self.color_type_ = GL_FLOAT
            self.shader = Shader([os.path.join('shaders', "position_ori_shader.vs")],
                                 [os.path.join('shaders', "position_ori_shader.fs")])
        else:
            raise Exception("Invalid Render Type")

        self.scr_width_ = width
        self.scr_height_ = height

        self.projection_ = set_intrinsics_from_K_matrix(camera_matrix, self.scr_width_, self.scr_height_, near, far)

        if(opencv_flag):
            self.view_ = change_transformation_matrix_opencv2opengl(transformation_matrix)
        else:
            self.view_ = transformation_matrix

        if m2mm:
            self.model_matrix_=np.eye(4) * 1000
        else:
            self.model_matrix_ = np.eye(4)
        self.model_matrix_[3, 3] = 1

        self.output_buffer=FrameBuffer(self.scr_width_, self.scr_height_, texture_flag, self.cbo_format_, self.color_format_, self.color_type_)

    def draw(self,model:ModelProcess):
        """
        单阶段渲染
        :param model: 输入ModelProcess类型模型
        :return:
        """

        self.output_buffer.Bind()
        self.output_buffer.ClearBuffer()
        self.output_buffer.SetFullViewPort()

        self.shader.use()
        self.shader.setMat4("model", self.model_matrix_)
        self.shader.setMat4("projection", self.projection_)
        self.shader.setMat4("view", self.view_)


        model.draw(self.shader)

    def ReadFrame(self,flip_flag:bool):

        array=self.output_buffer.ReadBuffer()
        if(flip_flag):
            array=np.flip(array,axis=0)
        return array

    def BindTexture(self,id:int):

        glActiveTexture(GL_TEXTURE0+id)
        self.output_buffer.BindTexture()

    def SetTransformation(self,transformation_matrix:np.ndarray,opencv_flag=True):
        """

        :param transformation_matrix: 转换矩阵
        :param opencv_flag: True为opencv 转换矩阵 False为 opengl转换矩阵
        :return:
        """
        if (opencv_flag):
            self.view_ = change_transformation_matrix_opencv2opengl(transformation_matrix)
        else:
            self.view_ = transformation_matrix

class EdgeRender:
    def __init__(self,type:str,width, height, camera_matrix,transformation_matrix,near,far,strong_thresh,weak_thresh,distance_thresh,texture_flag=False,opencv_flag=True,m2mm=True):

        self.scr_width_=width
        self.scr_height_=height

        self.normal_redner=OneStageRender("Normal",self.scr_width_,self.scr_height_,camera_matrix,transformation_matrix,near,far,texture_flag=True,opencv_flag=opencv_flag,m2mm=m2mm)
        self.postion_render = OneStageRender("Position", self.scr_width_,self.scr_height_,camera_matrix,transformation_matrix,near,far,texture_flag=True,opencv_flag=opencv_flag,m2mm=m2mm)

        # 边缘图像
        if(type=="Edge"):
            self.cbo_format_ = GL_RGB
            self.color_format_ = GL_RED
            self.color_type_ = GL_UNSIGNED_INT
            self.shader = Shader([os.path.join('shaders', "Edge_vertex.vs")],
                                 [os.path.join('shaders', "Edge_fragment.fs")])
        # 抑制边缘法向图像
        elif(type=="AntiEdgeNormal"):
            self.cbo_format_ = GL_RGB32F
            self.color_format_ = GL_RGB
            self.color_type_ = GL_FLOAT
            self.shader = Shader([os.path.join('shaders', "AntiEdgeNormal_vertex.vs")],
                                 [os.path.join('shaders', "AntiEdgeNormal_fragment.fs")])
        else:
            raise Exception("Invalid Render Type")

        # 设置边缘参数
        self.shader.use()
        self.shader.setInt("normal_map", 1)
        self.shader.setInt("position_map", 0)
        self.shader.setFloat("delta_x", 1.0 / self.scr_width_)
        self.shader.setFloat("delta_y", 1.0 / self.scr_height_)
        self.shader.setFloat("strong_threshold", 2.0 * math.sin(math.radians(strong_thresh) / 2.0))
        self.shader.setFloat("weak_threshold", 2.0 * math.sin(math.radians(weak_thresh) / 2.0))
        self.shader.setFloat("distance_threshold", distance_thresh)

        self.ndc_render=NDCScreen()

        self.output_buffer=FrameBuffer(self.scr_width_, self.scr_height_, texture_flag, self.cbo_format_, self.color_format_,self.color_type_)

    def draw(self,model:ModelProcess):

        self.postion_render.draw(model)
        self.normal_redner.draw(model)

        self.postion_render.BindTexture(0)
        self.normal_redner.BindTexture(1)

        self.shader.use()
        self.output_buffer.Bind()
        self.output_buffer.ClearBuffer()
        self.output_buffer.SetFullViewPort()

        self.ndc_render.draw()

    def ReadFrame(self,flip_flag:bool):

        array=self.output_buffer.ReadBuffer()
        if(flip_flag):
            array=np.flip(array,axis=0)
        return array

    def SetTransformation(self,transformation_matrix:np.ndarray,opencv_flag=True):
        """

        :param transformation_matrix: 转换矩阵
        :param opencv_flag: True为opencv 转换矩阵 False为 opengl转换矩阵
        :return:
        """
        self.normal_redner.SetTransformation(transformation_matrix,opencv_flag)
        self.postion_render.SetTransformation(transformation_matrix, opencv_flag)

class NDCScreen:

    def __init__(self):

        # 设置屏幕NDC坐标
        quad_vertices = np.array([
            [-1.0, 1.0, 0.0, 1.0],
            [-1.0, -1.0, 0.0, 0.0],
            [1.0, -1.0, 1.0, 0.0],

            [-1.0, 1.0, 0.0, 1.0],
            [1.0, -1.0, 1.0, 0.0],
            [1.0, 1.0, 1.0, 1.0]
        ], np.float32).flatten()

        self.scr_vao = glGenVertexArrays(1)
        self.scr_vbo = glGenBuffers(1)

        # 绑定屏幕缓冲
        glBindVertexArray(self.scr_vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.scr_vbo)
        glBufferData(GL_ARRAY_BUFFER, len(quad_vertices) * 4, quad_vertices, GL_STATIC_DRAW)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 16, ctypes.c_voidp(0))
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 16, ctypes.c_voidp(8))

        glBindVertexArray(0)
        glBindBuffer(GL_ARRAY_BUFFER,0)

    def draw(self):
        glBindVertexArray(self.scr_vao)

        glDrawArrays(GL_TRIANGLES, 0, 6)

class PoseProcess:
    def __init__(self,model_path,scr_width,scr_height,camera_matrix,near,far,strong_thresh,weak_thresh,distance_thresh,m2mm=True,opencv_flag=True,windows_name="draw Cube "):
        self.scr_width_=scr_width
        self.scr_height_=scr_height
        self.camera_matrix_=camera_matrix
        self.near_=near
        self.far_=far
        self.strong_thresh_=strong_thresh
        self.weak_thresh_=weak_thresh
        self.distance_thresh_=distance_thresh


        if not glfw.init():
            sys.exit()

        # 使窗口不显示
        glfw.window_hint(glfw.VISIBLE, False)
        # Create a windowed mode window and its OpenGL context
        window = glfw.create_window(self.scr_width_, self.scr_height_, windows_name, None, None)
        if not window:
            glfw.terminate()
            sys.exit()

        # Make the window's context current
        glfw.make_context_current(window)
        self.model = ModelProcess(model_path)
        self.bounding_box=self.model.bounding_box_
        transformation=np.identity(4)
        self.edge_render = EdgeRender("Edge", self.scr_width_, self.scr_height_, self.camera_matrix_, transformation, self.near_, self.far_,
                                 self.strong_thresh_, self.weak_thresh_, self.distance_thresh_,m2mm=m2mm,opencv_flag=opencv_flag)
        self.mask_render=OneStageRender("Mask",self.scr_width_,self.scr_height_,self.camera_matrix_,transformation,self.near_,self.far_,texture_flag=True,m2mm=m2mm,opencv_flag=opencv_flag)
        self.normal_render =  OneStageRender("Normal", self.scr_width_, self.scr_height_, self.camera_matrix_, transformation, self.near_, self.far_,
                                 texture_flag=True,m2mm=m2mm,opencv_flag=opencv_flag)
        self.position_render = OneStageRender("Ori_Position", self.scr_width_, self.scr_height_, self.camera_matrix_,
                                            transformation, self.near_, self.far_,
                                            texture_flag=True, m2mm=m2mm, opencv_flag=opencv_flag)
        self.depth_render = OneStageRender("Depth", self.scr_width_, self.scr_height_, self.camera_matrix_,
                                      transformation,
                                      self.near_, self.far_, texture_flag=True, opencv_flag=opencv_flag, m2mm=m2mm)
    def PoseShow(self,transformation):
        self.edge_render.SetTransformation(transformation)
        self.edge_render.draw(self.model)
        img_edge = self.edge_render.ReadFrame(flip_flag=True)
        img_edge[img_edge>255]=255
        img_edge=img_edge.astype(np.uint8)

        return img_edge

    def MaskShow(self,transformation):
        self.mask_render.SetTransformation(transformation)
        self.mask_render.draw(self.model)
        img_mask = self.mask_render.ReadFrame(flip_flag=True)
        img_mask[img_mask>255]=255
        img_mask=img_mask.astype(np.uint8)

        return img_mask

    def NormalShow(self,transformation):
        self.normal_render.SetTransformation(transformation)
        self.normal_render.draw(self.model)
        img_normal = self.normal_render.ReadFrame(flip_flag=True)

        return img_normal

    def PositionShow(self,transformation):
        self.position_render.SetTransformation(transformation)
        self.position_render.draw(self.model)
        img_position = self.position_render.ReadFrame(flip_flag=True)

        return img_position

    def DepthShow(self,transformation):
        self.depth_render.SetTransformation(transformation)
        self.depth_render.draw(self.model)
        img_depth = self.depth_render.ReadFrame(flip_flag=True)
        return img_depth