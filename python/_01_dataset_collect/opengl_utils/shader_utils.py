import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GL.shaders import *

def printOpenGLError():
    err = glGetError()  # pylint: disable=E1111
    if (err != GL_NO_ERROR):
        print('GLERROR: ', gluErrorString(err))  # pylint: disable=E1101


import os


# sys.path.append(os.path.abspath(os.path.dirname(__file__)))
class Shader:

    def __init__(self, vertex_shader_paths, fragment_shader_paths):
        vertex_shader_source_list = []
        fragment_shader_source_list = []
        if (isinstance(vertex_shader_paths, list)):

            for GLSL in vertex_shader_paths:
                absDIR = os.path.abspath(os.path.join(os.path.join(os.path.dirname(__file__)), GLSL))
                f = open(absDIR, 'rb')
                vertex_shader_source_list.append(f.read())
                f.close()
            for GLSL in fragment_shader_paths:
                absDIR = os.path.abspath(os.path.join(os.path.join(os.path.dirname(__file__)), GLSL))
                f = open(absDIR, 'rb')
                fragment_shader_source_list.append(f.read())
                f.close()
            self.initShader(vertex_shader_source_list, fragment_shader_source_list)

    def initShader(self, vertex_shader_source_list, fragment_shader_source_list):

        # vertex shader
        # print('compile vertex shader...')
        # self.vs = glCreateShader(GL_VERTEX_SHADER)  # pylint: disable=E1111
        # glShaderSource(self.vs, vertex_shader_source_list)
        self.vs=compileShader(vertex_shader_source_list,GL_VERTEX_SHADER)
        if (GL_TRUE != glGetShaderiv(self.vs, GL_COMPILE_STATUS)):
            err = glGetShaderInfoLog(self.vs)
            raise Exception(err)


        # fragment shader
        # print('compile fragment shader...')
        # self.fs = glCreateShader(GL_FRAGMENT_SHADER)  # pylint: disable=E1111
        # glShaderSource(self.fs, fragment_shader_source_list)
        # glCompileShader(self.fs)
        self.fs=compileShader(fragment_shader_source_list,GL_FRAGMENT_SHADER)
        if (GL_TRUE != glGetShaderiv(self.fs, GL_COMPILE_STATUS)):
            err = glGetShaderInfoLog(self.fs)
            raise Exception(err)

        # create program
        self.ID = glCreateProgram()  # pylint: disable=E1111
        glAttachShader(self.ID, self.vs)
        glAttachShader(self.ID, self.fs)
        printOpenGLError()

        # print('link...')
        glLinkProgram(self.ID)
        if (GL_TRUE != glGetProgramiv(self.ID, GL_LINK_STATUS)):
            err = glGetShaderInfoLog(self.vs)
            raise Exception(err)
        printOpenGLError()

    def use(self):
        if glUseProgram(self.ID):
            printOpenGLError()

    def end(self):
        glUseProgram(0)

    def get_ID(self):
        return self.ID

    def setBool(self,name:str,value:bool):
        glUniform1i(glGetUniformLocation(self.ID,name),int(value))

    def setInt(self,name:str,value:int):
        glUniform1i(glGetUniformLocation(self.ID, name), value)

    def setFloat(self,name:str,value:float):
        glUniform1f(glGetUniformLocation(self.ID, name), value)

    def setMat4(self,name:str,value):
        glUniformMatrix4fv(glGetUniformLocation(self.ID, name),1,GL_FALSE,value)