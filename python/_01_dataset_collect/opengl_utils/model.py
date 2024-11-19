import pyassimp
import pyassimp.postprocess
import glfw
from opengl_utils.mesh import MeshProcess,Vertex,Vec2,Vec3
from opengl_utils import glutils
import numpy as np
from OpenGL.GL import *
from opengl_utils.shader_utils import Shader
import math
import sys

class ModelProcess:
    def __init__(self,path,gamma=False):

        # 属性设置
        self.mesh_processes_=[]
        self.textures_loaded_=[]
        self.bounding_box_ = np.zeros((2, 3))
        self.first_flag_ = True
        self.position_=[]

        # 加载模型
        self.loadModel(path)


    def loadModel(self,path):
        with pyassimp.load(path,processing=pyassimp.postprocess.aiProcess_GenSmoothNormals | pyassimp.postprocess.aiProcess_CalcTangentSpace | pyassimp.postprocess.aiProcess_Triangulate|pyassimp.postprocess.aiProcess_FlipUVs)  as scene:

            self.processNode(scene.rootnode,scene)

    def draw(self,shader):
        for MeshProcess in self.mesh_processes_:
            MeshProcess.Draw(shader)

    def processNode(self,node,scene):
        # 此处与c++不同,rootnode下即为全部的mesh
        for index, mesh in enumerate(node.meshes):
            #mesh=scene.meshes[mesh_index]
            self.mesh_processes_.append(self.processMesh(mesh, scene))
            #mesh=scene.meshes

        for child in node.children:
            self.processNode(child,scene)

    def processMesh(self,mesh,scene):

        vertices = (mesh.vertices.shape[0] * Vertex)()
        for i in range(mesh.vertices.shape[0]):
            vertices[i].Position = Vec3(*mesh.vertices[i])
            vertices[i].Normal = Vec3(*mesh.normals[i])
            vertices[i].TexCoords = Vec2(0, 0)
            vertices[i].Tangent = Vec3(0, 0, 0)
            vertices[i].Bitangent = Vec3(0, 0, 0)

            # if mesh.texturecoords[0] is not None:
            #     vertices[i].TexCoords = Vec2(*mesh.texturecoords[0][i][:2])
            #     vertices[i].Tangent = Vec3(*mesh.tangents[i])
            #     vertices[i].Bitangent = Vec3(*mesh.bitangents[i])
            # else:
            #     vertices[i].TexCoords = Vec2(0, 0)
            #     vertices[i].Tangent = Vec3(0, 0, 0)
            #     vertices[i].Bitangent = Vec3(0, 0, 0)

        idx = [i for face in mesh.faces for i in face]
        indices = (ctypes.c_uint * len(idx))(*idx)
        # self.position_.append(mesh.vertices)
        textures = []
        material = scene.materials[mesh.materialindex]

        return MeshProcess(vertices,indices,textures)

    def getPosition(self):
        return self.position_







