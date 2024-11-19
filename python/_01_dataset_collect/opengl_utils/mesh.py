import pyassimp
import pyassimp.postprocess
from OpenGL.GL import *
from OpenGL import GL as gl
import numpy as np
from ctypes import c_float, sizeof, c_void_p, Structure

Vec2 = (2 * c_float)
Vec3 = (3 * c_float)

class Vertex(Structure):
    _fields_ = [
        ("Position",    Vec3),
        ("Normal",      Vec3),
        ("TexCoords",   Vec2),
        ("Tangent",     Vec3),
        ("Bitangent",   Vec3),
    ]


class MeshProcess:

    def __init__(self,data,indices,textures):
        # vertices为字典
        self.data=data
        self.indices=indices
        self.textures=textures
        self.vao = None

        self._vbo = None
        self._ebo = None

        self.setupMesh()

    def Draw(self,shader):

        diffuseNr = 1
        specularNr = 1
        normalNr = 1
        heightNr = 1


        for i,texture in enumerate(self.textures):
            glActiveTexture(GL_TEXTURE0+i)
            number=0
            name=texture.type
            if (name == "texture_diffuse"):
                diffuseNr = diffuseNr + 1
                number = str(diffuseNr)
            elif(name == "texture_specular"):
                specularNr = specularNr + 1
                number = str(specularNr)
            elif(name == "texture_normal"):
                normalNr = normalNr + 1
                number = str(normalNr)
            elif(name == "texture_height"):
                heightNr = heightNr + 1
                number = str(heightNr)
            glUniform1i(glGetUniformLocation(shader.get_ID()),str(int(name)+int(number)),i)
            glBindBuffer(GL_TEXTURE_2D,texture.id)

        # 绘制网格
        glBindVertexArray(self.vao)
        glDrawElements(GL_TRIANGLES, len(self.indices)*4, GL_UNSIGNED_INT, None)
        glBindVertexArray(0)
        glActiveTexture(GL_TEXTURE0)

    def setupMesh(self):

        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        self._vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, sizeof(self.data), self.data, gl.GL_STATIC_DRAW)

        self._ebo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self._ebo)
        gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER, sizeof(self.indices), self.indices, gl.GL_STATIC_DRAW)

        # -- set vertex attibute pointers
        # -- vertex positions
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE,
                                 sizeof(Vertex), c_void_p(Vertex.Position.offset))

        # -- vertex normals
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE,
                                 sizeof(Vertex), c_void_p(Vertex.Normal.offset))

        # -- vertex texture coords
        gl.glEnableVertexAttribArray(2)
        gl.glVertexAttribPointer(2, 2, gl.GL_FLOAT, gl.GL_FALSE,
                                 sizeof(Vertex), c_void_p(Vertex.TexCoords.offset))

        # -- vertex tangent
        gl.glEnableVertexAttribArray(3)
        gl.glVertexAttribPointer(3, 3, gl.GL_FLOAT, gl.GL_FALSE,
                                 sizeof(Vertex), c_void_p(Vertex.Tangent.offset))

        # -- vertex bitangent
        gl.glEnableVertexAttribArray(4)
        gl.glVertexAttribPointer(4, 3, gl.GL_FLOAT, gl.GL_FALSE,
                                 sizeof(Vertex), c_void_p(Vertex.Bitangent.offset))

        gl.glBindVertexArray(0)
