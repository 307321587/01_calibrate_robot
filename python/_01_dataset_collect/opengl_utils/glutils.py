"""
glutils.py
Author: Mahesh Venkitachalam
Some OpenGL utilities.
"""

import OpenGL
from OpenGL.GL import *
from OpenGL.GL.shaders import *

import numpy, math
import numpy as np



def perspective(fov, aspect, zNear, zFar):
    """returns matrix equivalent for gluPerspective"""
    fovR = math.radians(fov)
    f = 1.0 / math.tan(fovR / 2.0)
    return numpy.array([f / float(aspect), 0.0, 0.0, 0.0,
                        0.0, f, 0.0, 0.0,
                        0.0, 0.0, (zFar + zNear) / float(zNear - zFar), -1.0,
                        0.0, 0.0, 2.0 * zFar * zNear / float(zNear - zFar), 0.0],
                       numpy.float32)

def glm_perspective(fov, aspect, zNear, zFar):
    """returns matrix equivalent for gluPerspective"""
    f = 1.0 / math.tan(fov / 2.0)
    return numpy.array([f / float(aspect), 0.0, 0.0, 0.0,
                        0.0, f, 0.0, 0.0,
                        0.0, 0.0, (zFar + zNear) / float(zNear - zFar), -1.0,
                        0.0, 0.0, 2.0 * zFar * zNear / float(zNear - zFar), 0.0],
                       numpy.float32)


def ortho(l, r, b, t, n, f):
    """returns matrix equivalent of glOrtho"""
    return numpy.array([2.0 / float(r - l), 0.0, 0.0, 0.0,
                        0.0, 2.0 / float(t - b), 0.0, 0.0,
                        0.0, 0.0, -2.0 / float(f - n), 0.0,
                        -(r + l) / float(r - l), -(t + b) / float(t - b),
                        -(f + n) / float(f - n), 1.0],
                       numpy.float32)


def lookAt(eye, center, up):
    """returns matrix equivalent of gluLookAt - based on MESA implementation"""
    # create an identity matrix
    m = np.identity(4, np.float32)

    forward = np.array(center) - np.array(eye)
    norm = np.linalg.norm(forward)
    forward= forward/ norm

    # normalize up vector
    norm = np.linalg.norm(up)
    up =up / norm

    # Side = forward x up
    side = np.cross(forward, up)
    # Recompute up as: up = side x forward
    up = np.cross(side, forward)

    m[0][0] = side[0]
    m[1][0] = side[1]
    m[2][0] = side[2]

    m[0][1] = up[0]
    m[1][1] = up[1]
    m[2][1] = up[2]

    m[0][2] = -forward[0]
    m[1][2] = -forward[1]
    m[2][2] = -forward[2]

    # eye translation
    t = np.identity(4, np.float32)
    t[3][0] += -eye[0]
    t[3][1] += -eye[1]
    t[3][2] += -eye[2]

    return t.dot(m)


def translate(tx, ty, tz):
    """creates the matrix equivalent of glTranslate"""
    return np.array([1.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0,
                     tx, ty, tz, 1.0], np.float32)


def loadShaders(strVS, strFS):
    """load vertex and fragment shaders from strings"""
    # compile vertex shader
    shaderV = compileShader([strVS], GL_VERTEX_SHADER)
    # compiler fragment shader
    shaderF = compileShader([strFS], GL_FRAGMENT_SHADER)

    # create the program object
    program = glCreateProgram()
    if not program:
        raise RunTimeError('glCreateProgram faled!')

    # attach shaders
    glAttachShader(program, shaderV)
    glAttachShader(program, shaderF)

    # Link the program
    glLinkProgram(program)

    # Check the link status
    linked = glGetProgramiv(program, GL_LINK_STATUS)
    if not linked:
        infoLen = glGetProgramiv(program, GL_INFO_LOG_LENGTH)
        infoLog = ""
        if infoLen > 1:
            infoLog = glGetProgramInfoLog(program, infoLen, None);
        glDeleteProgram(program)
        raise RunTimeError("Error linking program:\n%s\n", infoLog);

    return program


def compileShader2(source, shaderType):
    """Compile shader source of given type
    source -- GLSL source-code for the shader
    shaderType -- GLenum GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, etc,
    returns GLuint compiled shader reference
    raises RuntimeError when a compilation failure occurs
    """
    if isinstance(source, str):
        print('string shader')
        source = [source]
    elif isinstance(source, bytes):
        print('bytes shader')
        source = [source.decode('utf-8')]

    shader = glCreateShader(shaderType)
    glShaderSource(shader, source)
    glCompileShader(shader)
    result = glGetShaderiv(shader, GL_COMPILE_STATUS)

    if not (result):
        # TODO: this will be wrong if the user has
        # disabled traditional unpacking array support.
        raise RuntimeError(
            """Shader compile failure (%s): %s""" % (
                result,
                glGetShaderInfoLog(shader),
            ),
            source,
            shaderType,
        )
    return shader


# a simple camera class
def _build_coordinate_frame_changing_transformation_matrix(destination_frame: list) -> np.ndarray:
    """ Builds a transformation matrix that switches the coordinate frame.

    :param destination_frame: An array containing three elements, describing each axis of the destination coordinate frame based on the axes of the source frame. (Allowed values: "X", "Y", "Z", "-X", "-Y", "-Z")
    :return: The transformation matrix
    """
    assert len(destination_frame) == 3, "The specified coordinate frame has more or less than tree axes: {}".format(destination_frame)

    # Build transformation matrix that maps the given matrix to the specified coordinate frame.
    tmat = np.zeros((4, 4))
    for i, axis in enumerate(destination_frame):
        axis = axis.upper()

        if axis.endswith("X"):
            tmat[i, 0] = 1
        elif axis.endswith("Y"):
            tmat[i, 1] = 1
        elif axis.endswith("Z"):
            tmat[i, 2] = 1
        else:
            raise Exception("Invalid axis: " + axis)

        if axis.startswith("-"):
            tmat[i] *= -1
    tmat[3, 3] = 1
    return tmat

def change_coordinate_of_transformation_matrix(matrix: np.ndarray, new_frame: list) -> np.ndarray:
    """ Changes the coordinate frame the given transformation matrix is mapping from.

    Given a matrix $T_A^B$ that maps from A to B, this function can be used
    to change the axes of A into A' and therefore end up with $T_A'^B$.

    :param matrix: The matrix to convert in form of a np.ndarray or mathutils.Matrix
    :param new_frame: An array containing three elements, describing each axis of the new coordinate frame based on the axes of the current frame. (Allowed values: "X", "Y", "Z", "-X", "-Y", "-Z")
    :return: The converted matrix is in form of a np.ndarray
    """
    tmat = _build_coordinate_frame_changing_transformation_matrix(new_frame)
    tmat = np.linalg.inv(tmat)

    # Apply transformation matrix
    output = np.matmul(matrix, tmat)
    return output

def change_transformation_matrix_opencv2opengl(matrix:np.ndarray)-> np.ndarray:

    # 将转移矩阵由opencv坐标系转换至opengl坐标系
    # 相当于绕x轴转了180度
    matrix = change_coordinate_of_transformation_matrix(matrix, ["X", "-Y", "-Z"])

    # 原转移矩阵为世界坐标到相对于相机的转换
    # 需要转换为物体相对于相机的转换,相机不变
    view_opengl = matrix.copy()
    view_opengl[1, 0] = -matrix[1, 0]
    view_opengl[1, 3] = -matrix[1, 3]
    view_opengl[2, 0] = -matrix[2, 0]
    view_opengl[2, 3] = -matrix[2, 3]
    view_opengl[0, 1] = -matrix[0, 1]
    view_opengl[0, 2] = -matrix[0, 2]
    view_opengl[3, 1] = -matrix[3, 1]
    view_opengl[3, 2] = -matrix[3, 2]
    view_opengl = np.transpose(view_opengl)

    return view_opengl

def change_transformation_matrix_opengl2opencv(matrix:np.ndarray)-> np.ndarray:

    view_opencv=np.transpose(matrix)

    view_opengl=view_opencv.copy()

    view_opencv[1, 0] = -view_opengl[1, 0]
    view_opencv[1, 3] = -view_opengl[1, 3]
    view_opencv[2, 0] = -view_opengl[2, 0]
    view_opencv[2, 3] = -view_opengl[2, 3]
    view_opencv[0, 1] = -view_opengl[0, 1]
    view_opencv[0, 2] = -view_opengl[0, 2]
    view_opencv[3, 1] = -view_opengl[3, 1]
    view_opencv[3, 2] = -view_opengl[3, 2]

    view_opencv=change_coordinate_of_transformation_matrix(view_opencv, ["X", "-Y", "-Z"])

    return view_opencv

def set_intrinsics_from_K_matrix(K,width,height,near,far):
    """
    参考:https://blog.csdn.net/hjwang1/article/details/94781272
    此处投影锥体矩阵-1位置与B交换
    :param K: 相机内参矩阵,与视觉slam14讲一致
    :param width: 图像宽度
    :param height: 图像高度
    :param near: 视野近点
    :param far: 视野远点
    :return: projection矩阵 opengl
    """
    fx, fy = K[0][0], K[1][1]
    cx, cy = K[0][2], K[1][2]

    projection = np.array([
        [2 * fx / width, 0,  1 - 2 * cx / width, 0],
        [0, 2 * fy / height, 2 * cy / height-1 , 0],
        [0, 0, -(far + near) / (far - near), -2 * far * near / (far - near)],
        [0, 0, -1, 0]
    ]).transpose()

    return projection
