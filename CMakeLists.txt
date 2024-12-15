project(robot_calibrate)#必须
cmake_minimum_required()

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VISP REQUIRED)
find_package(jsoncpp REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${VISP_INCLUDE_DIRS})

set(ROBOT_DEPENLIBS "${VISP_LIBRARIES};aubo_sdk;aral;jsoncpp_lib")
set(VISON_DEPENLIBS "${OPENCV_LIBRARIES};")
add_subdirectory(modules/aubo)
add_subdirectory(modules/pybind)
add_subdirectory(modules/test)
add_subdirectory(modules/calibration)