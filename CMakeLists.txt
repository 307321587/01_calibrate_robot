project(robot_calibrate)                                          #必须
cmake_minimum_required()

find_package(OpenCV REQUIRED)
 

include_directories(${OpenCV_INCLUDE_DIRS})

set(ROBOT_DEPENLIBS "aubo_sdk;aral")
set(VISON_DEPENLIBS "${OPENCV_LIBRARIES};")


add_subdirectory(modules/test)
add_subdirectory(modules/calibration)
add_subdirectory(modules/aubo)
add_subdirectory(modules/pybind)
