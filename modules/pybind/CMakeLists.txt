cmake_minimum_required(VERSION 3.15...3.29)

include_directories(${PROJECT_SOURCE_DIR}/third_party/aubo/inc)
link_directories(${PROJECT_SOURCE_DIR}/third_party/aubo/lib/x64)
message(STATUS "${PROJECT_SOURCE_DIR}")

set(pybind11_DIR "/home/lza/miniconda3/envs/robot/lib/python3.10/site-packages/pybind11/share/cmake/pybind11")
# set(PYTHON_EXECUTABLE "/home/lza/miniconda3/envs/robot/bin/python") cmake 3.14
set(Python3_ROOT_DIR "/home/lza/miniconda3/envs/robot")# cmake 3.30
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

include_directories("/home/lza/miniconda3/envs/robot/lib/python3.10/site-packages/pybind11/include")
find_package(pybind11 CONFIG REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/modules/aubo/include)
file(GLOB mains RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
include_directories("include")

pybind11_add_module(robot src/aubo_sdk.cpp)
install(TARGETS robot DESTINATION .)
target_link_libraries(robot PRIVATE ${VISON_DEPENLIBS} ${ROBOT_DEPENLIBS} AuboControl) 

