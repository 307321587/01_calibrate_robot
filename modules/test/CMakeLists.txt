
file(GLOB mains RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
include_directories("include")

include_directories(${PROJECT_SOURCE_DIR}/third_party/aubo/inc)
include_directories(${PROJECT_SOURCE_DIR}/modules/aubo/include)

link_directories(${PROJECT_SOURCE_DIR}/third_party/aubo/lib/x64)

message(STATUS "${PROJECT_SOURCE_DIR}")

foreach(mainfile IN LISTS mains)
    # Get file name without directory
    
    get_filename_component(mainname ${mainfile} NAME_WE)
    add_executable(${mainname} ${mainfile})
    target_link_libraries(${mainname} ${VISON_DEPENLIBS} ${ROBOT_DEPENLIBS} AuboControl pthread)  
  endforeach()
# target_link_libraries(test_calibration calibration)

