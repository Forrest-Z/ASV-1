# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testgui)
set(CMAKE_CXX_STANDARD 17)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../sensors/gpsimu/include"
	"${PROJECT_SOURCE_DIR}/../../property/include"
	"${PROJECT_SOURCE_DIR}/../../controller/include"
	"${PROJECT_SOURCE_DIR}/../../controller/motor/include"
	"${PROJECT_SOURCE_DIR}/../../estimator/include"
	"${PROJECT_SOURCE_DIR}/../../planner/include"
	"${PROJECT_SOURCE_DIR}/../../third_party/eigen"
	"${PROJECT_SOURCE_DIR}/../../third_party/serial/include"
	"${PROJECT_SOURCE_DIR}/../../timer/include"
	"${PROJECT_SOURCE_DIR}/../../fileIO/include"
	"${PROJECT_SOURCE_DIR}/../../logging/include"
    "../include/")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
   )
 
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../logging/src/easylogging++.cc"                            
	"${CMAKE_CURRENT_SOURCE_DIR}/testguicommunication.cc")  
# 指定生成目标
add_executable (testgui ${SOURCE_FILES})
target_include_directories(testgui PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
target_link_libraries(testgui ${SERIAL_LIBRARY})



add_executable (testcrc "testcrc.cc")
target_include_directories(testcrc PRIVATE ${HEADER_DIRECTORY})

