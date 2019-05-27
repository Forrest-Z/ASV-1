# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testgui)
set(CMAKE_CXX_STANDARD 17)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"../serial/include"
    "./include/")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
   )
 
# 指定生成目标
add_executable (testgui guiclient.cc)
target_include_directories(testgui PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
target_link_libraries(testgui ${SERIAL_LIBRARY})
