# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testGPS)
set(CMAKE_CXX_STANDARD 17)


#GeographicLib库
find_package (GeographicLib 1.49 REQUIRED)

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../third_party/eigen"
	"${PROJECT_SOURCE_DIR}/../../../third_party/serial/include"
    "../include/")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
   )
    
# 指定生成目标
add_executable (testGPS testGPS.cc)
target_include_directories(testGPS PRIVATE ${HEADER_DIRECTORY})
find_library(SERIAL_LIBRARY serial ${LIBRARY_DIRECTORY})
target_link_libraries(testGPS ${SERIAL_LIBRARY})
target_link_libraries(testGPS ${GeographicLib_LIBRARIES})