# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testestimator)
set(CMAKE_CXX_STANDARD 17)

# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_INCLUDE_CURRENT_DIR ON)


# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${CMAKE_CURRENT_SOURCE_DIR}/../include"
	"/usr/include" 
	"${PROJECT_SOURCE_DIR}/../../fileIO/include"
	"${PROJECT_SOURCE_DIR}/../../property/include"
	"${PROJECT_SOURCE_DIR}/../../third_party/eigen"
	"/opt/mosek/8/tools/platform/linux64x86/h")


set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} "/usr/lib")
# 指定生成目标

add_executable (testkalman testkalman.cc)
target_include_directories(testkalman PRIVATE ${HEADER_DIRECTORY})
