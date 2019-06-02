# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.5)

# 项目信息
project (testtimer)
set(CMAKE_CXX_STANDARD 14)

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
set(PROJECT_DIRECTORY ${PROJECT_DIRECTORY} "/home/scar1et/Coding/ASV")

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${CMAKE_CURRENT_SOURCE_DIR}/../include"
	"/usr/include" )


set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} "/usr/lib")
# 指定生成目标

add_executable (testtimer testtimer.cc)
target_include_directories(testtimer PRIVATE ${HEADER_DIRECTORY})
