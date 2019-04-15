# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.5)

# 项目信息
project (testfileio)
set(CMAKE_CXX_STANDARD 14)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)


# 添加 include 子目录
INCLUDE_DIRECTORIES ( "/home/scar1et/Coding/ASV/third_party/eigen" )
set(HEADER_FILES ../include/json.hpp ../include/utilityio.h)
# add_subdirectory (../include)


# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
# aux_source_directory(. DIR_SRCS)

# 指定生成目标
add_executable (Testutility test.cpp ${HEADER_FILES})


