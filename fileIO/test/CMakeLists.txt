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
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)


# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${CMAKE_CURRENT_SOURCE_DIR}/../include"
	"/usr/include" 
	 "/home/scar1et/Coding/ASV/third_party/eigen")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} "/usr/lib")
set(HEADER_FILES ${HEADER_FILES} 
	"${CMAKE_CURRENT_SOURCE_DIR}/../include/json.hpp"                            
	"${CMAKE_CURRENT_SOURCE_DIR}/../include/utilityio.h")


# 指定生成目标

add_executable (testutility testutilityIO.cc)
target_include_directories(testutility PRIVATE ${HEADER_DIRECTORY})

# add_executable (testjson testjson.cc ${HEADER_FILES})
add_executable (testjson testjson.cc)
target_include_directories(testjson PRIVATE ${HEADER_DIRECTORY})

# add_executable (testsqlite functions.cc ${HEADER_FILES})
add_executable (testsqlite functions.cc)
target_include_directories(testsqlite PRIVATE ${HEADER_DIRECTORY})
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
target_link_libraries(testsqlite PUBLIC ${SQLITE3_LIBRARY})
