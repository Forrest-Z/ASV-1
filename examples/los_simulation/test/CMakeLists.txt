# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testlossimulation)
set(CMAKE_CXX_STANDARD 17)

# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#GeographicLib库
find_package (GeographicLib 1.49 REQUIRED)
# thread 库
find_package(Threads REQUIRED)

# 添加 include 子目录
set(PROJECT_DIRECTORY ${PROJECT_DIRECTORY} "/home/scar1et/Coding/ASV")


set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_DIRECTORY}/sensors/gpsimu/include"
	"${PROJECT_DIRECTORY}/controller/include"
	"${PROJECT_DIRECTORY}/estimator/include"
	"${PROJECT_DIRECTORY}/planner/include"
	"${PROJECT_DIRECTORY}/timer/include"
	"${PROJECT_DIRECTORY}/fileIO/include"
	"${PROJECT_DIRECTORY}/property/include"
	"${PROJECT_DIRECTORY}/logging/include"
	"${PROJECT_DIRECTORY}/third_party/eigen"
	"/opt/mosek/8/tools/platform/linux64x86/h"
	"/usr/include" 
    "${CMAKE_CURRENT_SOURCE_DIR}/../include")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/opt/mosek/8/tools/platform/linux64x86/bin"
   )
    
# 指定生成目标
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_DIRECTORY}/logging/src/easylogging++.cc"                            
	"${CMAKE_CURRENT_SOURCE_DIR}/testbiling.cc")
add_executable (testlos ${SOURCE_FILES})
target_include_directories(testlos PRIVATE ${HEADER_DIRECTORY})
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
find_library(MOSEK8_LIBRARY mosek64 HINTS ${LIBRARY_DIRECTORY})
target_link_libraries(testlos PUBLIC ${SQLITE3_LIBRARY})
target_link_libraries(testlos PUBLIC ${GeographicLib_LIBRARIES})
target_link_libraries(testlos PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(testlos PUBLIC ${MOSEK8_LIBRARY})
