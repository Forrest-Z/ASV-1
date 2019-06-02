# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testfileio)
set(CMAKE_CXX_STANDARD 17)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)


# thread 库
find_package(Threads REQUIRED)


# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../sensors/gpsimu/include"
	"${PROJECT_SOURCE_DIR}/../../controller/include"
	"${PROJECT_SOURCE_DIR}/../../estimator/include"
	"${PROJECT_SOURCE_DIR}/../../planner/include"
	"${PROJECT_SOURCE_DIR}/../../timer/include"
	"${PROJECT_SOURCE_DIR}/../../fileIO/include"
	"${PROJECT_SOURCE_DIR}/../../property/include"
	"${PROJECT_SOURCE_DIR}/../../logging/include"
	"${PROJECT_SOURCE_DIR}/../../third_party/eigen"
	"/usr/include" )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} "/usr/lib")


# 指定生成目标

add_executable (testutility testutilityIO.cc)
target_include_directories(testutility PRIVATE ${HEADER_DIRECTORY})

# add_executable (testjson testjson.cc ${HEADER_FILES})
add_executable (testjson testjson.cc)
target_include_directories(testjson PRIVATE ${HEADER_DIRECTORY})


set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../logging/src/easylogging++.cc"                            
	"${CMAKE_CURRENT_SOURCE_DIR}/testsqlite.cc")
add_executable (testsqlite ${SOURCE_FILES})
target_include_directories(testsqlite PRIVATE ${HEADER_DIRECTORY})
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
target_link_libraries(testsqlite PUBLIC ${SQLITE3_LIBRARY})
target_link_libraries(testsqlite PUBLIC ${CMAKE_THREAD_LIBS_INIT})

add_executable (testcsv testcsv.cc)
target_include_directories(testcsv PRIVATE ${HEADER_DIRECTORY})

