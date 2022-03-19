cmake_minimum_required(VERSION 3.1)
project(VisionInfantry)
set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH          ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/Libs)




###调用本地头文件###
include_directories(Include)
include_directories(Include/SerialPort)
include_directories(Include/Src/ArmorFinder)
include_directories(DeltaCV-master/cpu/include)
###end###

###调用opencv###
find_package(OpenCV)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${OpenCV_LIBRARY_DIRS})
###end###

###eigen###
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
###end###

###Boost###
find_package(Boost REQUIRED COMPONENTS system atomic)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
###end###




###动态链接库###
set(LINK_DIR
        /opt/DahuaTech/MVviewer/lib/GenICam/bin/Linux64_x64
        /opt/DahuaTech/MVviewer/lib
        Libs/
        )
link_directories( ${LINK_DIR} )
###end###





###Dahua头文件及相机共享内存###
include_directories(Include/Camera /opt/DahuaTech/MVviewer/include DeltaCV-master/cpu/include/deltaCV/SIMD)
add_subdirectory(Camera)
###end###







add_executable(Vision
        Src/main.cpp
        Include/Src/ArmorFinder/ArmorLibs.hpp
        Include/ForDefine.h
        Src/shm.cpp
        SerialPort/PublishAsync.cpp
        SerialPort/Serial_port.cpp
        SerialPort/SubscribeAsync.cpp
        Include/SerialPort/imgSubscrible.h
        Include/SerialPort/imgPublish.h
        Include/SerialPort/Serial_port.h


        Include/Src/windmill/Energy.h

        )

TARGET_LINK_LIBRARIES(Vision
        ${OpenCV_LIBS}
        ${Boost_LIBRARY}
        boost_thread boost_system
        timer
        imgSubscrible rt -pthread)



