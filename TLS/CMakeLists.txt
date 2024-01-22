cmake_minimum_required( VERSION 2.8 )
project( TLS )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set(OpenCV_DIR "/usr/local/opencv320/share/OpenCV/")

# 添加cmake模块以使用g2o库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( G2O REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Sophus REQUIRED )
find_package(Cholmod REQUIRED)
find_package(Ceres REQUIRED)
include_directories( 
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
    "/home/fyy/Documents/my_code/relative_positioning/deal_string/"
    ${CERES_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} 
    ${Sophus_INCLUDE_DIRS} 
    ${CHOLMOD_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
)

# 寻找OpenCV库
add_executable( TLS ${PROJECT_SOURCE_DIR}/src/TLS.cpp )

# 与G2O库、自己写的库和opencv库进行链接进行链接进行链接
target_link_libraries( TLS
    g2o_core g2o_stuff g2o_types_sba
    /home/fyy/Documents/my_code/relative_positioning/deal_string/build/libdeal_string.so
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${CHOLMOD_LIBRARIES} 
    ${CERES_LIBRARIES}
)
