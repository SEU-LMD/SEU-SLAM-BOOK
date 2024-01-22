cmake_minimum_required( VERSION 2.8 )
project( posegraph )
set( CMAKE_BUILD_TYPE "Debug" )
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
# 设置本程序是opencv3
set(OpenCV_DIR "/usr/local/opencv320/share/OpenCV/")
# 添加cmake模块以使用g2o库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( Sophus REQUIRED )
find_package( G2O REQUIRED )
find_package(Cholmod REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread date_time regex timer chrono)#如果想使用gtsam必须加上寻找boost库
find_package( GTSAM REQUIRED )

find_package(Ceres REQUIRED)

# 添加头文件
include_directories( 		     
		      "/home/fyy/Documents/lib/SuiteSparse-master/CSparse/Include/"
		      "/home/fyy/Documents/my_code/relative_positioning/deal_string/"
		     ${Boost_INCLUDE_DIR}
		     ${GTSAM_INCLUDE_DIR}
                     ${OpenCV_INCLUDE_DIRS}  
		     ${Sophus_INCLUDE_DIRS}  
		     ${G2O_INCLUDE_DIRS}
		     ${CHOLMOD_INCLUDE_DIR}
		     ${CERES_INCLUDE_DIRS} 
		     ${PROJECT_SOURCE_DIR}
		     "/usr/include/eigen3"
		    )

add_executable( posegraph ${PROJECT_SOURCE_DIR}/src/posegraph.cpp )
# 链接库
target_link_libraries(posegraph 
		      /home/fyy/Documents/lib/SuiteSparse-master/CSparse/Lib/libcsparse.a
		      g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d g2o_types_sim3
		      ${OpenCV_LIBS} 
		      ${Sophus_LIBRARIES}
		      ${CHOLMOD_LIBRARIES} 
		      ${Boost_LIBRARIES}
		      ${CERES_LIBRARIES}
		      gtsam
		      /home/fyy/Documents/my_code/relative_positioning/deal_string/build/libdeal_string.so
		      )
