cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar_sample_data)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs novatel_gps_msgs nmea_msgs  cv_bridge sensor_msgs pcl_ros message_generation)

add_service_files(DIRECTORY srv
                  FILES
                  Order.srv)

add_message_files(
  FILES
  Sampledata.msg
)

generate_messages(DEPENDENCIES sensor_msgs std_msgs novatel_gps_msgs nmea_msgs)#我们的message需要依赖的ros中的其他数据类型
catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include
		    ${PROJECT_SOURCE_DIR}
		    "/usr/include/eigen3"
                    ${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} 
		      ${OpenCV_LIBS} 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})  

add_executable(visualizer src/visualizer.cpp)
target_link_libraries(visualizer
		      ${OpenCV_LIBS} 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      )
add_executable(read src/read.cpp)
target_link_libraries(read
		      ${OpenCV_LIBS} 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      )                      
add_dependencies(visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS})  




