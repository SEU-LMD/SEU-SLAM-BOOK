cmake_minimum_required(VERSION 2.8)
project(KITTICornerDetection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC  -lm -Wl,--no-as-needed -DLINUX -g -Wall \
-std=c++11 -Wunused-variable -Wwrite-strings -Wsign-compare -fpermissive ")
set(CMAKE_BUILD_TYPE "Release")

set(OpenCV_DIR "/usr/local/OpenCV345/share/OpenCV")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(OpenCV_LIBS ${OpenCV_LIBRARIES})



set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

INCLUDE_DIRECTORIES(./)
AUX_SOURCE_DIRECTORY(./ gSOURCE_FILES_)

add_library(${PROJECT_NAME} SHARED 
	    ${gSOURCE_FILES_}
  	    )    
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
