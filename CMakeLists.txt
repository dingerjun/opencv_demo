SET(CMAKE_BUILD_TYPE "Debug")

cmake_minimum_required(VERSION 2.8)
project( 24ColorChecker )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( 24ColorChecker 24ColorChecker.cpp )
target_link_libraries( 24ColorChecker ${OpenCV_LIBS} )

cmake_minimum_required(VERSION 2.8)
project( DualCamera )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( DualCamera DualCamera.cpp )
target_link_libraries( DualCamera ${OpenCV_LIBS} )

cmake_minimum_required(VERSION 2.8)
project( CameraCanny )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( CameraCanny CameraCanny.cpp )
target_link_libraries( CameraCanny ${OpenCV_LIBS} )

cmake_minimum_required(VERSION 2.8)
project( FeaturePoint )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( FeaturePoint LocalFeature.cpp main.cpp )
target_link_libraries( FeaturePoint ${OpenCV_LIBS} )

cmake_minimum_required(VERSION 2.8)
project( StereoMatch )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(StereoMatch  stereo_match.cpp )
target_link_libraries( StereoMatch ${OpenCV_LIBS} )



#cmake_minimum_required(VERSION 2.8)
#project( Depth )
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#add_executable( Depth Depth.cpp StereoVision.cpp )
#target_link_libraries( Depth ${OpenCV_LIBS} )

