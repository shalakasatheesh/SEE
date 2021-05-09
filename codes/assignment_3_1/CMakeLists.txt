cmake_minimum_required(VERSION 2.8)
## Setting the project name
project( Camera_calibration )

##asking it to find required packages
find_package( OpenCV REQUIRED )
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

##Add executable
add_executable( Assignment_3_1 assignment_3_1.cpp )


##Target link libraries
target_link_libraries( Assignment_3_1 ${OpenCV_LIBS})
target_link_libraries( Assignment_3_1 Eigen3::Eigen)
