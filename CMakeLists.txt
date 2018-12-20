cmake_minimum_required(VERSION 2.8.3)
project(dvs_hot_pixel_filter)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# make the executable
cs_add_executable(${PROJECT_NAME} src/hot_pixel_filter.cpp)

# link required libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

cs_install()
