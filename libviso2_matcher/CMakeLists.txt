cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        cv_bridge 
                                        image_transport
                                        dynamic_reconfigure
                                        libviso2_matcher_msgs
                                        )
                                        
find_package(OpenCV REQUIRED)

project(libviso2_matcher)

generate_dynamic_reconfigure_options(
  cfg/matcher_parameters.cfg
)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=c++11")

include_directories(include . ${catkin_INCLUDE_DIRS})

add_library(libviso2 SHARED        libviso2/matcher.cpp
                                   libviso2/matrix.cpp
                                   libviso2/triangle.cpp
                                   libviso2/filter.cpp)
                                   
add_executable(matcher_node 			 src/matcher_node.cpp
																	 src/libviso2_matcher_wrapper.cpp
																	 src/feature_manager.cpp)

target_link_libraries(matcher_node ${catkin_LIBRARIES} libviso2)

add_dependencies(matcher_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
