find_package(Eigen3 REQUIRED )
message(EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS} )

add_library(ros_message_manager SHARED RosMessageManager.cpp)
add_ros(ros_message_manager roscpp)
target_link_libraries(ros_message_manager PUBLIC tf )
target_include_directories(ros_message_manager PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIR})


install_target(ros_message_manager)
