configure_file(package.xml.in ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)



set(ROS_DISTRO $ENV{ROS_DISTRO})
set(LD_LIBRARY_PATH $ENV{LD_LIBRARY_PATH})
message( LD_LIBRARY_PATH = ${LD_LIBRARY_PATH})
message( ROS_DISTRO = ${ROS_DISTRO})
set(ROS_INSTALL_PATH /opt/ros/${ROS_DISTRO})



#add_ros(ros_lib rosconsole roscpp  roscpp_serialization xmlrpcpp rostime cpp_common)
function(add_ros target)

#    target_link_directories(${target} PUBLIC ${ROS_INSTALL_PATH}/lib)
#    target_link_libraries(${target} PUBLIC
#            /opt/ros/noetic/lib/librosconsole.so
#            /opt/ros/noetic/lib/librosconsole_log4cxx.so
#            /opt/ros/noetic/lib/librosconsole_backend_interface.so
#            /lib/x86_64-linux-gnu/liblog4cxx.so.10
#            /opt/ros/noetic/lib/librostime.so
#            /lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
#            /lib/x86_64-linux-gnu/libpthread.so.0
#            /lib/x86_64-linux-gnu/libstdc++.so.6
#            /lib/x86_64-linux-gnu/libgcc_s.so.1
#            /lib/x86_64-linux-gnu/libc.so.6
#            /lib/x86_64-linux-gnu/libapr-1.so.0
#            /lib/x86_64-linux-gnu/libaprutil-1.so.0
#            /lib/x86_64-linux-gnu/libm.so.6
#            /lib/x86_64-linux-gnu/libicui18n.so.66
#            /lib/x86_64-linux-gnu/libicuuc.so.66
#            /lib64/ld-linux-x86-64.so.2
#            /lib/x86_64-linux-gnu/libuuid.so.1
#            /lib/x86_64-linux-gnu/libdl.so.2
#            /lib/x86_64-linux-gnu/libcrypt.so.1
#            /lib/x86_64-linux-gnu/libexpat.so.1
#            /lib/x86_64-linux-gnu/libicudata.so.66
#            )


    set(ROS_LIB_LIST ${ARGN})
    message(add_ros ROS_LIB_LIST "\n ${ROS_LIB_LIST}\n")

    foreach(e IN LISTS ARGN)
        set(${e}_ROOT ${ROS_INSTALL_PATH}/share)
        message(${e}_ROOT : ${${e}_ROOT})
        find_package(${e} REQUIRED )
        message( "find ${e}_INCLUDE_DIRS: " ${${e}_INCLUDE_DIRS} )
        message("find ${e}_LIBRARIES: " ${${e}_LIBRARIES} )
        target_link_libraries(${target} PUBLIC ${${e}_LIBRARIES})
        target_include_directories(${target} PUBLIC ${${e}_INCLUDE_DIRS})
    endforeach()
    target_include_directories(${target} PUBLIC ${ROS_INSTALL_PATH}/include)
endfunction()




#
#if(  DEFINED catkin_package)
#    #file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/package.xml  ${PACKAGE_XML_CONTENT})
#    catkin_package(
#            INCLUDE_DIRS include
#            LIBRARIES ${PROJECT_NAME})
#
#    set(shared_dirs "launch" "param" )
#    foreach(dir ${shared_dirs})
#        if(EXISTS "${dir}" AND IS_DIRECTORY "${dir}")
#            message("EXISTS：${dir}")
#            #Installing roslaunch Files or Other Resources
#            install(DIRECTORY ${dir}/
#                    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#                    PATTERN ".svn" EXCLUDE)
#        endif()
#    endforeach()
#
#
#
#endif()
#
#
