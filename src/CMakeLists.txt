# This file handles building each task as sub-project.
#
# Each task becomes an independent project in which you can experiment.
#
# Tasks are added as separate sub-projects so as compilation errors in
# one task don't affect another task.


project(cmake_super_build)

set(CMAKE_CXX_STANDARD 14)

#add_executable(o3d_test o3d_test.cpp)
#target_link_libraries(o3d_test PRIVATE Open3D::Open3D )
#set_target_properties(Open3d_DrawWebRTC PROPERTIES LINK_FLAGS "-Wl,--dynamic-linker=$ENV{CUSTOM_GLIBC_PATH}/lib/ld-linux-x86-64.so.2")  # set RPATH ok ok
#target_link_options( Open3d_DrawWebRTC PRIVATE "SHELL:-Wl,-L$ENV{CUSTOM_GLIBC_PATH}/lib" )
#target_link_directories(Open3d_DrawWebRTC PUBLIC  $ENV{CUSTOM_GLIBC_PATH}/lib )
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--dynamic-linker=$ENV{CUSTOM_GLIBC_PATH}/lib/ld-linux-x86-64.so.2")
#message("use CUSTOM_GLIBC_PATH 1: " $ENV{CUSTOM_GLIBC_PATH} )
#
#set_target_rpath(Open3d_DrawWebRTC $ENV{CUSTOM_GLIBC_PATH})

#add_executable(cv_process_mat cv_process_mat.cpp)
#target_link_libraries(cv_process_mat PUBLIC
#        ${OpenCV_LIBS}
#        )
#
#target_include_directories(cv_process_mat PUBLIC
#        ${OpenCV_INCLUDE_DIRS})


add_executable(ceres_simple_solve ceres_simple_solve.cpp)

target_link_libraries(ceres_simple_solve PUBLIC
        ${CERES_LIBRARIES}
        )
target_include_directories(ceres_simple_solve PUBLIC  ${CMAKE_SOURCE_DIR}/include)
#set_asan(ceres_simple_solve)

install(TARGETS ceres_simple_solve DESTINATION bin)

#find_package(Boost 1.58 REQUIRED COMPONENTS system filesystem thread program_options chrono)


if(BUILD_ROS)
    add_subdirectory(message/impl/ros)
    add_subdirectory(message/impl/mqtt)

    add_executable(message_manager_test message_manager_test.cpp )
    #add_ros(message_manager_test roscpp)
    #target_link_libraries(message_manager_test PUBLIC tf )
    target_include_directories(message_manager_test PUBLIC
            ${CMAKE_SOURCE_DIR}/third_party/nlohmann
            ${CMAKE_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/src
            ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(message_manager_test PUBLIC  ros_message_manager mqtt_message_manager absl::strings)
    #target_include_directories(message_manager_test PUBLIC ${CMAKE_SOURCE_DIR}/third_party/sol ${mosquitto_INCLUDE_DIR})

    set_asan(message_manager_test)
endif ()






add_executable(fruit_greet fruit_greet.cpp)
target_link_libraries(fruit_greet PUBLIC
        ${FRUIT_LIBRARY}
        )
target_include_directories(
        fruit_greet PUBLIC
        ${FRUIT_INCLUDE_DIR}
)

#install(IMPORTED_RUNTIME_ARTIFACTS ${FRUIT_LIBRARY})
#install(CODE [[
#    file(GET_RUNTIME_DEPENDENCIES
#        RESOLVED_DEPENDENCIES_VAR RES
#        UNRESOLVED_DEPENDENCIES_VAR UNRES
#        CONFLICTING_DEPENDENCIES_PREFIX CONFLICTING_DEPENDENCIES
#        EXECUTABLES $<TARGET_FILE:fruit_greet>
#        DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR}
#        PRE_EXCLUDE /usr /lib
#        LIBRARIES ${FRUIT_LIBRARY}
#    )
#
#    message("\n\nFound dependencies :")
#    foreach(DEP ${RES})
#        message("${DEP}")
#    endforeach()
#    message("\n\nNot found dependencies :")
#    foreach(DEP ${UNRES})
#        message("${DEP}")
#    endforeach()
#]])
set(MY_DEPENDENCY_PATHS ${Fruit_ROOT})

# Transfer the value of ${MY_DEPENDENCY_PATHS} into the install script

install(CODE "set(my_pre_exclude_path \"/usr /lib\")")
# https://stackoverflow.com/questions/62884439/how-to-use-cmake-file-get-runtime-dependencies-in-an-install-statement/64009154#64009154
# https://discourse.cmake.org/t/file-get-runtime-dependencies-issues/2574/3

# "^/usr/.*\\.so(.+|.?)"




install(CODE
        [[
        set(__install_target $<TARGET_FILE:fruit_greet>)
        ]]
        )
install_deps(fruit_greet)

#install(TARGETS fruit_greet DESTINATION bin)
#
#install(CODE [[
#
#  file(GET_RUNTIME_DEPENDENCIES
#
#    EXECUTABLES ${__install_target}
#    RESOLVED_DEPENDENCIES_VAR _r_deps
#    PRE_EXCLUDE_REGEXES "usr.*"
#    POST_EXCLUDE_REGEXES "^/(usr|lib).*"
#    UNRESOLVED_DEPENDENCIES_VAR _u_deps
#    DIRECTORIES ${MY_DEPENDENCY_PATHS}
#  )
#  message("\n\nFound dependencies :")
#
#  foreach(dep_filename ${_r_deps})
#  message("-- ${dep_filename}")
#
#      if (IS_ABSOLUTE ${dep_filename})
#
#        endif()
#       file(INSTALL
#              DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
#              TYPE SHARED_LIBRARY
#              FOLLOW_SYMLINK_CHAIN
#              FILES "${dep_filename}"
#            )
#  endforeach()
#  list(LENGTH _u_deps _u_length)
#  if("${_u_length}" GREATER 0)
#    message(WARNING "Unresolved dependencies detected!")
#  endif()
#]])



set(schema_path "schema.fbs")
get_filename_component(fb_path ${schema_path} ABSOLUTE)

if(NOT EXISTS ${fb_path})
    message(FATAL_ERROR "${Red}Schema ${schema_path} not found${ColourReset}")
endif()

set(schema_build_path "${CMAKE_CURRENT_BINARY_DIR}/schema")
set(schema "${schema_build_path}/schema_generated.h")
add_custom_command(
        OUTPUT "${schema}"
        COMMAND ${FLATBUFFERS_COMPILER}
        ARGS  -o ${schema_build_path}
        -c
        ${fb_path}
        COMMENT "Creating ${schema}"
        DEPENDS ${fb_path}
)

# flatbuffers::flatc

add_executable(zmq_flatbuffer_hello zmq_flatbuffer_hello.cpp ${schema} )
target_include_directories(zmq_flatbuffer_hello PUBLIC
        ${cppzmq_INCLUDE_DIR}
        ${ZeroMQ_INCLUDE_DIR}
        ${schema_build_path}

        )
target_link_libraries(zmq_flatbuffer_hello PUBLIC
        ${ZeroMQ_LIBRARY}
        flatbuffers::flatbuffers
        )

print_include_dir(zmq_flatbuffer_hello)



add_executable(zmq_hello zmq_hello.cpp)
target_include_directories(zmq_hello PUBLIC
        ${cppzmq_INCLUDE_DIR}
        ${ZeroMQ_INCLUDE_DIR}
        )
target_link_libraries(zmq_hello PUBLIC
        ${ZeroMQ_LIBRARY}

        )
target_link_libraries( zmq_hello PUBLIC Threads::Threads)

add_executable(zmq_tcp_test  zmq_tcp_test.cpp)
target_include_directories(zmq_tcp_test PUBLIC
        ${cppzmq_INCLUDE_DIR}
        ${ZeroMQ_INCLUDE_DIR}
        )
target_link_libraries(zmq_tcp_test PUBLIC
        ${ZeroMQ_LIBRARY}

        )
find_package(OpenMP REQUIRED)
message( OpenMP_CXX_LIBRARIES : ${OpenMP_CXX_LIBRARIES}, OpenMP_CXX_FLAGS : ${OpenMP_CXX_FLAGS}, OpenMP_EXE_LINKER_FLAGS : ${OpenMP_EXE_LINKER_FLAGS} )
add_executable(omp_hello  omp_hello.cpp)
set_omp(omp_hello)


add_executable(tbb_hello  tbb_hello.cpp)
#target_include_directories(tbb_hello PUBLIC )
target_link_libraries(tbb_hello PUBLIC TBB::tbb)
#target_include_directories(omp_hello PUBLIC ${OpenMP_CXX_INCLUDE_DIRS} )
#target_link_libraries(omp_hello PUBLIC
#        ${OpenMP_CXX_LIBRARIES}
#        )
#target_compile_options(omp_hello PUBLIC ${OpenMP_CXX_FLAGS})

add_executable(sol_hello sol_hello.cpp)
target_link_libraries(sol_hello PUBLIC liblua ) # dl
target_include_directories(sol_hello PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries( sol_hello PUBLIC Threads::Threads)

add_executable(mosquitto_hello mosquitto_hello.cpp)
target_link_libraries(mosquitto_hello PUBLIC liblua ${mosquitto_LIBRARY})
target_include_directories(mosquitto_hello PUBLIC ${CMAKE_SOURCE_DIR}/third_party/sol ${mosquitto_INCLUDE_DIR})




#add_executable(ros_lua ros_lua.cpp)
#target_link_directories(ros_lua PUBLIC /home/waxz/dev/roscpp_src/install_isolated/lib )
#target_link_libraries(ros_lua PUBLIC
#        roscpp xmlrpcpp rosconsole cpp_common rostime roscpp_serialization
#        liblua dl
#        ${mosquitto_LIBRARY}
#        )
#target_include_directories(
#        ros_lua PUBLIC
#        /home/waxz/dev/roscpp_src/install_isolated/include
#        ${CMAKE_SOURCE_DIR}/third_party/sol
#        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
#        ${mosquitto_INCLUDE_DIR}
#
#)

#add_executable(ros_talker ros_talker.cpp)
#target_link_directories(ros_talker PUBLIC /home/waxz/dev/roscpp_src/install_isolated/lib )
#target_link_libraries(ros_talker PUBLIC
#        roscpp xmlrpcpp rosconsole cpp_common rostime roscpp_serialization
#        liblua dl
#        ${mosquitto_LIBRARY}
#        )
#target_include_directories(
#        ros_talker PUBLIC
#        /home/waxz/dev/roscpp_src/install_isolated/include
#        ${CMAKE_SOURCE_DIR}/third_party/sol
#        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
#        ${mosquitto_INCLUDE_DIR}
#
#)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")



#add_executable(ros_listener ros_listener.cpp)
#target_link_directories(ros_listener PUBLIC /home/waxz/dev/roscpp_src/install_isolated/lib )
#target_link_libraries(ros_listener PUBLIC
#        roscpp xmlrpcpp rosconsole cpp_common rostime roscpp_serialization
#        liblua dl
#        ${mosquitto_LIBRARY}
#        ${Boost_LIBRARIES}
#        ${GeographicLib_LIBRARIES}
#        )
#target_include_directories(
#        ros_listener PUBLIC
#        /home/waxz/dev/roscpp_src/install_isolated/include
#        ${CMAKE_SOURCE_DIR}/third_party/sol
#        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
#        ${mosquitto_INCLUDE_DIR}
#
#)
#install(CODE [[ set(__install_target $<TARGET_FILE:ros_listener>) ]]  )
#install_deps(ros_listener)

if(  DEFINED catkin_package)
add_executable(serial_hello  serial_hello.cpp)
target_link_libraries(serial_hello PUBLIC ${serial_LIBRARIES} )
target_include_directories(serial_hello PUBLIC ${serial_INCLUDE_DIRS})
endif()

#add_library(MqttClient message/MqttClient.cpp)
#target_include_directories(MqttClient PUBLIC ${CMAKE_SOURCE_DIR}/include ${mosquitto_INCLUDE_DIR}   ${CMAKE_SOURCE_DIR}/third_party/nlohmann)

add_executable(mqtt_cient_test  mqtt_cient_test.cpp message/impl/mqtt/MqttClient.cpp )
target_link_libraries(mqtt_cient_test  PUBLIC ${mosquitto_LIBRARY} ) # X11 pthread
target_include_directories(mqtt_cient_test PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}  ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include ${mosquitto_INCLUDE_DIR}   ${CMAKE_SOURCE_DIR}/third_party/nlohmann)
print_include_dir(mqtt_cient_test)

add_executable(system_command system_command.cpp md5/md5.cpp )
target_include_directories( system_command PUBLIC ${CMAKE_SOURCE_DIR}/include )
target_link_libraries(system_command PUBLIC  absl::strings)

add_executable(test_using_catch test_catch_main.cpp test/catch2/catch_amalgamated.cpp)

set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")

add_executable(template_test template_test.cpp
        )
target_include_directories(
        template_test PUBLIC
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann

)
set_asan(template_test)
set_property(TARGET template_test PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")


#add_executable(matplot_test matplot_test.cpp)
#target_link_libraries(matplot_test  PUBLIC Matplot++::matplot)





add_executable(immer_test.cpp immer_test.cpp )
target_include_directories(immer_test.cpp PUBLIC ${CMAKE_SOURCE_DIR}/third_party/immer-master ${CMAKE_SOURCE_DIR}/include )

add_executable(xml_json xml_json.cpp )
target_include_directories( xml_json PUBLIC ${CMAKE_SOURCE_DIR}/third_party/nlohmann)
target_link_libraries(xml_json PUBLIC tinyxml2::tinyxml2)

#add_executable(stl_test  stl_test.cpp)
#target_include_directories( stl_test PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann)
#target_link_libraries( stl_test PUBLIC Threads::Threads dl)
# Boost Package ############################################################
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

#find_package (Eigen3 REQUIRED NO_MODULE)

add_library(pcl_norm_2d icp/PCA2D.cpp icp/Normal2dEstimation.cpp)
target_include_directories( pcl_norm_2d PUBLIC  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )


add_executable(pcl_test pcl_test.cpp  )
target_include_directories( pcl_test PUBLIC  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )
target_link_libraries(pcl_test PUBLIC pcl_norm_2d Matplot++::matplot ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIBRARIES} )


set_omp(pcl_test)
add_executable(pcl_icp_test pcl_icp_test.cpp   icp/pcl_icp_with_normal.cpp )

target_include_directories( pcl_icp_test PUBLIC  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )
target_link_libraries(pcl_icp_test PUBLIC   Matplot++::matplot ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIBRARIES} )

if (BUILD_LIBPOINTMATCHER)
    message(Boost_LIBRARIES ${Boost_LIBRARIES})
    message(libnabo_LIBRARIES ${libnabo_LIBRARIES})
    add_executable(libpointmatcher_test  libpointmatcher_test.cpp)
    target_include_directories( libpointmatcher_test PUBLIC  ${libpointmatcher_INCLUDE_DIRS}  ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  )
    target_link_libraries(libpointmatcher_test PUBLIC ${Boost_LIBRARIES} ${libpointmatcher_LIBRARIES} Boost::filesystem )
endif ()

#target_link_directories(libpointmatcher_test PUBLIC  ${libnabo_ROOT}/lib)

#add_library(log_test_lib log_test_lib.cpp)
#target_include_directories( log_test_lib PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann)
#target_link_libraries( log_test_lib PUBLIC Threads::Threads dl)


#add_executable(log_test log_test.cpp)
#target_include_directories( log_test PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann)
#target_link_libraries( log_test PUBLIC Threads::Threads log_test_lib)



add_executable(taskflow_test taskflow_test.cpp)
target_include_directories( taskflow_test PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann)
target_link_libraries( taskflow_test PUBLIC Threads::Threads  Taskflow::Taskflow )


add_executable(std_test std_test.cpp)
target_include_directories( std_test PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann)
target_link_libraries( std_test PUBLIC Threads::Threads )

if (BUILD_OPTIM)
add_executable(optimlib_test optimlib_test.cpp)
target_include_directories(optimlib_test PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann )
target_link_libraries(optimlib_test PUBLIC liboptim)
print_include_dir(optimlib_test)
endif ()

add_executable(function_overload_test function_overload_test.cpp)

#add_executable(ifm3d_test  ifm3d_test.cpp)
#target_link_libraries(ifm3d_test PUBLIC
#        xmlrpc-c::xmlrpc_client++
#        ifm3d::camera
#        ifm3d::framegrabber
#        ifm3d::stlimage
#        ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
#        )
#target_include_directories( ifm3d_test PUBLIC
#        ${CMAKE_CURRENT_SOURCE_DIR}/ifm_tools/simpleimage/include
#        ${CMAKE_CURRENT_SOURCE_DIR}/ifm_tools/ppm-io-master/include
#        ${PCL_INCLUDE_DIRS}
#        ${CMAKE_SOURCE_DIR}/include
#        )

#target_link_directories(ifm3d_test PUBLIC  ${ifm3d_ROOT}/lib )


add_executable(eigen_test eigen_test.cpp)
target_include_directories(eigen_test PUBLIC ${EIGEN3_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann )
#target_link_libraries(eigen_test PUBLIC liboptim)
print_include_dir(eigen_test)

add_executable(json_test json_test.cpp)
target_include_directories(json_test PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann )

if (BUILD_PCL_QHULL)
add_executable(pcl_crop_to_hull pcl_crophull_segment_test.cpp)
target_include_directories( pcl_crop_to_hull PUBLIC  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )
target_link_libraries(pcl_crop_to_hull PUBLIC  Matplot++::matplot ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIBRARIES} )
endif ()

#add_executable(foonathan_memory_example foonathan_memory_example.cpp)
#target_include_directories( foonathan_memory_example PUBLIC  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )
#target_link_libraries(foonathan_memory_example PUBLIC   foonathan_memory Matplot++::matplot ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIBRARIES} )


#add_executable(likely_test likely_test.cpp)
#target_include_directories( likely_test PUBLIC  ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  )
#target_link_libraries(likely_test PUBLIC   foonathan_memory)


#add_executable(next_icp next_icp.cpp)
#target_include_directories( next_icp PUBLIC  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include   ${CMAKE_SOURCE_DIR}/src/ICP-Variants-main/icp-variants  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )
#target_link_libraries(next_icp PUBLIC  ${CERES_LIBRARIES}  Matplot++::matplot ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIBRARIES} )


add_subdirectory(tbb)

#add_subdirectory(gui)
#set(SANITIZE_ADDRESS ON)
#set(SANITIZE_MEMORY ON)
#set(SANITIZE_THREAD ON)
#set(SANITIZE_UNDEFINED ON)
#find_package(Sanitizers)
#
#add_executable(sanitizers_test sanitizers_test.cpp)
#add_sanitizers(sanitizers_test)
#set_asan(sanitizers_test)


#add_executable(pointer_address_test pointer_address_test.cpp)
#target_include_directories( pointer_address_test PUBLIC  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include   ${CMAKE_SOURCE_DIR}/src/ICP-Variants-main/icp-variants  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )
##set_asan(pointer_address_test)
#target_link_libraries( pointer_address_test PUBLIC Threads::Threads)

#add_executable(mock_test mock_test.cpp)
#target_include_directories( mock_test PUBLIC ${CMAKE_SOURCE_DIR}/third_party/Fakeit  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include   ${CMAKE_SOURCE_DIR}/src/ICP-Variants-main/icp-variants  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )


add_executable(boost_graph_test boost_graph_test.cpp)
target_link_libraries(boost_graph_test PUBLIC  ${Boost_LIBRARIES})
target_include_directories( boost_graph_test PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
        )
#add_library(tinyco SHARED
#        ${CMAKE_SOURCE_DIR}/include/tinyco-master/include/tinyco/context.cc
#        ${CMAKE_SOURCE_DIR}/include/tinyco-master/include/tinyco/coroutine.cc
#        ${CMAKE_SOURCE_DIR}/include/tinyco-master/include/tinyco/schedule.cc
#        )
#target_include_directories(tinyco PUBLIC ${CMAKE_SOURCE_DIR}/include/tinyco-master/include)
#
#add_executable(Observable_test Observable_test.cpp)
#target_link_libraries(Observable_test PUBLIC  ${Boost_LIBRARIES} libcopp::cotask tinyco)
#target_include_directories( Observable_test PUBLIC
#        ${EIGEN3_INCLUDE_DIR}
#        ${CMAKE_SOURCE_DIR}/include
#        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
#        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
#        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
#        ${CMAKE_SOURCE_DIR}/third_party/stackless_coroutine-master
#        )

add_executable(affinity_test affinity_test.cpp)
target_link_libraries(affinity_test PUBLIC  ${Boost_LIBRARIES} ${hwloc_LIBRARIES})
target_include_directories( affinity_test PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${hwloc_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
        )
target_link_libraries( affinity_test PUBLIC Threads::Threads
#        hwloc
        )
if(BUILD_BREAKPAD)
add_library(breakpad_uploader_library STATIC breakpad/http_upload.cc breakpad/google_crashdump_uploader.cc breakpad/libcurl_wrapper.cc)
target_include_directories(breakpad_uploader_library PUBLIC ${breakpad_include_dir}  ./breakpad)
target_link_libraries(breakpad_uploader_library PUBLIC ${breakpad_client_library} curl dl)
add_executable(breakpad_test breakpad_test.cpp breakpad_func.cpp  )
target_include_directories(breakpad_test PUBLIC ${breakpad_include_dir}  ./breakpad)
target_link_libraries(breakpad_test PUBLIC breakpad_uploader_library)
endif ()



add_executable(thread_test thread_test.cpp)
#add_executable(simd_hwy_test simd_hwy_test.cpp)
#target_link_libraries( simd_hwy_test PUBLIC  hwy::hwy hwy::hwy_contrib)



#find_library(USB_CAN_LIBRARY
#        NAMES usbcan
#        PATHS ${CMAKE_SOURCE_DIR}/third_party/zlg_usb_can/USBCAN_mini_Linux_Drivers/usbcan_libusb_x64_2018_10_15/test/
#        REQUIRED
#        )
#
#message(USB_CAN_LIBRARY ${USB_CAN_LIBRARY})
#add_library(usbcan SHARED IMPORTED GLOBAL)
#set_target_properties(usbcan PROPERTIES
#        IMPORTED_LOCATION_RELEASE ${USB_CAN_LIBRARY}
#        IMPORTED_LOCATION_DEBUG ${USB_CAN_LIBRARY}
#        )
#
#target_include_directories(usbcan INTERFACE
#        ${CMAKE_SOURCE_DIR}/third_party/zlg_usb_can/USBCAN_mini_Linux_Drivers/usbcan_libusb_x64_2018_10_15/test
#        )
#add_executable(zlg_can_test zlg_can_test.cpp)
#target_link_libraries(zlg_can_test PUBLIC ${USB_CAN_LIBRARY} )
#target_include_directories(zlg_can_test PUBLIC
#        ${CMAKE_SOURCE_DIR}/third_party/zlg_usb_can/USBCAN_mini_Linux_Drivers/usbcan_libusb_x64_2018_10_15/test
#        )

add_executable(can_test can_test.cpp)
target_link_libraries(can_test PUBLIC
        ${canfestival_library}
        pthread rt dl
        )
target_include_directories(can_test PUBLIC
        ${canfestival_include_dir}
        )
#set_asan(can_test  )

#set_property(TARGET can_test PROPERTY POSITION_INDEPENDENT_CODE ON)
add_executable(lamda_function_test lamda_function_test.cpp)
target_include_directories( lamda_function_test PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
        ${CMAKE_SOURCE_DIR}/third_party/stackless_coroutine-master
        )

add_executable(task_schedual_test task_schedual_test.cpp)
target_include_directories( task_schedual_test PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
        ${CMAKE_SOURCE_DIR}/third_party/stackless_coroutine-master
        )

add_executable(endian_align_test endian_align_test.cpp)
target_include_directories( endian_align_test PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
        ${CMAKE_SOURCE_DIR}/third_party/stackless_coroutine-master
        )

add_library(libcatch2 SHARED ${CMAKE_SOURCE_DIR}/include/catch2/catch_amalgamated.cpp)
target_include_directories( libcatch2 PUBLIC
        ${CMAKE_SOURCE_DIR}/include/catch2
        )
add_executable(static_assert_test static_assert_test.cpp
        )
target_include_directories( static_assert_test PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/catch2

#        ${CMAKE_SOURCE_DIR}/include/fakeit/include
#        ${CMAKE_SOURCE_DIR}/include/fakeit/config/catch
        ${CMAKE_SOURCE_DIR}/include/fakeit/single_header/catch
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
        )
target_link_libraries(static_assert_test PUBLIC  libcatch2)

add_executable(lua_test lua_test.cpp)
target_include_directories( lua_test PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/catch2

        #        ${CMAKE_SOURCE_DIR}/include/fakeit/include
        #        ${CMAKE_SOURCE_DIR}/include/fakeit/config/catch
        ${CMAKE_SOURCE_DIR}/include/fakeit/single_header/catch
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
        )
target_link_libraries(lua_test PUBLIC  libcatch2)

target_link_libraries(lua_test PUBLIC liblua ) # dl
target_include_directories(lua_test PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries( lua_test PUBLIC Threads::Threads)


#add_executable(argparse_test argparse_test.cpp)
#target_include_directories( argparse_test PUBLIC
#        ${EIGEN3_INCLUDE_DIR}
#        ${CMAKE_SOURCE_DIR}/include
#        ${CMAKE_SOURCE_DIR}/include/catch2
#
#        #        ${CMAKE_SOURCE_DIR}/include/fakeit/include
#        #        ${CMAKE_SOURCE_DIR}/include/fakeit/config/catch
#        ${CMAKE_SOURCE_DIR}/include/fakeit/single_header/catch
#        ${CMAKE_SOURCE_DIR}/third_party/nlohmann
#        ${CMAKE_SOURCE_DIR}/third_party/observable-master/observable/include
#        ${CMAKE_SOURCE_DIR}/third_party/coroutine-master
#        )

#add_subdirectory(root_test)

add_executable(clock_test clock_test.cpp)
target_include_directories(clock_test PUBLIC ${CMAKE_SOURCE_DIR}/include)

pybind11_add_module(cmake_example py11_test.cpp)

add_executable(absl_test absl_test.cpp )
target_include_directories( absl_test PUBLIC
        ${CMAKE_SOURCE_DIR}/include
#        ${CMAKE_SOURCE_DIR}/include/fakeit/single_header/catch
        ${FRUIT_INCLUDE_DIR}
        )

target_link_libraries(absl_test PUBLIC
        libcatch2
        doctest::doctest
        nanobench::nanobench
        absl::strings  absl::str_format
        absl::log
#        absl::log_internal_check_impl
        absl::log_internal_check_op
        absl::statusor
        absl::debugging
        absl::random_random
        absl::any

        ${FRUIT_LIBRARY}
)



#https://abseil.io/docs/cpp/platforms/compilerflags#gcc-flags
SET_GCC_FLAGS(absl_test)
set_asan(absl_test)

add_executable(transform_test  transform_test.cpp)
target_include_directories( transform_test PUBLIC ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(transform_test PUBLIC libcatch2 ) #range-v3::range-v3
set_omp(transform_test)



add_executable(math_test  math_test.cpp)
target_include_directories( math_test PUBLIC ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(c_test)
add_subdirectory(dds)
add_subdirectory(timer)

add_subdirectory(network)

add_subdirectory(smart_ptr)

add_subdirectory(thread_openmp_tbb)

add_subdirectory(simulate)

add_subdirectory(std)

add_subdirectory(profiler_test)

add_subdirectory(tcc)
add_subdirectory(simd)

add_subdirectory(boost_test)

add_subdirectory(ceres_test)