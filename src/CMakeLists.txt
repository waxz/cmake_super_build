# This file handles building each task as sub-project.
#
# Each task becomes an independent project in which you can experiment.
#
# Tasks are added as separate sub-projects so as compilation errors in
# one task don't affect another task.


project(cmake_super_build)

set(CMAKE_CXX_STANDARD 14)

add_executable(cv_process_mat cv_process_mat.cpp)
target_link_libraries(cv_process_mat PUBLIC
        ${OpenCV_LIBS}
        )

target_include_directories(cv_process_mat PUBLIC
        ${OpenCV_INCLUDE_DIRS})


add_executable(ceres_simple_solve ceres_simple_solve.cpp)

target_link_libraries(ceres_simple_solve PUBLIC
        ${CERES_LIBRARIES}
        )
install(TARGETS ceres_simple_solve DESTINATION bin)

find_package(Boost 1.58 REQUIRED COMPONENTS system filesystem)



#add_executable(ros_init_node ros_init_node.cpp)
#target_link_directories(ros_init_node PUBLIC /home/waxz/dev/roscpp_src/install_isolated/lib )
#target_link_libraries(ros_init_node PUBLIC
#        roscpp xmlrpcpp rosconsole cpp_common rostime roscpp_serialization
#        ${Boost_LIBRARIES}
#
#        )
#target_include_directories(
#        ros_init_node PUBLIC
#        /home/waxz/dev/roscpp_src/install_isolated/include
#)



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

add_executable(sol_hello2 sol_hello.cpp)
target_link_libraries(sol_hello2 PUBLIC liblua ) # dl
target_include_directories(sol_hello2 PUBLIC ${CMAKE_SOURCE_DIR}/include)

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


add_executable(serial_hello  serial_hello.cpp)
target_link_libraries(serial_hello PUBLIC ${serial_LIBRARIES} )
target_include_directories(serial_hello PUBLIC ${serial_INCLUDE_DIRS})


#add_library(MqttClient message/MqttClient.cpp)
#target_include_directories(MqttClient PUBLIC ${CMAKE_SOURCE_DIR}/include ${mosquitto_INCLUDE_DIR}   ${CMAKE_SOURCE_DIR}/third_party/nlohmann)

add_executable(mqtt_cient_test  mqtt_cient_test.cpp message/MqttClient.cpp )
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
