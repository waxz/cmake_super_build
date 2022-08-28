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

add_executable(ros_init_node ros_init_node.cpp)
target_link_directories(ros_init_node PUBLIC /home/waxz/dev/roscpp_src/install_isolated/lib )
target_link_libraries(ros_init_node PUBLIC
        roscpp
        )
target_include_directories(
        ros_init_node PUBLIC
        /home/waxz/dev/roscpp_src/install_isolated/include
)

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

