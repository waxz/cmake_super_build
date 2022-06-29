include(FetchContent)
include(ExternalProject)
# This function is used to force a build on a dependant project at cmake configuration phase.
#
function (build_external_project target prefix url) #FOLLOWING ARGUMENTS are the CMAKE_ARGS of ExternalProject_Add
    include(ProcessorCount)
    ProcessorCount(N)

    set(trigger_build_dir ${CMAKE_BINARY_DIR}/force_${target})

    #mktemp dir in build tree
    file(MAKE_DIRECTORY ${trigger_build_dir} ${trigger_build_dir}/build)

    #generate false dependency project
    set(CMAKE_LIST_CONTENT "
        cmake_minimum_required(VERSION 3.0)
        project(${target}_build)
        include(ExternalProject)

        ExternalProject_add(${target}
            PREFIX ${prefix}/${target}
            URL ${url}
            CMAKE_ARGS ${ARGN} -DCMAKE_INSTALL_PREFIX=${trigger_build_dir}/install
            #INSTALL_COMMAND \"\"
            INSTALL_COMMAND ${CMAKE_COMMAND} --install .
            )

        add_custom_target(trigger_${target})
        add_dependencies(trigger_${target} ${target})
    ")

    file(WRITE ${trigger_build_dir}/CMakeLists.txt "${CMAKE_LIST_CONTENT}")

    execute_process(COMMAND ${CMAKE_COMMAND} -G${CMAKE_GENERATOR} ..
            WORKING_DIRECTORY ${trigger_build_dir}/build
            )
    execute_process(COMMAND ${CMAKE_COMMAND} --build . -j${N}
            WORKING_DIRECTORY ${trigger_build_dir}/build
            )
    set(${target}_ROOT ${trigger_build_dir}/install PARENT_SCOPE)

    #    execute_process(COMMAND ${CMAKE_COMMAND} --install ./build --prefix ${trigger_build_dir}/install
    #            WORKING_DIRECTORY ${trigger_build_dir}
    #            )
endfunction()

