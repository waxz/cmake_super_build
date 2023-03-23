# cmake super build

This project demonstrates how to use cmake to compile various open source libraries. In addition to this, sample code is written as an introductory tutorial.

##### support by [JetBrains Open Source Development - Community Support](https://www.jetbrains.com/community/opensource/#support)
<img src="https://resources.jetbrains.com/storage/products/company/brand/logos/jb_beam.png" alt="JetBrains Logo (Main) logo." width="100">
<img src="https://resources.jetbrains.com/storage/products/company/brand/logos/CLion_icon.png" alt="JetBrains Logo CLion logo." width="100">

### cmake tips
1. set c++ standard, linker flag, cmake variable
```cmake
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++... instead of -std=gnu++...
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic -Wno-dev -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")
```
2. set asan flags
```cmake
function(set_asan target)
    # Treat all warnings as errors
    #    target_compile_options(${target} PRIVATE "-Werror")
    target_compile_options(${target} PUBLIC "-fsanitize=undefined")
    target_compile_options(${target} PUBLIC "-fno-builtin-malloc")
    target_compile_options(${target} PUBLIC "-fno-builtin-calloc")
    target_compile_options(${target} PUBLIC "-fno-builtin-realloc")
    target_compile_options(${target} PUBLIC "-fno-builtin-free")


    #    target_compile_options(${target} PUBLIC "-fsanitize-address-use-after-scope")

    target_compile_options(${target} PUBLIC "-fsanitize=address")
    target_compile_options(${target} PUBLIC "-fno-optimize-sibling-calls")


    target_compile_options(${target} PUBLIC "-fsanitize=leak")
    target_compile_options(${target} PUBLIC "-fno-omit-frame-pointer")

    target_compile_options(${target} PUBLIC "-fstack-protector")
    target_link_libraries(${target} PUBLIC  "-fuse-ld=gold")

    target_link_libraries(${target} PUBLIC "-fsanitize=address  -fsanitize=leak -fsanitize=undefined")
    #    target_compile_options(${target} PUBLIC "-fsanitize=memory")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=memory")
    #    target_compile_options(${target} PUBLIC "-fsanitize=thread")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=thread")

endfunction(set_asan)
```
3. set rpath , runpath
```cmake
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")



macro(set_target_rpath __Target)
    set (extra_args ${ARGN})

    set_target_properties(${__Target} PROPERTIES LINK_FLAGS "-Wl,-rpath,.,-disable-new-dtags")  # set RPATH ok ok
    set_property(
            TARGET ${__Target}
            PROPERTY BUILD_RPATH
            "${CMAKE_BINARY_DIR}/lib"
            "$ORIGIN/../lib"
            ${extra_args}

    )
    set_property(
            TARGET ${__Target}
            PROPERTY INSTALL_RPATH
            "${CMAKE_INSTALL_PREFIX}/lib"
            "$ORIGIN/../lib"
            ${extra_args}

    )
endmacro(set_target_rpath)

macro(set_target_runpath __Target)
    set (extra_args ${ARGN})

    set_target_properties(${__Target} PROPERTIES LINK_FLAGS "-Wl,-rpath,.,-enable-new-dtags")  # set RPATH ok ok
    set_property(
            TARGET ${__Target}
            PROPERTY BUILD_RPATH
            PROPERTY BUILD_RPATH
            "${CMAKE_BINARY_DIR}/lib"
            "$ORIGIN/../lib"
            ${extra_args}

    )
    set_property(
            TARGET ${__Target}
            PROPERTY INSTALL_RPATH
            "${CMAKE_INSTALL_PREFIX}/lib"
            "$ORIGIN/../lib"
            ${extra_args}

    )
endmacro(set_target_runpath)

```
4.build_external_project
```cmake

include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)

build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.1.0.tar.gz  -DMINIGLOG=ON -DGFLAGS=OFF )
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )


add_executable(cmake_super_build main.cpp)
target_link_libraries(cmake_super_build PUBLIC
        ${CERES_LIBRARIES}
        )
set_asan(cmake_super_build)
```

5. ros environment setup

set Build-Execution-Deployment, cmake, environment
```sh
LD_LIBRARY_PATH=/opt/ros/noetic/lib;ROS_HOME=/tmp
```
if you build and run code in docker, setup environment in docker container settings
```sh
LD_LIBRARY_PATH=/opt/ros/noetic/lib;ROS_HOME=/tmp;ROS_MASTER_URI=http://172.20.0.1:11311
```
in `CMakeLists.txt`
```
add_executable(ros_init_node ros_init_node.cpp)
add_ros(ros_init_node roscpp)
```




### libraries
- https://github.com/oneapi-src/oneTBB
- https://github.com/ceres-solver/ceres-solver
- https://github.com/PointCloudLibrary/pcl
- https://github.com/TartanLlama/optional
- https://github.com/gulrak/filesystem
- https://abseil.io/about/intro
- https://github.com/eProsima/Fast-DDS
- https://github.com/eclipse-cyclonedds/cyclonedds-cxx
- https://github.com/arsenm/sanitizers-cmake
