cmake_minimum_required(VERSION 3.16)
project(cmake_super_build)

include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)
include(cmake/find_opencv_helper.cmake)
set(CMAKE_CXX_STANDARD  14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/sanitizers")


# 设置生成的安装包名字
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
# 设置支持指定安装目录的控制为 ON
set(CPACK_SET_DESTDIR ON)
# 设置安装到的目录路径
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
# 这是生成的安装的版本号信息
set(CPACK_PACKAGE_VERSION "1.0.0")
# 设置 group 名字
set(CPACK_RPM_PACKAGE_GROUP "vesoft")
# 设置 vendor 名字
set(CPACK_PACKAGE_VENDOR "vesoft")
# 设置 license 信息
set(CPACK_RPM_PACKAGE_LICENSE "Apache 2.0 + Common Clause 1.0")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Yan QiDong")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "This is a demo package from Yan QiDong.")
set(CPACK_DEBIAN_PACKAGE_VERSION ${PACKAGE_VERSION})
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.1-6), libc6 (<< 3)")



# https://cyclonedds.io/
# https://github.com/eclipse-iceoryx/iceoryx/releases/tag/v2.0.3
#iceoryx-2.0.3.zip
#https://man7.org/tlpi/code/faq.html#sys-acl.h-missing
#sudo apt-get install libacl1-dev
build_external_project_command(iceoryx lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/iceoryx-2.0.3.zip  "cmake -Bbuild -Hiceoryx_meta -DBUILD_DOC=OFF -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/force_iceoryx/install -DCMAKE_BUILD_TYPE=RelWithDebInfo" "cmake --build build" " cmake --build build --target install" )
set(iceoryx_binding_c_ROOT ${iceoryx_ROOT})
find_package(iceoryx_binding_c REQUIRED)

#https://github.com/eclipse-cyclonedds/cyclonedds/releases/tag/0.10.4
#cyclonedds-0.10.2.zip
build_external_project(CycloneDDS lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cyclonedds-0.10.4.tar.gz -Diceoryx_binding_c_ROOT=${iceoryx_binding_c_ROOT} -DCMAKE_BUILD_TYPE=Debug -DENABLE_SHM=YES)
find_package(CycloneDDS REQUIRED)

# https://github.com/eclipse-cyclonedds/cyclonedds-cxx/releases/tag/0.10.4
#cyclonedds-cxx-0.10.2.zip
#
build_external_project(CycloneDDS-CXX lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cyclonedds-cxx-0.10.4.tar.gz -DCycloneDDS_ROOT=${CycloneDDS_ROOT}  -Diceoryx_binding_c_ROOT=${iceoryx_binding_c_ROOT}  -DCMAKE_BUILD_TYPE=Debug -DENABLE_SHM=YES)
find_package(CycloneDDS-CXX REQUIRED)

#https://github.com/TinyCC/tinycc
# --with-libgcc
build_external_project_command(tinycc lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinycc-mob-20240330.zip "./configure --prefix=${CMAKE_BINARY_DIR}/force_tinycc/install  --enable-static" "make -j6" " make install" )
#build_external_project_command(tinycc lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinycc-mob_2023_1220.zip "./configure --prefix=${CMAKE_BINARY_DIR}/force_tinycc/install --enable-static --debug --extra-cflags='-std=c11 -Wpedantic -fsanitize=address,undefined -g' " "make -j6" " make install" )

set(tinycc_COMPILER_DIR ${tinycc_ROOT}/lib/tcc)
set(tinycc_LIBTCC_INCLUDE_DIR ${tinycc_ROOT}/include)
set(tinycc_TCCLIB_INCLUDE_DIR ${tinycc_ROOT}/lib/tcc/include)
set(tinycc_LIB ${tinycc_ROOT}/lib/libtcc.a)


#strliteral-master.zip
build_external_project_command(strliteral lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/strliteral-master.zip "mkdir -p ${CMAKE_BINARY_DIR}/force_strliteral/install/bin" "make" "cp ./strliteral ${CMAKE_BINARY_DIR}/force_strliteral/install/bin" )
set(strliteral_exe ${strliteral_ROOT}/bin/strliteral)
macro(strliteral_build in out)
    message("strliteral_build strliteral_exe: " ${strliteral_exe})
    message("strliteral_build WORKING_DIRECTORY: " ${CMAKE_CURRENT_BINARY_DIR})
    message("strliteral_build in: " ${in})
    message("strliteral_build out: " ${out})

    execute_process(COMMAND ${strliteral_exe} ${in} ${out}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endmacro()

# Eigen3
build_external_project(Eigen3 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0.zip  -DCMAKE_BUILD_TYPE=Release )
message(Eigen3_ROOT : ${Eigen3_ROOT} )

set(Eigen3_ROOT ${Eigen3_ROOT}/share/eigen3/cmake)
find_package(Eigen3 3.4.0 REQUIRED HINTS ${Eigen3_ROOT})
message(new eigen: EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS} )

# ceres-solver-2.2.0.tar.gz
build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.2.0.tar.gz  -DEigen3_ROOT=${Eigen3_ROOT} -DCMAKE_BUILD_TYPE=Release -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCUMENTATION=OFF)
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )
#find_opencv(/home/waxz/dev/docker/docker_dev/share/opencv/install 4.6.0)

build_external_project(Fruit lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/fruit-3.7.1.tar.gz  -DFRUIT_USES_BOOST=False -DCMAKE_BUILD_TYPE=Release)
set(FRUIT_INSTALLED_DIR ${Fruit_ROOT})
find_package(Fruit  REQUIRED)
message(Fruit_ROOT : ${Fruit_ROOT} , FRUIT_LIBRARY : ${FRUIT_LIBRARY}, FRUIT_INCLUDE_DIR : ${FRUIT_INCLUDE_DIR} )
set(FRUIT_INCLUDE_DIR ${FRUIT_INCLUDE_DIR}/../)

string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(Red         "${Esc}[31m")

find_program(FLATBUFFERS_COMPILER flatc HINTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bin)
if (NOT FLATBUFFERS_COMPILER)
    message(FATAL_ERROR "${Red}Flatc not found${ColourReset}")
endif()

build_external_project(ZeroMQ lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/zeromq-4.3.4.tar.gz -DCMAKE_BUILD_TYPE=Release)

find_package(ZeroMQ  REQUIRED)

message(ZeroMQ_ROOT : ${ZeroMQ_ROOT} ,ZeroMQ_STATIC_LIBRARY : ${ZeroMQ_STATIC_LIBRARY}, ZeroMQ_LIBRARY : ${ZeroMQ_LIBRARY}, ZeroMQ_INCLUDE_DIR : ${ZeroMQ_INCLUDE_DIR} )
find_package(ZeroMQ REQUIRED)

# add cmake_policy(SET CMP0074 NEW) in cppzmq/CMakeLists.txt
build_external_project(cppzmq lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cppzmq-4.8.0.tar.gz -DCMAKE_BUILD_TYPE=Release -DCPPZMQ_BUILD_TESTS=OFF -DZeroMQ_ROOT=${ZeroMQ_ROOT} -DZeroMQ_DIR=${ZeroMQ_ROOT})

#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq REQUIRED)
message(cppzmq_DIR : ${cppzmq_DIR} , cppzmq_INCLUDE_DIR : ${cppzmq_INCLUDE_DIR})


# TBB
build_external_project(TBB lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oneTBB-2021.8.0.tar.gz -DCMAKE_BUILD_TYPE=Release -DTBB_TEST=OFF -DTBB_USE_DEBUG=0)
find_package(TBB REQUIRED)

# flatbuffers FlatBuffersConfig.cmake
build_external_project(FLATBUFFERS lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/flatbuffers-2.0.7.tar.gz -DCMAKE_BUILD_TYPE=Release)
set(FlatBuffers_ROOT ${FLATBUFFERS_ROOT})
find_package(FlatBuffers REQUIRED)


include(CMakeDependentOption)
include(CMakePackageConfigHelpers)
include(ExternalProject)

if (BUILD_LUA)

set(LUA_VERSION "5.3.5" CACHE STRING "The version of Lua needed. Can be 5.1, 5.2, 5.3, LuaJIT, or a more specific 3-part version number for a specifc Lua (e.g., 5.3.4 or luajit-2.0.5)")
string(TOLOWER ${LUA_VERSION} NORMALIZED_LUA_VERSION)
#    find_package(LuaBuild REQUIRED COMPONENTS ${LUA_VERSION})
build_external_project_make(lua lib "https://www.lua.org/ftp/lua-${LUA_VERSION}.tar.gz")
message(lua_ROOT : ${lua_ROOT})
set(LUA_LIBRARIES ${lua_ROOT}/lib/liblua.a)
set(LUA_INCLUDE_DIR ${lua_ROOT}/include)
endif ()


#ExternalProject_Add(lua
#        URL "https://www.lua.org/ftp/lua-${LUA_VERSION}.tar.gz"
#        CONFIGURE_COMMAND ""
#        BUILD_COMMAND make generic
#        BUILD_ALWAYS true
#        BUILD_IN_SOURCE true
#        INSTALL_COMMAND ""
#        )
#ExternalProject_Get_property(lua SOURCE_DIR)
#message("liblua will be found at '${SOURCE_DIR}/src'")
#set(LUA_LIBRARIES ${SOURCE_DIR}/src/liblua.a)
#
#
#

#build_external_project(CJSON lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cJSON-1.7.15.tar.gz -DCMAKE_BUILD_TYPE=Release)
#set(cJSON_DIR ${CJSON_ROOT}/lib/cmake/cJSON)
#find_package(cJSON REQUIRED)

build_external_project(mosquitto lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mosquitto-2.0.15.tar.gz -DCMAKE_BUILD_TYPE=Release  -DDOCUMENTATION=OFF) #-DcJSON_DIR=${cJSON_DIR}
set(mosquitto_LIBRARY ${mosquitto_ROOT}/lib/libmosquitto.so ${mosquitto_ROOT}/lib/libmosquittopp.so)
set(mosquitto_INCLUDE_DIR ${mosquitto_ROOT}/include)



build_external_project(GeographicLib lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/GeographicLib-2.1.1.tar.gz -DCMAKE_BUILD_TYPE=Release )
find_package(GeographicLib REQUIRED)

if(  DEFINED catkin_package)
# serial-1.2.1.tar.gz
build_external_project(serial lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/serial-1.2.1.tar.gz -DCMAKE_BUILD_TYPE=Release )
find_package(serial REQUIRED)
endif()
#===============

#abseil-cpp-20220623.1.tar.gz
#build_external_project(absl lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/abseil-cpp-20230125.3.tar.gz -DABSL_BUILD_TESTING=OFF -DABSL_USE_GOOGLETEST_HEAD=ON -DCMAKE_CXX_STANDARD=14  -DABSL_ENABLE_INSTALL=ON -DBUILD_SHARED_LIBS=OFF)
build_external_project(absl lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/abseil-cpp-20230802.1.tar.gz -DABSL_BUILD_TESTING=OFF -DABSL_USE_GOOGLETEST_HEAD=ON -DCMAKE_CXX_STANDARD=14  -DABSL_ENABLE_INSTALL=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON)

find_package(absl REQUIRED)


#double-conversion-3.3.0.tar.gz
build_external_project(double-conversion lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/double-conversion-3.3.0.tar.gz -DCMAKE_CXX_STANDARD=14 -DBUILD_SHARED_LIBS=OFF)
find_package(double-conversion REQUIRED)

#folly-v2023.08.28.00.tar.gz
#build_external_project(folly lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/folly-v2023.08.28.00.tar.gz -DCMAKE_CXX_STANDARD=14 -DBUILD_SHARED_LIBS=OFF -Ddouble-conversion_ROOT=${double-conversion_ROOT})
#find_package(folly REQUIRED)

#build_external_project(flashlight lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/flashlight-0.3.2.tar.gz -DCMAKE_BUILD_TYPE=Release )
#find_package(flashlight REQUIRED)

#set(Open3D_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-pre-cxx11-abi-0.16.1+9ca1c11)

#Open3D-0.15.1.tar.gz
#build_external_project(Open3D lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Open3D-0.15.1.tar.gz -DBUILD_FILAMENT_FROM_SOURCE=OFF -DGLIBCXX_USE_CXX11_ABI=OFF -DBUILD_GUI=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_CACHED_CUDA_MANAGER=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_MODULE=OFF )
#
#find_package(Open3D REQUIRED)

#matplotplusplus-1.1.0.tar.gz
build_external_project(Matplot++ lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/matplotplusplus-1.1.0.tar.gz  -DBUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON)
find_package(Matplot++ REQUIRED)

# range-v3-0.12.0.tar.gz
#build_external_project(range-v3 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/range-v3-0.12.0.tar.gz   -DCMAKE_BUILD_TYPE=Release  )
#find_package(range-v3 REQUIRED)




#include(FetchContent)
#
#FetchContent_Declare(
#        Boost
#        URL https://github.com/boostorg/boost/releases/download/boost-1.81.0/boost-1.82.0.tar.xz
##        URL_MD5 6cf0cdd797bca685910d527ae3c08cb3
##        DOWNLOAD_EXTRACT_TIMESTAMP ON
#)
#
#set(BOOST_INCLUDE_LIBRARIES timer filesystem regex)
#FetchContent_MakeAvailable(Boost)

#build_external_project(mv-lm-icp lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mv-lm-icp-master-all.zip  -DCMAKE_BUILD_TYPE=Release  )
#find_package(mv-lm-icp REQUIRED)

#set(BOOST_INCLUDE_LIBRARIES "filesystem\;date_time\;iostreams\;system")

#set(BOOST_INCLUDE_LIBRARIES "timer;filesystem;regex;date_time")
#list(APPEND BOOST_INCLUDE_LIBRARIES filesystem)
#list(APPEND BOOST_INCLUDE_LIBRARIES date_time)
#list(APPEND BOOST_INCLUDE_LIBRARIES iostreams)
#list(APPEND BOOST_INCLUDE_LIBRARIES system)

#message("BOOST_INCLUDE_LIBRARIES = ${BOOST_INCLUDE_LIBRARIES}")
#foreach(__boost_included_lib IN LISTS BOOST_INCLUDE_LIBRARIES)
#
#    message("__boost_included_lib = ${__boost_included_lib}")
#
#endforeach()

#https://stackoverflow.com/questions/33242956/cmake-passing-lists-on-command-line
build_external_project(Boost lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost-1.82.0.tar.xz -DCMAKE_BUILD_TYPE=Release '-DBOOST_INCLUDE_LIBRARIES=filesystem;date_time;iostreams;system;program_options' -DBUILD_SHARED_LIBS=ON )

#find_package(boost_timer REQUIRED)
#find_package(boost_filesystem REQUIRED)
set(BOOST_REQUIRED_MODULES filesystem date_time iostreams system program_options)

find_package(Boost 1.82.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})

#flann-1.9.1.zip
build_external_project(flann lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/flann-master.zip  -DCMAKE_BUILD_TYPE=Release -DBUILD_C_BINDINGS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF  )

find_package(flann REQUIRED )

build_external_project(PCL lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pcl-pcl-1.13.0.tar.gz -DBOOST_ROOT=${Boost_ROOT} -DFLANN_ROOT=${flann_ROOT} -DWITH_VTK=OFF -DWITH_PCAP=OFF -DWITH_OPENGL=OFF -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIRS} -DCMAKE_BUILD_TYPE=Release  -DBUILD_examples=OFF -DBUILD_apps=OFF -DBUILD_global_tests=OFF -DBUILD_simulation=OFF -DBUILD_tools=OFF)
find_package(PCL 1.13 REQUIRED )
message("PCL_LIBRARIES = " ${PCL_LIBRARIES})

#ifm3d-0.93.0.zip
if (BUILD_IFM3D)
    build_external_project(ifm3d lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ifm3d-0.93.0.zip -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  -DPCL_ROOT=${PCL_ROOT} -DBUILD_TESTS=OFF)

    set(glog_ROOT ${ifm3d_ROOT}/lib/cmake/glog )

    find_package(glog REQUIRED)
    set(CURL_ROOT ${ifm3d_ROOT}/lib/cmake/CURL )

    find_package(CURL REQUIRED)


    set(xmlrpc-c_ROOT ${ifm3d_ROOT}/lib/cmake/xmlrpc-c )
    find_package(
            xmlrpc-c CONFIG
            REQUIRED COMPONENTS xmlrpc xmlrpc_client xmlrpc++ xmlrpc_client++
    )


    find_package(ifm3d REQUIRED COMPONENTS
            camera
            framegrabber
            stlimage
            )
endif ()

if(BUILD_LIBPOINTMATCHER)
    #octomap-1.9.6.tar.gz
    build_external_project(octomap lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/octomap-1.9.6.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DBUILD_OCTOVIS_SUBPROJECT=OFF -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF)
    find_package(octomap REQUIRED)

    # libnabo-master.zip
    build_external_project(libnabo lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libnabo-master.zip -DBOOST_ROOT=${Boost_ROOT} -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIRS} -DFLANN_ROOT=${flann_ROOT}  -DCMAKE_BUILD_TYPE=Release -DBUILD_TYPE=cmake -Dcatkin_DIR=/opt/ros/noetic/share/catkin/cmake )
    find_package(libnabo REQUIRED)

    # libpointmatcher-master.zip -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIRS}
    build_external_project(libpointmatcher lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libpointmatcher-master.zip -DBUILD_TYPE=cmake -DBOOST_ROOT=${Boost_ROOT} -DCMAKE_BUILD_TYPE=Release -Dlibnabo_ROOT=${libnabo_ROOT} -DSHARED_LIBS=OFF )

    find_package(libpointmatcher REQUIRED)
endif ()


# taskflow-3.4.0.tar.gz
build_external_project(Taskflow lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/taskflow-3.4.0.tar.gz -DBUILD_TYPE=cmake -DCMAKE_BUILD_TYPE=Release -DTF_BUILD_EXAMPLES=OFF -DTF_BUILD_TESTS=OFF -DTF_BUILD_PROFILER=OFF -DTF_BUILD_BENCHMARKS=OFF  )

find_package(Taskflow REQUIRED)


#autodiff-1.0.3.tar.gz
build_external_project(autodiff lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/autodiff-1.0.3.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_PYTHON=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF -DAUTODIFF_BUILD_DOCS=OFF)
find_package(autodiff REQUIRED)
if (BUILD_OPTIM)
    build_external_project_command(optim lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/optim.zip "EIGEN_INCLUDE_PATH=${EIGEN3_INCLUDE_DIRS} ./configure -i ./install -l eigen -p" "make && mkdir -p ./install/lib" "make install"  -DCMAKE_BUILD_TYPE=Release  -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_PYTHON=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF -DAUTODIFF_BUILD_DOCS=OFF)
    #find_package(optim REQUIRED)
    message(optim_BUILD_DIR ${optim_BUILD_DIR})
    set(optim_ROOT ${optim_BUILD_DIR}/optim/install)
    set(optim_INCLUDE_DIR ${optim_ROOT}/include/optim)
    set(optim_LIBRARY ${optim_ROOT}/lib/liboptim.so)

    message(optim_INCLUDE_DIR ${optim_INCLUDE_DIR})
    message(optim_LIBRARY ${optim_LIBRARY})

    add_library(liboptim SHARED IMPORTED GLOBAL)
    set_target_properties(liboptim PROPERTIES
            IMPORTED_LOCATION_RELEASE ${optim_LIBRARY}
            IMPORTED_LOCATION_DEBUG ${optim_LIBRARY}
            )

    set_property(TARGET liboptim PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${optim_INCLUDE_DIR}  ${EIGEN3_INCLUDE_DIRS})
    set_property(TARGET liboptim PROPERTY INTERFACE_LINK_LIBRARIES autodiff::autodiff )


    print_include_dir(liboptim)
endif ()


# https://github.com/openresty/luajit2
build_external_project_command(luajit lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/luajit2-2.1-20230410.tar.gz "mkdir -p ${CMAKE_BINARY_DIR}/force_luajit/install/bin" "make PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install" "make install PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install")
set(LUA_LIBRARIES ${luajit_ROOT}/lib/libluajit-5.1.a)
set(LUA_INCLUDE_DIR ${luajit_ROOT}/include/luajit-2.1)

add_library(liblua STATIC IMPORTED)
set_target_properties(liblua PROPERTIES  IMPORTED_LOCATION "${LUA_LIBRARIES}")
target_include_directories(liblua INTERFACE ${LUA_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/third_party/sol)
target_link_libraries(liblua INTERFACE  dl )

# breakpad-2023.01.27-full.zip
# https://github.com/google/breakpad/blob/master/docs/linux_starter_guide.md#producing-symbols-for-your-application
# https://github.com/google/breakpad.git
# https://chromium.googlesource.com/linux-syscall-support/+/refs/tags/v2022.10.12
# copy linux-syscall-support/linux_syscall_support.h to breakpad/src/third_party/lss
build_external_project_command(breakpad lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/breakpad-2023.01.27-full.zip "mkdir -p ${CMAKE_BINARY_DIR}/force_breakpad/install/bin && ./configure --prefix=${CMAKE_BINARY_DIR}/force_breakpad/install" "make" "make install")
message(STATUS "breakpad_ROOT ${breakpad_ROOT}")

set(breakpad_client_library ${breakpad_ROOT}/lib/libbreakpad_client.a)
set(breakpad_library ${breakpad_ROOT}/lib/libbreakpad.a)
set(breakpad_include_dir ${breakpad_ROOT}/include/breakpad)
message(STATUS "breakpad_client_library ${breakpad_client_library}")
message(STATUS "breakpad_library ${breakpad_library}")


build_external_project(tinyxml2 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2-9.0.0.tar.gz  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF )
find_package(tinyxml2 REQUIRED)


# https://github.com/ToruNiina/toml11/releases
# toml11-3.7.1.tar.gz
build_external_project(toml11 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/toml11-3.7.1.tar.gz  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF )
find_package(toml11 REQUIRED)

#https://github.com/rttrorg/rttr/releases/tag/v0.9.6
#rttr-0.9.6-src.tar.gz
#build_external_project(rttr lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rttr-0.9.6-src.tar.gz  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF )
#find_package(rttr REQUIRED)

set(test_fast_dds ON)
if(test_fast_dds)

    #https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#cmake-installation

    # C++ API
    # https://github.com/eProsima/foonathan_memory_vendor/releases
    # https://github.com/eProsima/Fast-CDR/releases
    # https://github.com/eProsima/Fast-DDS/releases


    # fast-dds-gen
    # https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#fast-dds-gen-installation
    #Fast DDS-Gen supports Java versions from 11 to 19.
    #The Fast-DDS-Gen folder contains the following packages:
    #
    #    share/fastddsgen, where the generated Java application is.
    #
    #    scripts, containing some user friendly scripts.
    # To make these scripts accessible from any shell session and directory, add the scripts folder path to the PATH environment variable.

    # DDS on microcontrollers
    # https://github.com/eProsima/Micro-XRCE-DDS


    #foonathan_memory_vendor-1.3.1.tar.gz
    build_external_project(foonathan_memory lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/foonathan_memory_vendor-1.3.1.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DBUILD_SHARED_LIBS=OFF)
    find_package(foonathan_memory REQUIRED)

    #Fast-CDR-1.0.27.zip
    build_external_project(fastcdr lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Fast-CDR-1.0.27.zip  -DCMAKE_BUILD_TYPE=Release )
    #message(1fastcdf_ROOT ${fastcdf_ROOT})

    # fastcdr-config.cmake
    #set(fastcdr_ROOT ${fastcdr_ROOT}/lib/cmake/fastcdr)
    #message(1fastcdf_ROOT ${fastcdf_ROOT})
    find_package(fastcdr REQUIRED)


    #fastrtps-config.cmake
    # Fast-DDS-2.11.2.tar.gz
    build_external_project(fastrtps lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Fast-DDS-2.11.2.tar.gz  -DCMAKE_BUILD_TYPE=Release -Dfoonathan_memory_ROOT=${foonathan_memory_ROOT} -Dfastcdr_ROOT=${fastcdr_ROOT} -DTinyXML2_ROOT=${tinyxml2_ROOT} -DBUILD_SHARED_LIBS=OFF)
    find_package(fastrtps REQUIRED)
endif ()



#ecal-5.11.2.zip
#cmake .. -DCMAKE_BUILD_TYPE=Release -DECAL_THIRDPARTY_BUILD_PROTOBUF=OFF -DECAL_THIRDPARTY_BUILD_CURL=OFF -DECAL_THIRDPARTY_BUILD_HDF5=OFF -DECAL_THIRDPARTY_BUILD_QWT=OFF
#build_external_project(ecal lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecal-5.11.2.zip  -DCMAKE_BUILD_TYPE=Release -DECAL_THIRDPARTY_BUILD_PROTOBUF=OFF -DECAL_THIRDPARTY_BUILD_CURL=OFF -DECAL_THIRDPARTY_BUILD_HDF5=OFF -DECAL_THIRDPARTY_BUILD_QWT=OFF)
#find_package(ecal REQUIRED)

#plplot-5.15.0.tar.gz
build_external_project(plplot lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/plplot-5.15.0.tar.gz -DCMAKE_BUILD_TYPE=Release)
find_package(plplot REQUIRED)

#FakeIt-2.3.2.zip
#build_external_project(FakeIt lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FakeIt-2.3.2.zip -DCMAKE_BUILD_TYPE=Release)
#find_package(FakeIt REQUIRED)

#libcopp-2.1.4.zip
#https://github.com/owent/libcopp
#https://github.com/atframework/cmake-toolset/tree/c911fe34b494dcb3200408803f25787772e5d8c6
build_external_project(libcopp lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libcopp-2.1.4.zip -DCMAKE_BUILD_TYPE=Release)
find_package(libcopp REQUIRED)

#highway-1.0.4.zip
#build_external_project(hwy lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/highway-1.0.4.zip -DCMAKE_BUILD_TYPE=Release)
#find_package(hwy REQUIRED)


build_external_project_command(hwloc lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hwloc-2.9.1.tar.gz "mkdir -p ${CMAKE_BINARY_DIR}/force_hwloc/install/ && ./configure --prefix=${CMAKE_BINARY_DIR}/force_hwloc/install/" "make" "make install")
set(hwloc_INCLUDE_DIR ${hwloc_ROOT}/include)
set(hwloc_LIBRARIES ${hwloc_ROOT}/lib/libhwloc.so)

if(build_can)
    #canfestival-de1fc3261f21.zip
    #examples build fail without python
    #change line24 form `all: objdictedit canfestival examples` to `all: objdictedit canfestival`
    # ldconfig create cache fail without root permission
    # change line48 from `ldconfig` to `ldconfig -C ./ld.so.cache`
    #  --disable-dll
    build_external_project_command(canfestival lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/canfestival-de1fc3261f21.zip "mkdir -p ${CMAKE_BINARY_DIR}/force_canfestival/install/ && ./configure --timers=unix --can=virtual --disable-dll --debug=MSG --prefix=${CMAKE_BINARY_DIR}/force_canfestival/install" "make all PREFIX=${CMAKE_BINARY_DIR}/force_canfestival/install" "make all install PREFIX=${CMAKE_BINARY_DIR}/force_canfestival/install")

    message(STATUS "canfestival_ROOT ${canfestival_ROOT}")

    # link order is important, if link libcanfestival.a before libcanfestival_unix.a, `canDispatch' , `TimeDispatch', `SetAlarm' will be undefined
    # https://stackoverflow.com/questions/51725564/cmake-doesnt-link-c-and-c-static-libraries-undefined-reference-to-function
    set(canfestival_library
            ${canfestival_ROOT}/lib/libcanfestival.a
            ${canfestival_ROOT}/lib/libcanfestival_unix.a
            ${canfestival_ROOT}/lib/libcanfestival.a
            ${canfestival_ROOT}/lib/libcanfestival_can_virtual.so
    )
    set(canfestival_include_dir ${canfestival_ROOT}/include/canfestival)
    message(STATUS "canfestival_library ${canfestival_library}")
    message(STATUS "canfestival_include_dir ${canfestival_include_dir}")
endif ()


#nanobench-4.3.11.tar.gz
build_external_project(nanobench lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanobench-4.3.11.tar.gz -DCMAKE_BUILD_TYPE=Release)
find_package(nanobench REQUIRED)
#doctest-2.4.11.tar.gz
build_external_project(doctest lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/doctest-2.4.11.tar.gz -DCMAKE_BUILD_TYPE=Release)
find_package(doctest REQUIRED)


#sockpp-0.8.1.tar.gz
build_external_project(sockpp lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sockpp-0.8.1.tar.gz -DCMAKE_BUILD_TYPE=Release -DSOCKPP_BUILD_EXAMPLES=OFF -DSOCKPP_BUILD_SHARED=OFF -DSOCKPP_BUILD_STATIC=ON)
find_package(sockpp REQUIRED)


#bolt-2.0.tar.gz
#build_external_project(bolt lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bolt-2.0.tar.gz -DCMAKE_BUILD_TYPE=Release -DLIBOMP_USE_ARGOBOTS=off)
#
#add_library(bolt STATIC IMPORTED)
#set_target_properties(bolt PROPERTIES  IMPORTED_LOCATION ${bolt_ROOT}/lib/libomp.so)
#target_include_directories(bolt INTERFACE ${bolt_ROOT}/include)
#target_link_directories(bolt INTERFACE ${bolt_ROOT}/lib)


#easy_profiler-2.1.0.tar.gz
build_external_project(easy_profiler lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/easy_profiler-develop.zip -DCMAKE_BUILD_TYPE=Release -DEASY_PROFILER_NO_SAMPLES=ON -DBUILD_SHARED_LIBS=OFF )
find_package(easy_profiler REQUIRED)

#orbit
#pip install conan==1.60.0
#./bootstrap-orbit.sh


include(cmake/set_ros.cmake)

find_package(Boost
        1.71.0      # Minimum or EXACT version e.g. 1.67.0
        REQUIRED            # Fail with error if Boost is not found
        COMPONENTS thread chrono filesystem
        )

message(Boost_LIBRARIES = ${Boost_LIBRARIES})
#set(ROOT_ROOT ${CMAKE_SOURCE_DIR}/third_party/root_v6.28.02.Linux-ubuntu20-x86_64-gcc9.4/root/cmake )
#find_package(ROOT REQUIRED COMPONENTS RIO Net)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/third_party/pybind11-2.10.4)
add_subdirectory(src)




#include(deb)  # build/cmake/deb.cmake
include(CPack)

