cmake_minimum_required(VERSION 3.16)
project(cmake_super_build)

include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)
include(cmake/find_opencv_helper.cmake)
set(CMAKE_CXX_STANDARD  14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


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


# Eigen3
build_external_project(Eigen3 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0.zip  -DCMAKE_BUILD_TYPE=Release )
message(Eigen3_ROOT : ${Eigen3_ROOT} )

set(Eigen3_ROOT ${Eigen3_ROOT}/share/eigen3/cmake)
find_package(Eigen3 3.4.0 REQUIRED HINTS ${Eigen3_ROOT})
message(new eigen: EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS} )


build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.1.0.tar.gz  -DEigen3_ROOT=${Eigen3_ROOT} -DCMAKE_BUILD_TYPE=Release -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCUMENTATION=OFF)
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )
#find_opencv(/home/waxz/dev/docker/docker_dev/share/opencv/install 4.6.0)

build_external_project(Fruit lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/fruit-3.6.0.tar.gz  -DFRUIT_USES_BOOST=False -DCMAKE_BUILD_TYPE=Release)
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

set(LUA_VERSION "5.3.5" CACHE STRING "The version of Lua needed. Can be 5.1, 5.2, 5.3, LuaJIT, or a more specific 3-part version number for a specifc Lua (e.g., 5.3.4 or luajit-2.0.5)")
string(TOLOWER ${LUA_VERSION} NORMALIZED_LUA_VERSION)
#    find_package(LuaBuild REQUIRED COMPONENTS ${LUA_VERSION})
include(ExternalProject)

build_external_project_make(lua lib "https://www.lua.org/ftp/lua-${LUA_VERSION}.tar.gz")
message(lua_ROOT : ${lua_ROOT})
set(LUA_LIBRARIES ${lua_ROOT}/lib/liblua.a)
set(LUA_INCLUDE_DIR ${lua_ROOT}/include)


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
build_external_project(absl lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/abseil-cpp-20220623.1.tar.gz -DABSL_BUILD_TESTING=OFF -DABSL_USE_GOOGLETEST_HEAD=ON -DCMAKE_CXX_STANDARD=14  -DABSL_ENABLE_INSTALL=ON)
find_package(absl REQUIRED)


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


build_external_project(tinyxml2 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2-9.0.0.tar.gz  -DCMAKE_BUILD_TYPE=Release  )
find_package(tinyxml2 REQUIRED)


#build_external_project(mv-lm-icp lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mv-lm-icp-master-all.zip  -DCMAKE_BUILD_TYPE=Release  )
#find_package(mv-lm-icp REQUIRED)

build_external_project(PCL lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pcl-pcl-1.13.0.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DBUILD_examples=OFF -DBUILD_apps=OFF -DBUILD_global_tests=OFF -DBUILD_simulation=OFF -DBUILD_tools=OFF)
find_package(PCL 1.13 REQUIRED )
message("PCL_LIBRARIES = " ${PCL_LIBRARIES})

#ifm3d-0.93.0.zip
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
#octomap-1.9.6.tar.gz
build_external_project(octomap lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/octomap-1.9.6.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DBUILD_OCTOVIS_SUBPROJECT=OFF -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF)
find_package(octomap REQUIRED)

# libnabo-master.zip
build_external_project(libnabo lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libnabo-master.zip  -DCMAKE_BUILD_TYPE=Release -DBUILD_TYPE=cmake -Dcatkin_DIR=/opt/ros/noetic/share/catkin/cmake )
find_package(libnabo REQUIRED)

# libpointmatcher-master.zip -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIRS}
build_external_project(libpointmatcher lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libpointmatcher-master.zip -DBUILD_TYPE=cmake -DCMAKE_BUILD_TYPE=Release -Dlibnabo_ROOT=${libnabo_ROOT} -DSHARED_LIBS=OFF )

find_package(libpointmatcher REQUIRED)

# taskflow-3.4.0.tar.gz
build_external_project(Taskflow lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/taskflow-3.4.0.tar.gz -DBUILD_TYPE=cmake -DCMAKE_BUILD_TYPE=Release -DTF_BUILD_EXAMPLES=OFF -DTF_BUILD_TESTS=OFF -DTF_BUILD_PROFILER=OFF -DTF_BUILD_BENCHMARKS=OFF  )

find_package(Taskflow REQUIRED)



build_external_project(autodiff lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/autodiff-0.6.12.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_PYTHON=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF -DAUTODIFF_BUILD_DOCS=OFF)
find_package(autodiff REQUIRED)

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

# https://github.com/openresty/luajit2
build_external_project_command(luajit lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/luajit2-2.1-20230119.tar.gz "mkdir -p ${CMAKE_BINARY_DIR}/force_luajit/install/bin/luajit-2.1.0-beta3" "make PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install" "make install PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install")
set(LUA_LIBRARIES ${luajit_ROOT}/lib/libluajit-5.1.a)
set(LUA_INCLUDE_DIR ${luajit_ROOT}/include/luajit-2.1)

add_library(liblua STATIC IMPORTED)
set_target_properties(liblua PROPERTIES  IMPORTED_LOCATION "${LUA_LIBRARIES}")
target_include_directories(liblua INTERFACE ${LUA_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/third_party/sol)
target_link_libraries(liblua INTERFACE  dl )




#third_party/Fast-DDS-2.9.1.zip
build_external_project(foonathan_memory lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/foonathan_memory_vendor-1.3.0.zip  -DCMAKE_BUILD_TYPE=Release  -DBUILD_SHARED_LIBS=ON)
find_package(foonathan_memory REQUIRED)

build_external_project(fastcdr lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Fast-CDR-1.0.26.zip  -DCMAKE_BUILD_TYPE=Release )
#message(1fastcdf_ROOT ${fastcdf_ROOT})

# fastcdr-config.cmake
#set(fastcdr_ROOT ${fastcdr_ROOT}/lib/cmake/fastcdr)
#message(1fastcdf_ROOT ${fastcdf_ROOT})
find_package(fastcdr REQUIRED)


#fastrtps-config.cmake
build_external_project(fastrtps lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Fast-DDS-2.9.1.zip  -DCMAKE_BUILD_TYPE=Release -Dfoonathan_memory_ROOT=${foonathan_memory_ROOT} -Dfastcdr_ROOT=${fastcdr_ROOT} -DFASTDDS_STATIC=OFF)
find_package(fastrtps REQUIRED)

#iceoryx-2.0.3.zip
build_external_project_command(iceoryx lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/iceoryx-2.0.3.zip  "cmake -Bbuild -Hiceoryx_meta -DBUILD_DOC=OFF -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/force_iceoryx/install" "cmake --build build" " cmake --build build --target install" )
set(iceoryx_binding_c_ROOT ${iceoryx_ROOT})
find_package(iceoryx_binding_c REQUIRED)

#cyclonedds-0.10.2.zip
build_external_project(CycloneDDS lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cyclonedds-0.10.2.zip -Diceoryx_binding_c_ROOT=${iceoryx_binding_c_ROOT} -DCMAKE_BUILD_TYPE=Release -DENABLE_SHM=YES)
find_package(CycloneDDS REQUIRED)

#cyclonedds-cxx-0.10.2.zip
#
build_external_project(CycloneDDS-CXX lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cyclonedds-cxx-0.10.2.zip -DCycloneDDS_ROOT=${CycloneDDS_ROOT}  -Diceoryx_binding_c_ROOT=${iceoryx_binding_c_ROOT}  -DCMAKE_BUILD_TYPE=Release -DENABLE_SHM=YES)
find_package(CycloneDDS-CXX REQUIRED)

#ecal-5.11.2.zip
#cmake .. -DCMAKE_BUILD_TYPE=Release -DECAL_THIRDPARTY_BUILD_PROTOBUF=OFF -DECAL_THIRDPARTY_BUILD_CURL=OFF -DECAL_THIRDPARTY_BUILD_HDF5=OFF -DECAL_THIRDPARTY_BUILD_QWT=OFF
#build_external_project(ecal lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecal-5.11.2.zip  -DCMAKE_BUILD_TYPE=Release -DECAL_THIRDPARTY_BUILD_PROTOBUF=OFF -DECAL_THIRDPARTY_BUILD_CURL=OFF -DECAL_THIRDPARTY_BUILD_HDF5=OFF -DECAL_THIRDPARTY_BUILD_QWT=OFF)
#find_package(ecal REQUIRED)

#plplot-5.15.0.tar.gz
build_external_project(plplot lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/plplot-5.15.0.tar.gz -DCMAKE_BUILD_TYPE=Release)
find_package(plplot REQUIRED)


include(cmake/set_ros.cmake)

add_subdirectory(src)




#include(deb)  # build/cmake/deb.cmake
include(CPack)

