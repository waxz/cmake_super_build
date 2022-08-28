cmake_minimum_required(VERSION 3.23)
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



build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.1.0.tar.gz  -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=ON)
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )
find_opencv(/home/waxz/dev/docker/docker_dev/share/opencv/install 4.6.0)

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
build_external_project(cppzmq lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cppzmq-4.8.0.tar.gz -DCMAKE_BUILD_TYPE=Release -DZeroMQ_ROOT=${ZeroMQ_ROOT} -DZeroMQ_DIR=${ZeroMQ_ROOT})

#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq REQUIRED)
message(cppzmq_DIR : ${cppzmq_DIR} , cppzmq_INCLUDE_DIR : ${cppzmq_INCLUDE_DIR})


# TBB
build_external_project(TBB lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oneTBB-2021.7.0-rc1.tar.gz -DCMAKE_BUILD_TYPE=Release)
find_package(TBB REQUIRED)

# flatbuffers FlatBuffersConfig.cmake
build_external_project(FLATBUFFERS lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/flatbuffers-2.0.7.tar.gz -DCMAKE_BUILD_TYPE=Release)
set(FlatBuffers_ROOT ${FLATBUFFERS_ROOT})
find_package(FlatBuffers REQUIRED)


add_subdirectory(src)

#include(deb)  # build/cmake/deb.cmake
include(CPack)

