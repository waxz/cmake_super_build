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



build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.1.0.tar.gz   -DCMAKE_BUILD_TYPE=Debug -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=ON)
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
build_external_project(TBB lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oneTBB-2021.7.0-rc1.tar.gz -DCMAKE_BUILD_TYPE=Release -DTBB_TEST=OFF)
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

add_library(liblua STATIC IMPORTED)
set_target_properties(liblua PROPERTIES  IMPORTED_LOCATION "${LUA_LIBRARIES}")
target_include_directories(liblua INTERFACE ${LUA_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/third_party/sol)
target_link_libraries(liblua INTERFACE  dl )
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

#set(Open3D_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-pre-cxx11-abi-0.15.1)

#Open3D-0.15.1.tar.gz
#build_external_project(Open3D lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Open3D-0.15.1.tar.gz -DBUILD_FILAMENT_FROM_SOURCE=OFF -DGLIBCXX_USE_CXX11_ABI=OFF -DBUILD_GUI=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_CACHED_CUDA_MANAGER=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_MODULE=OFF )
#
#find_package(Open3D REQUIRED)

#matplotplusplus-1.1.0.tar.gz
#build_external_project(Matplot++ lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/matplotplusplus-1.1.0.tar.gz  -DBUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON)
#find_package(Matplot++ REQUIRED)

# range-v3-0.12.0.tar.gz
#build_external_project(range-v3 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/range-v3-0.12.0.tar.gz   -DCMAKE_BUILD_TYPE=Release  )
#find_package(range-v3 REQUIRED)


build_external_project(tinyxml2 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2-9.0.0.tar.gz  -DCMAKE_BUILD_TYPE=Release  )
find_package(tinyxml2 REQUIRED)


#build_external_project(mv-lm-icp lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mv-lm-icp-master-all.zip  -DCMAKE_BUILD_TYPE=Release  )
#find_package(mv-lm-icp REQUIRED)



include(cmake/set_ros.cmake)

add_subdirectory(src)




#include(deb)  # build/cmake/deb.cmake
include(CPack)

