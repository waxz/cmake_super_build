cmake_minimum_required(VERSION 3.23)
project(cmake_super_build)

include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)
include(cmake/find_opencv_helper.cmake)
set(CMAKE_CXX_STANDARD  14)

build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.1.0.tar.gz  -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=ON)
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )
find_opencv(/home/waxz/dev/docker/docker_dev/share/opencv/install 4.6.0)

add_executable(cmake_super_build main.cpp)
target_link_libraries(cmake_super_build PUBLIC
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
        )
target_include_directories(
        cmake_super_build PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
set_asan(cmake_super_build)


