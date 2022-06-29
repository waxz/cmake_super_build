cmake_minimum_required(VERSION 3.23)
project(cmake_super_build)

include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)



build_external_project(Ceres lib /home/waxz/CLionProjects/docker_demo/3rdparty/ceres-solver-2.1.0.tar.gz  -DMINIGLOG=ON -DGFLAGS=OFF )
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )


add_executable(cmake_super_build main.cpp)
target_link_libraries(cmake_super_build PUBLIC
        ${CERES_LIBRARIES}
        )
set_asan(cmake_super_build)


