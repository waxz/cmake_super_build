add_executable(ceres_simple ceres_simple.cpp)
target_link_libraries(ceres_simple PUBLIC Ceres::ceres)

add_executable(ceres_numeric_wrap_test ceres_numeric_wrap_test.cpp)
target_link_libraries(ceres_numeric_wrap_test PUBLIC Ceres::ceres)