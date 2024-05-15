
add_library(absl_hash_seed_test SHARED absl_hash_seed_test.cpp)
target_link_libraries(absl_hash_seed_test PUBLIC
        absl::flat_hash_map
        absl::strings  absl::str_format
        absl::log
        #        absl::log_internal_check_impl
        absl::log_internal_check_op
        absl::statusor
        absl::debugging
        absl::random_random
        absl::any
)


add_executable(absl_variant_test absl_variant_test.cpp)
target_include_directories(absl_variant_test PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(absl_variant_test PUBLIC
        absl::flat_hash_map
        absl::strings  absl::str_format
        absl::log
        #        absl::log_internal_check_impl
        absl::log_internal_check_op
        absl::statusor
        absl::debugging
        absl::random_random
        absl::any
)
target_link_libraries(absl_variant_test PUBLIC
        absl_hash_seed_test
)