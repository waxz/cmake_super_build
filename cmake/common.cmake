cmake_policy(SET CMP0074 NEW)
find_package(Threads REQUIRED)
# target_link_libraries( target PUBLIC Threads::Threads)

# ---------------------------------------------------------------------------------------------------------
# Handle C++ standard version.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++... instead of -std=gnu++...


# ---------------------------------------------------------------------------------------------------------

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# ---------------------------------------------------------------------------------------------------------

# flags

# fix ld: unrecognized option '--push-state--no-as-needed'
# https://stackoverflow.com/questions/50024731/ld-unrecognized-option-push-state-no-as-needed
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")



# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
            "MinSizeRel" "RelWithDebInfo")
endif()

# https://stackoverflow.com/questions/17707044/getting-cmake-to-give-an-error-warning-about-unreferenced-symbols

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Werror=return-type")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
set(CMAKE_EXE_LINKER_FLAGS    "-Wl,--as-needed ${CMAKE_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed ${CMAKE_SHARED_LINKER_FLAGS}")

#https://stackoverflow.com/questions/48754619/what-are-cmake-build-type-debug-release-relwithdebinfo-and-minsizerel


list(APPEND ABSL_GCC_FLAGS
        "-Wall"
        "-Wextra"
        "-Wcast-qual"
        "-Wconversion-null"
        "-Wformat-security"
        "-Wmissing-declarations"
        "-Woverlength-strings"
        "-Wpointer-arith"
        "-Wundef"
        "-Wunused-local-typedefs"
        "-Wunused-result"
        "-Wvarargs"
        "-Wvla"
        "-Wwrite-strings"
        "-DNOMINMAX"
        )

list(APPEND ABSL_GCC_TEST_FLAGS
        "-Wall"
        "-Wextra"
        "-Wcast-qual"
        "-Wconversion-null"
        "-Wformat-security"
        "-Woverlength-strings"
        "-Wpointer-arith"
        "-Wundef"
        "-Wunused-local-typedefs"
        "-Wunused-result"
        "-Wvarargs"
        "-Wvla"
        "-Wwrite-strings"
        "-DNOMINMAX"
        "-Wno-deprecated-declarations"
        "-Wno-missing-declarations"
        "-Wno-self-move"
        "-Wno-sign-compare"
        "-Wno-unused-function"
        "-Wno-unused-parameter"
        "-Wno-unused-private-field"
        )

function (SET_GCC_FLAGS target)
    target_compile_options(
            ${target}
            PRIVATE
            ${ABSL_GCC_FLAGS}
    )

endfunction()


if (CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate -fprofile-use")


    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--whole-archive  -Wl,--no-whole-archive")
    #  -ffp-contract=fast -flto
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -march=native -ftree-vectorize -fopt-info-vec-optimized -ffp-contract=fast -flto")

endif ()

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate -fprofile-use")


    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--whole-archive  -Wl,--no-whole-archive")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -march=native -ftree-vectorize -fopt-info-vec-optimized ")

endif ()
if (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    #-Wall -Wextra -pedantic
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Ofast  -ffast-math -ftree-vectorize   -march=native -funsafe-loop-optimizations -mavx -mfma")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Wall -Wextra   -Ofast   ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Wall -Wextra   -march=native   -O2 -g -DNDEBUG  -Ofast -funsafe-loop-optimizations ")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -ftree-vectorize  -ffast-math -fopt-info-vec-optimized  -opt-report=5 ")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -pthread -fprofile-arcs -mstackrealign  -march=native")


endif ()
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic -Wno-dev -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -O0 -o -ggdb  -ggdb3 -fprofile-arcs -mstackrealign  -march=native  ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread  -g -O0 ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -mstackrealign  -march=native  -fstack-protector")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -fno-optimize-sibling-calls -fno-omit-frame-pointer -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free")

    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined   -fsanitize=address -fsanitize=leak -fsanitize=leak ")
    #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address  -fsanitize=leak")
    #set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=address  -fsanitize=leak")
endif ()

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
    target_compile_options(${target} PUBLIC  "-fuse-ld=gold")

#    target_compile_options(${target} PUBLIC -fsanitize-coverage=trace-pc-guard -fsanitize=address,undefined,leak -fuse-ld=gold)


    target_link_libraries(${target} PUBLIC "-fsanitize=address  -fsanitize=leak -fsanitize=undefined")
    #    target_compile_options(${target} PUBLIC "-fsanitize=memory")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=memory")
    #    target_compile_options(${target} PUBLIC "-fsanitize=thread")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=thread")

endfunction(set_asan)
# ---------------------------------------------------------------------------------------------------------


function(print_include_dir target)
    get_target_property(LIBA_INCLUDES ${target} INCLUDE_DIRECTORIES)
    message(${target} INCLUDES : ${LIBA_INCLUDES})
endfunction()

# ---------------------------------------------------------------------------------------------------------


function(set_omp target)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(${target} PUBLIC OpenMP::OpenMP_CXX)
        target_include_directories(${target} PUBLIC ${OpenMP_CXX_INCLUDE_DIRS} )
        target_link_libraries(${target} PUBLIC
                ${OpenMP_CXX_LIBRARIES}
                )
        target_compile_options(${target} PUBLIC ${OpenMP_CXX_FLAGS})
    endif()

endfunction()










# ---------------------------------------------------------------------------------------------------------

macro(install_target)
    message(STATUS "Configuring installation for target(s) ${ARGV0}")
    install(TARGETS ${ARGV0}
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )
endmacro()

# ---------------------------------------------------------------------------------------------------------

# https://stackoverflow.com/a/51987470
#https://stackoverflow.com/questions/32183975/how-to-print-all-the-properties-of-a-target-in-cmake

# NOTE: Only used in multi-configuration environments
#set(CMAKE_CONFIGURATION_TYPES "Debug;RelWithDebInfo" CACHE STRING "My multi config types" FORCE)
# Get all propreties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

# Convert command output into a CMake list
STRING(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
STRING(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
# Fix https://stackoverflow.com/questions/32197663/how-can-i-remove-the-the-location-property-may-not-be-read-from-target-error-i
list(FILTER CMAKE_PROPERTY_LIST EXCLUDE REGEX "^LOCATION$|^LOCATION_|_LOCATION$")
# For some reason, "TYPE" shows up twice - others might too?
list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)

# build whitelist by filtering down from CMAKE_PROPERTY_LIST in case cmake is
# a different version, and one of our hardcoded whitelisted properties
# doesn't exist!
unset(CMAKE_WHITELISTED_PROPERTY_LIST)
foreach(prop ${CMAKE_PROPERTY_LIST})
    if(prop MATCHES "^(INTERFACE|[_a-z]|IMPORTED_LIBNAME_|MAP_IMPORTED_CONFIG_)|^(COMPATIBLE_INTERFACE_(BOOL|NUMBER_MAX|NUMBER_MIN|STRING)|EXPORT_NAME|IMPORTED(_GLOBAL|_CONFIGURATIONS|_LIBNAME)?|NAME|TYPE|NO_SYSTEM_FROM_IMPORTED)$")
        list(APPEND CMAKE_WHITELISTED_PROPERTY_LIST ${prop})
    endif()
endforeach(prop)

function(print_properties)
    message ("CMAKE_PROPERTY_LIST = ${CMAKE_PROPERTY_LIST}")
endfunction(print_properties)

function(print_whitelisted_properties)
    message ("CMAKE_WHITELISTED_PROPERTY_LIST = ${CMAKE_WHITELISTED_PROPERTY_LIST}")
endfunction(print_whitelisted_properties)

function(print_target_properties tgt)
    if(NOT TARGET ${tgt})
        message("There is no target named '${tgt}'")
        return()
    endif()

    get_target_property(target_type ${tgt} TYPE)
    if(target_type STREQUAL "INTERFACE_LIBRARY")
        set(PROP_LIST ${CMAKE_WHITELISTED_PROPERTY_LIST})
    else()
        set(PROP_LIST ${CMAKE_PROPERTY_LIST})
    endif()

    foreach (prop ${PROP_LIST})
        string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" prop ${prop})
        # message ("Checking ${prop}")
        get_property(propval TARGET ${tgt} PROPERTY ${prop} SET)
        if (propval)
            get_target_property(propval ${tgt} ${prop})
            message ("${tgt} ${prop} = ${propval}")
        endif()
    endforeach(prop)
endfunction(print_target_properties)

