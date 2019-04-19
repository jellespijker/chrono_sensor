project(chrono_sensor_tests)

include(FetchContent)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.8.1
)

FetchContent_GetProperties(googletest)

if (NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
    set(gtest_force_shared_crt ON CACHE BOOL " " FORCE)
    set(gtest_disable_pthreads ON CACHE BOOL " " FORCE)
    add_subdirectory(
            ${googletest_SOURCE_DIR}
            ${googletest_BINARY_DIR}
    )
endif ()

find_package(Boost REQUIRED)
if (NOT Boost_FOUND)
    message(STATUS "Could not find Boost needed for statistics testing")
    message(STATUS "SKIPPING statistics testing")
    add_definitions(-DSTAT_TEST)
endif ()

include_directories(../chrono_sensor/include)
include_directories(${BOOST_INCLUDEDIR})

set(SOURCE_FILES main.cpp src/chrono_sensor_tests.cpp src/chrono_sensor_function_tests.cpp)

add_executable(chrono_sensor_tests "")
target_sources(chrono_sensor_tests
        PRIVATE
        ${SOURCE_FILES}
        )
target_link_libraries(chrono_sensor_tests
        PRIVATE
        chrono_sensor
        ${Boost_LIBRARIES}
        gtest_main)
enable_testing()
add_test(
        NAME google_test
        COMMAND $<TARGET_FILE:chrono_sensor_tests>
)

