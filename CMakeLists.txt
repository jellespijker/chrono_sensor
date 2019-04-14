cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# C++ 14 STANDARD GNU
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# DEFINE BINARY AND LIBS PATHS ACCORDING TO GNU STANDARDS
include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

# PROJECT CHRONO PACKAGE
if (ENABLE_DEMOS)
    find_package(Chrono
            COMPONENTS Irrlicht Vehicle
            CONFIG)
else ()
    find_package(Chrono
            CONFIG)
endif ()
if (NOT Chrono_FOUND)
    message(STATUS "Could not find Chrono or one of its required modules")
    return()
endif ()
include_directories(${CHRONO_INCLUDE_DIRS})

# SOURCE CODE
add_subdirectory(chrono_sensor)

# EXAMPLES
option(ENABLE_DEMOS "Enable examples" ON)
message(STATUS "Enable demos: ${ENABLE_DEMOS}")
if (ENABLE_DEMOS)
    add_subdirectory(demos)
endif ()

# UNIT TESTING
option(ENABLE_UNIT_TESTS "Enable unit test" ON)
message(STATUS "Enable testing: ${ENABLE_UNIT_TESTS}")
if (ENABLE_UNIT_TESTS)
    add_subdirectory(test)
endif ()


