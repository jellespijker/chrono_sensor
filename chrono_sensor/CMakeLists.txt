project(chrono_sensor VERSION 0.1 LANGUAGES CXX)

add_library(chrono_sensor SHARED
        src/ChSensor.cpp)

target_include_directories(chrono_sensor PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

set_target_properties(chrono_sensor PROPERTIES
        COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
        COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
        LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(chrono_sensor PUBLIC ${CHRONO_LIBRARIES})

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS chrono_sensor EXPORT chrono_sensorConfig
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT chrono_sensorConfig DESTINATION share/chrono_sensor/cmake)

# This makes the project importable from the build directory
export(TARGETS chrono_sensor FILE chrono_sensorConfig.cmake)