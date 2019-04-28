project(chrono_sensor VERSION 0.1 LANGUAGES CXX)

set(SRC_FILES
        src/Accelerometer.cpp
        src/Gyroscope.cpp)

set(HDR_FILES
        include/chrono_sensor/ChSensor.h
        include/chrono_sensor/ChFunction_Sensor.h
        include/chrono_sensor/ChFunction_SensorNoise.h
        include/chrono_sensor/ChFunction_SensorBias.h
        include/chrono_sensor/ChFunction_SensorDigitize.h
        include/chrono_sensor/Gyroscope.h
        )

add_library(chrono_sensor SHARED ${SRC_FILES} ${HDR_FILES})


target_include_directories(chrono_sensor PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)
target_compile_options(chrono_sensor PUBLIC -pthread -fopenmp -march=native -msse4.2 -mfpmath=sse -march=native -mavx)
target_compile_definitions(chrono_sensor PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
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