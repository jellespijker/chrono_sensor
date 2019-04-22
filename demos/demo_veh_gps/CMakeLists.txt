project(DEMO_VEH_GPS)

if (NOT Chrono_FOUND)
    message("Could not find Chrono or one of its required modules")
    return()
endif ()

include_directories(../chrono_sensor/include)
include_directories(${CHRONO_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp)

add_executable(DEMO_VEH_GPS "")
target_sources(DEMO_VEH_GPS
        PRIVATE
        ${SOURCE_FILES}
        )
target_link_libraries(DEMO_VEH_GPS
        PRIVATE
        chrono_sensor
        ${CHRONO_LIBRARIES})

add_definitions(-DCHRONO_DATA_DIR="${CHRONO_DATA_DIR}")
add_definitions(-DCHRONO_VEHICLE_DATA_DIR="${CHRONO_VEHICLE_DATA_DIR}")

set_target_properties(DEMO_VEH_GPS PROPERTIES
        COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
        LINK_FLAGS "${CHRONO_LINKER_FLAGS}")
