add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/NmeaGpsSubtopologyConfig")

register_fprime_module(
    INTERFACE
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Subtopology.fpp"
    DEPENDS
        fprime-sensors_NmeaGps_Subtopology_NmeaGpsSubtopologyConfig
        fprime-sensors_NmeaGps_Components_NmeaDetector
    EXCLUDE_FROM_ALL
)
