add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/XBeeConfig")

register_fprime_module(
    INTERFACE
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Subtopology.fpp"
    EXCLUDE_FROM_ALL
    DEPENDS
        fprime-sensors_XBee_Subtopology_XBeeConfig
)
