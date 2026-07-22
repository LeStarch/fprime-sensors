add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Rfm69Config")

register_fprime_module(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Subtopology.fpp"
    EXCLUDE_FROM_ALL
    DEPENDS
        fprime-sensors_Rfm69_Subtopology_Rfm69Config
)
