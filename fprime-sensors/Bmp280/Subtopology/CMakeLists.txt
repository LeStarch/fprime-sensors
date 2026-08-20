add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Bmp280Config")

register_fprime_module(
    INTERFACE
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Subtopology.fpp"
    DEPENDS
        fprime-sensors_Bmp280_Subtopology_Bmp280Config

) 