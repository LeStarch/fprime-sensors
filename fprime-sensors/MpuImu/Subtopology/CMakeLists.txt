add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/MpuImuConfig")

register_fprime_module(
    INTERFACE
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Subtopology.fpp"
    EXCLUDE_FROM_ALL
)
