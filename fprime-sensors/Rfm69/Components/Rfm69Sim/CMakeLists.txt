####
# FPrime CMakeLists.txt:
#
# More information in the F´ CMake API documentation:
# https://fprime.jpl.nasa.gov/latest/docs/user-manual/build-system/cmake-api/
#
####

register_fprime_module(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69Sim.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69Sim.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69SimModel.cpp"
)

register_fprime_ut(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69Sim.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/test/ut/Rfm69SimModelTestMain.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69SimModel.cpp"
)
