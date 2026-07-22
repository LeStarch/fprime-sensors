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
