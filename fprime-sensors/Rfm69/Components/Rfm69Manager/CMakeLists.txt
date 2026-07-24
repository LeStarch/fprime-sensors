####
# FPrime CMakeLists.txt:
#
# More information in the F´ CMake API documentation:
# https://fprime.jpl.nasa.gov/latest/docs/user-manual/build-system/cmake-api/
#
####

register_fprime_module(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69Manager.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69Manager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69Helpers.cpp"
    DEPENDS
        Drv_Ports
)

register_fprime_ut(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/Rfm69Manager.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/test/ut/Rfm69ManagerTestMain.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/test/ut/Rfm69ManagerTester.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../Rfm69Sim/Rfm69SimModel.cpp"
    UT_AUTO_HELPERS
)
