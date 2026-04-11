

# Test to run the mola-sm-georeferencing CLI app:
add_test(
    NAME test-mola-sm-georeferencing
    COMMAND $<TARGET_FILE:mola-sm-georeferencing>
        --input ${CMAKE_CURRENT_SOURCE_DIR}/m2dgr_gate01_gps.simplemap
)
set_tests_properties(test-mola-sm-georeferencing PROPERTIES
    PASS_REGULAR_EXPRESSION "Obtained georeferencing:.*lat: 31deg 1' 29.*lon: 121deg 26' 26.*h: 18.8"
)


# C++ unit tests:
mola_add_test(
  TARGET  test_imu_attitude
  SOURCES test_imu_attitude.cpp
  LINK_LIBRARIES
    mola::mola_georeferencing
)
