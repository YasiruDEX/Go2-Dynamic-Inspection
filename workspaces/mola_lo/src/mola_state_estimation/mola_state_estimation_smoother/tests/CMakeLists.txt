# Unit tests:
mola_add_test(
  TARGET  test-navstate-basic
  SOURCES test-navstate-basic.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-navstate-odom-gnss-fusion
  SOURCES test-navstate-odom-gnss-fusion.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-static-gnss-imu-orientation
  SOURCES test-static-gnss-imu-orientation.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-two-odometries
  SOURCES test-two-odometries.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-lidar-imu-leveling
  SOURCES test-lidar-imu-leveling.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-twist-fusion
  SOURCES test-twist-fusion.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

