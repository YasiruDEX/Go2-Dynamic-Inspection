# Unit tests:
mola_add_test(
  TARGET  test-imu-rotation-integrator
  SOURCES test-imu-rotation-integrator.cpp
  LINK_LIBRARIES
    mola::mola_imu_preintegration
)

mola_add_test(
  TARGET  test-velocity-buffer
  SOURCES test-velocity-buffer.cpp
  LINK_LIBRARIES
    mola::mola_imu_preintegration
)

mola_add_test(
  TARGET  test-trajectory-from-buffer
  SOURCES test-trajectory-from-buffer.cpp
  LINK_LIBRARIES
    mola::mola_imu_preintegration
)

mola_add_test(
  TARGET  test-imu-initial-calibrator
  SOURCES test-imu-initial-calibrator.cpp
  LINK_LIBRARIES
    mola::mola_imu_preintegration
)
