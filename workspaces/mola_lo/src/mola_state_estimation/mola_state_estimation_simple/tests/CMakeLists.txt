# Unit tests:
mola_add_test(
  TARGET  test-state-estimation-simple
  SOURCES test-state-estimation-simple.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_simple
)
