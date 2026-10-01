# L6 stage (b), the rotor-speed sensor path (decision 0013, owner decision 7), T1:
#   unit_l6_rotor_speed          the plant's model: telemetry-grid known answers, the stopped and invalid rules, the pole
#                                count, the delay, refusals, bit-identity of the step and IMU bytes, the layout against
#                                the firmware's RotorSpeedSample; controls: pole count off by 2, rev/s for rad/s,
#                                rounding for truncation, the delay off by one tick, a one-ulp body change.
#   unit_l6_rotor_speed_adapter  the adapter's opt-in path: the bytes handed to the SIL equal a twin plant's direct sample
#                                sequence. marv_sil_tick and marv_sil_tick_with_rotor_speed are recorders here (no SIL).
#   unit_l6_rotor_speed_sil      the SIL entry marv_sil_tick_with_rotor_speed and the HAL pull read, against the real SIL
#                                sources and a probe composition. The SIL sources are compiled into this executable (as
#                                marv_add_sil_library compiles them into a shared library, whose symbols are hidden from the
#                                test); marv_sil_init succeeds once per process, so every case runs in a forked child.
# The shared scenario fixture is the frozen L1 plant fixture (every value in it is a labelled scenario value).
add_executable(unit_l6_rotor_speed kat_test.cpp model_test.cpp refusal_test.cpp identity_test.cpp layout_test.cpp)
target_include_directories(unit_l6_rotor_speed PRIVATE
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/fw/sil/include"
  "${PROJECT_SOURCE_DIR}/sim/plant/src")
target_link_libraries(unit_l6_rotor_speed PRIVATE marv_plant marv_plant_noise marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l6_rotor_speed)
marv_frozen_suite(unit_l6_rotor_speed)

add_executable(unit_l6_rotor_speed_adapter adapter_test.cpp)
target_include_directories(unit_l6_rotor_speed_adapter PRIVATE
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L06/adapter"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L06/rotor_speed")
target_link_libraries(unit_l6_rotor_speed_adapter PRIVATE
  marv_gz_adapter marv_plant marv_plant_noise marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l6_rotor_speed_adapter)
marv_frozen_suite(unit_l6_rotor_speed_adapter)

add_library(marv_l06_rotor_probe_composition STATIC composition/composition.cpp)
target_include_directories(marv_l06_rotor_probe_composition PUBLIC composition/include)
target_link_libraries(marv_l06_rotor_probe_composition PUBLIC marv_hal marv_types)
marv_apply_flags(marv_l06_rotor_probe_composition FIRMWARE)

add_library(marv_l06_rotor_probe_sil STATIC
  "${PROJECT_SOURCE_DIR}/fw/sil/src/marv_sil.cpp"
  "${PROJECT_SOURCE_DIR}/fw/sil/src/param_override.cpp")
target_include_directories(marv_l06_rotor_probe_sil
  PUBLIC "${PROJECT_SOURCE_DIR}/fw/sil/include"
  PRIVATE "${PROJECT_SOURCE_DIR}/fw/sil/src")
target_link_libraries(marv_l06_rotor_probe_sil PUBLIC
  marv_l06_rotor_probe_composition marv_hal_sim marv_params_l0_runtime marv_params_l0_generated)
marv_apply_flags(marv_l06_rotor_probe_sil FIRMWARE)

add_executable(unit_l6_rotor_speed_sil sil_test.cpp)
target_link_libraries(unit_l6_rotor_speed_sil PRIVATE
  marv_l06_rotor_probe_sil marv_l06_rotor_probe_composition marv_hal_sim marv_types marv_params_l0_generated
  GTest::gtest_main)
marv_apply_flags(unit_l6_rotor_speed_sil)
marv_frozen_suite(unit_l6_rotor_speed_sil)
