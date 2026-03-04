cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(remote_motor_fw)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_sources(app PRIVATE
  src/main.c
  src/bluetooth/bluetooth.c
  src/watchdog/watchdog.c
  src/motor/motor.c

  src/motor_control/motor_control.c
  src/motor_control/pid.c

)

if(CONFIG_MOTOR_SIM)
  target_sources(app PRIVATE src/simulation/bldc_driver_sim.c)   # MOCK SIMULATION: NO HARDWARE FOR APP/BLE TESTING + PoC
else()
  target_sources(app PRIVATE src/motor_control/bldc_driver.c)       # REAL BLDC DRIVER WITH TIM1 AND HALL ISR
endif()
