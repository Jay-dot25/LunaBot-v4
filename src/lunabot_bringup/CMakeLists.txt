cmake_minimum_required(VERSION 3.8)
project(lunabot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/dashboard_telemetry_adapter.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/dashboard_control_gateway.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/dashboard_telemetry.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/dashboard_control_policy.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/dashboard/adapter.json
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/dashboard/controls.json
  DESTINATION share/${PROJECT_NAME}/config/dashboard
)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    lunabot_packaging_contract
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_packaging_contract.py
    TIMEOUT 180
  )
endif()
ament_package()
