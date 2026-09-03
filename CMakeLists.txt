cmake_minimum_required(VERSION 3.8)
project(teleop_twist_gamepad)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_pytest_test(test_config_contract test/test_config_contract.py)
  ament_add_pytest_test(test_installation test/test_installation.py)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)

  # ament_lint_auto cannot pass the package Flake8 configuration.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8)
  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  # Register the remaining linters declared as test dependencies in package.xml.
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
