cmake_minimum_required(VERSION 3.8)
project(pepper_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS src/nodes/cmd_vel_guard src/nodes/keyboard_teleop src/nodes/validate_base
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY validation DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(
  DIRECTORY
    src/launch
    src/config
    src/rviz
    src/urdf
    src/worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(base_metrics test/test_base_metrics.py TIMEOUT 30)
  ament_add_pytest_test(control_config test/test_control_config.py TIMEOUT 45)
  ament_add_pytest_test(teleop test/test_teleop.py TIMEOUT 30)
  ament_add_pytest_test(omni_kinematics test/test_omni_kinematics.py TIMEOUT 30)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
