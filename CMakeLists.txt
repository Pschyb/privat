cmake_minimum_required(VERSION 3.8)
project(rose2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Contour.msg"
  "msg/Edge.msg"
  "msg/ExtendedLine.msg"
  "msg/ROSE2Features.msg"
  "msg/ROSEFeatures.msg"
  "msg/Room.msg"
  "srv/ROSE.srv"
  "srv/ROSE2.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs visualization_msgs
)

# Keep algorithm code in a Python package separate from the generated
# rose2.msg/rose2.srv Python package.
ament_python_install_package(rose2_core)

install(PROGRAMS
  scripts/feature_extractor_rose
  scripts/feature_extractor_rose2
  scripts/receive_features
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(rose2_test_pythonpath "${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}")
  ament_add_pytest_test(
    test_grid_conversion test/test_grid_conversion.py
    APPEND_ENV "PYTHONPATH=${rose2_test_pythonpath}"
  )
  ament_add_pytest_test(
    test_node_imports test/test_node_imports.py
    APPEND_ENV "PYTHONPATH=${rose2_test_pythonpath}"
  )
endif()

ament_package()
