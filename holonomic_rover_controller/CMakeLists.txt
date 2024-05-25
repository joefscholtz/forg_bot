cmake_minimum_required(VERSION 3.8)
project(holonomic_rover_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

generate_parameter_library(holonomic_rover_controller_parameters
  src/holonomic_rover_controller.yaml
)
add_library(
  holonomic_rover_controller
  SHARED
  src/holonomic_rover_controller.cpp
)
target_compile_features(holonomic_rover_controller PUBLIC c_std_99 cxx_std_17)

target_include_directories(holonomic_rover_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/steering_controllers_library>")

target_link_libraries(holonomic_rover_controller PUBLIC
  holonomic_rover_controller_parameters)

ament_target_dependencies(
  holonomic_rover_controller PUBLIC
  "ament_cmake"
  "control_msgs"
  "controller_interface"
  "generate_parameter_library"
  "geometry_msgs"
  "hardware_interface"
  "nav_msgs"
  "pluginlib"
  "rclcpp"
  "rclcpp_lifecycle"
  "realtime_tools"
  "tf2_msgs"
  "Eigen3"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(holonomic_rover_controller PRIVATE "HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_BUILDING_DLL" "_USE_MATH_DEFINES")

pluginlib_export_plugin_description_file(controller_interface holonomic_rover_controller.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # find_package(ament_cmake_gmock REQUIRED)
  # find_package(controller_manager REQUIRED)
  # find_package(ros2_control_test_assets REQUIRED)
  #
  # add_rostest_with_parameters_gmock(
  #   test_steering_controllers_library test/test_steering_controllers_library.cpp
  #   ${CMAKE_CURRENT_SOURCE_DIR}/test/steering_controllers_library_params.yaml)
  # target_include_directories(test_steering_controllers_library PRIVATE include)
  # target_link_libraries(test_steering_controllers_library steering_controllers_library)
  # ament_target_dependencies(
  #   test_steering_controllers_library
  #   controller_interface
  #   hardware_interface
  # )
  # ament_add_gmock(test_steering_odometry test/test_steering_odometry.cpp)
  # target_link_libraries(test_steering_odometry steering_controllers_library)

endif()

install(
  DIRECTORY include/
  DESTINATION include/holonomic_rover_controller
)
install(
  TARGETS holonomic_rover_controller holonomic_rover_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  holonomic_rover_controller
)
ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  "ament_cmake"
  "control_msgs"
  "controller_interface"
  "generate_parameter_library"
  "geometry_msgs"
  "hardware_interface"
  "nav_msgs"
  "pluginlib"
  "rclcpp"
  "rclcpp_lifecycle"
  "realtime_tools"
  "tf2_msgs"
  "Eigen3"
  "eigen3_cmake_module"
)

ament_package()
