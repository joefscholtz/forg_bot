cmake_minimum_required(VERSION 3.8)
project(mimic_joint_plugin_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  mimic_joint_plugin_gazebo
  SHARED
  src/mimic_joint_plugin.cpp
)
target_compile_features(mimic_joint_plugin_gazebo PUBLIC c_std_99 cxx_std_17)

target_include_directories(mimic_joint_plugin_gazebo PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/steering_controllers_library>")

ament_target_dependencies(
  mimic_joint_plugin_gazebo PUBLIC
  "gazebo_ros"
  "rclcpp"
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

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
endif()

install(
  DIRECTORY include/
  DESTINATION include/mimic_joint_plugin_gazebo
)
install(
  TARGETS mimic_joint_plugin_gazebo
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  mimic_joint_plugin_gazebo
)
ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  "gazebo_ros"
  "rclcpp"
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/ament-env-hooks/mimic_joint_plugin_gazebo_plugin_path.dsv.in")

ament_package()
