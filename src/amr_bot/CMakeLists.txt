cmake_minimum_required(VERSION 3.8)
project(amr_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# MuJoCo ships as part of the pip wheel, not an ament package, so it's
# wired up with plain CMake rather than find_package/ament_target_dependencies.
set(MUJOCO_DIR /usr/local/lib/python3.10/dist-packages/mujoco)

# No libmujoco.so symlink ships in the wheel, only the versioned .so, so we
# link the exact file instead of -lmujoco.
add_executable(mujoco_bridge src/mujoco_bridge.cpp)

target_include_directories(mujoco_bridge PRIVATE
  ${MUJOCO_DIR}/include
)

target_link_libraries(mujoco_bridge
  ${MUJOCO_DIR}/libmujoco.so.3.11.0
)

ament_target_dependencies(mujoco_bridge rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros)

set_target_properties(mujoco_bridge PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH ${MUJOCO_DIR}
)

install(TARGETS mujoco_bridge
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
