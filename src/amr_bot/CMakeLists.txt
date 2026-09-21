cmake_minimum_required(VERSION 3.8)
project(amr_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# MuJoCo ships as part of the pip wheel, not an ament package, so it's
# wired up with plain CMake rather than find_package/ament_target_dependencies.
set(MUJOCO_DIR /usr/local/lib/python3.10/dist-packages/mujoco)

add_executable(mujoco_test src/mujoco_test.cpp)

target_include_directories(mujoco_test PRIVATE
  ${MUJOCO_DIR}/include
)

# No libmujoco.so symlink ships in the wheel, only the versioned .so, so we
# link the exact file instead of -lmujoco.
target_link_libraries(mujoco_test
  ${MUJOCO_DIR}/libmujoco.so.3.11.0
)

# Bake the lib's location into the binary (RPATH) so `ros2 run` works from
# any shell without exporting LD_LIBRARY_PATH every session.
set_target_properties(mujoco_test PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH ${MUJOCO_DIR}
)

install(TARGETS mujoco_test
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
