cmake_minimum_required(VERSION 3.8)
project(pmw3901_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# libgpiod
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

include_directories(
  include
  ${LIBGPIOD_INCLUDE_DIRS}
)


# pmw3901
add_library(pmw3901_lib src/pmw3901.cpp)
target_include_directories(pmw3901_lib PUBLIC include)
target_link_libraries(pmw3901_lib ${LIBGPIOD_LIBRARIES})

# pmw3901_motion
add_executable(pmw3901_motion src/pmw3901_motion.cpp)
ament_target_dependencies(pmw3901_motion rclcpp geometry_msgs)
target_link_libraries(pmw3901_motion pmw3901_lib)

# pmw3901_frame_grab
add_executable(pmw3901_frame_grab src/pmw3901_frame_grab.cpp)
ament_target_dependencies(pmw3901_frame_grab rclcpp sensor_msgs)
target_link_libraries(pmw3901_frame_grab pmw3901_lib)
install(TARGETS
  pmw3901_motion
  pmw3901_frame_grab
  DESTINATION lib/${PROJECT_NAME})

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
