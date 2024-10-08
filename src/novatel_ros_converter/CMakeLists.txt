cmake_minimum_required(VERSION 3.8)
project(novatel_ros_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- FIND DEPENDENCIES ---
find_package(ament_cmake_auto REQUIRED)  # automatically find dependencies
ament_auto_find_build_dependencies()
# pose to odometry node
ament_auto_add_executable(novatel_to_pose
  src/main_novatel_to_pose_node.cpp)
target_compile_features(novatel_to_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install executable(s)
install(TARGETS
novatel_to_pose
  DESTINATION lib/${PROJECT_NAME})

# install launchfile(s) [all within the "launch" folder]
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# --- SPECIFY TESTS ---
# configure clang format
set(ament_cmake_clang_format_CONFIG_FILE ../../.clang-format)

if(BUILD_TESTING)
  # add test config here
endif()

ament_package()
