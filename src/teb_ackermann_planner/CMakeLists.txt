cmake_minimum_required(VERSION 3.8)
project(teb_ackermann_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---------------------------------------------------------------------------
# g2o / Eigen：TEB 轨迹优化后端
# g2o 不走 rosdep，直接用系统安装的 CMake 配置包（同仓库根目录 demo 的方式）
# ---------------------------------------------------------------------------
# 注意：g2o 安装出的配置文件名为 g2oConfig.cmake，find_package 的包名必须用小写 g2o
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(teb_ackermann_node
  src/teb_ackermann_node.cpp
  src/planner_manager.cpp
  src/ackermann_controller.cpp)
target_include_directories(teb_ackermann_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teb_ackermann_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  teb_ackermann_node
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs"
)

target_link_libraries(teb_ackermann_node
  g2o::core
  g2o::stuff
  Eigen3::Eigen)

install(TARGETS teb_ackermann_node
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

  # -------------------------------------------------------------------------
  # TEB 核心单元测试：进程内合成参考路径/障碍点，直接调用 plannerManager，
  # 不依赖 ROS 话题；复用 src/planner_manager.cpp（只额外链接 g2o/Eigen）
  # -------------------------------------------------------------------------
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_teb_planner
    test/test_teb_planner.cpp
    src/planner_manager.cpp)
  target_include_directories(test_teb_planner PRIVATE include)
  target_compile_features(test_teb_planner PRIVATE cxx_std_17)
  target_link_libraries(test_teb_planner
    g2o::core
    g2o::stuff
    Eigen3::Eigen)
endif()

ament_package()
