cmake_minimum_required(VERSION 3.8)
project(prototyping-project2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Python ROS 2 라이브러리 추가
find_package(nav_msgs REQUIRED)  # OccupancyGrid 메시지 사용을 위한 의존성 추가

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


# Python 실행 파일 설치
install(
  PROGRAMS
    src/PotentialFieldVisualizer.py  # Python 스크립트 경로
    src/SpawnObjectNode.py
    src/BoxObject.py
  DESTINATION lib/${PROJECT_NAME}      # 실행 파일 설치 경로
)

install(DIRECTORY launch src
DESTINATION share/${PROJECT_NAME}
)

ament_package()


#ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/nav2_params.yaml map:=/path/to/my_map.yaml
