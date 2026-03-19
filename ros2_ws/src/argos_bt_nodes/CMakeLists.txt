cmake_minimum_required(VERSION 3.8)
project(argos_bt_nodes)

# C++17 필수 (behaviortree_cpp v4 요구사항)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ============================================================
# 의존성 선언
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)        # BehaviorTree.CPP v4
find_package(nav2_behavior_tree REQUIRED)      # BtActionNode, BtConditionNode
find_package(argos_interfaces REQUIRED)        # FireAlert.msg, ReportFireAlert.action
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================================
# BT 플러그인 공유 라이브러리 빌드
#
# 라이브러리명 "argos_bt_nodes" →
#   nav2_params_sim.yaml의 plugin_lib_names: [argos_bt_nodes] 와 일치
#
# SHARED: .so로 빌드 (Nav2가 dlopen으로 런타임 동적 로드)
# ============================================================
add_library(argos_bt_nodes SHARED
  src/detect_fire_condition.cpp   # DetectFire + BT_REGISTER_NODES(전체 등록)
  src/report_fire_action.cpp      # ReportFire 구현
)

# 헤더 디렉토리 (빌드 시 + 설치 후 모두 적용)
target_include_directories(argos_bt_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ament 의존성 연결
ament_target_dependencies(argos_bt_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_behavior_tree
  argos_interfaces
  geometry_msgs
  std_msgs
)

# ============================================================
# 설치 규칙
# ============================================================

# .so 라이브러리 → lib/ (Nav2가 LD_LIBRARY_PATH에서 탐색)
install(
  TARGETS argos_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 헤더 파일 → include/ (다른 패키지가 참조할 경우 대비)
install(
  DIRECTORY include/
  DESTINATION include/
)

# ============================================================
# 테스트 (빌드 시 --cmake-args -DBUILD_TESTING=ON 선택 시 활성)
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
