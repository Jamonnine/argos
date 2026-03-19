// ============================================================
// report_fire_action.cpp
// 화점 보고 BT Action 노드 구현
//
// 설계 원칙:
//   - BtActionNode 상속: Action 전송/취소/타임아웃을 Nav2가 자동 처리
//   - on_tick()에서만 Goal 구성 (최초 틱 시 1회 실행)
//   - on_success()에서 accepted 필드 확인 → SUCCESS/FAILURE 결정
//   - 노드 자체에 비즈니스 로직 없음 — 오케스트레이터가 결정
//
// 연동: orchestrator_node.py의 /report_fire_alert Action Server
// ============================================================

#include "argos_bt_nodes/report_fire_action.hpp"

namespace argos_bt
{

ReportFireAction::ReportFireAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<argos_interfaces::action::ReportFireAlert>(
    xml_tag_name,
    action_name,
    conf)
{
}

void ReportFireAction::on_tick()
{
  // 블랙보드에서 DetectFire가 기록한 화점 좌표 읽기
  geometry_msgs::msg::PoseStamped fire_pose;
  getInput("fire_pose", fire_pose);

  // 보고 로봇 ID 읽기 (기본값: "ugv_0")
  std::string robot_id{"ugv_0"};
  getInput("robot_id", robot_id);

  // Action Goal 구성 — 필요한 필드만 채움 (얇은 노드 원칙)
  goal_.fire_location.header = fire_pose.header;
  goal_.fire_location.point.x = fire_pose.pose.position.x;
  goal_.fire_location.point.y = fire_pose.pose.position.y;
  goal_.fire_location.point.z = fire_pose.pose.position.z;
  goal_.robot_id = robot_id;

  // 온도/심각도는 블랙보드에서 직접 가져오는 대신
  // DetectFire가 출력한 fire_pose의 stamp를 기준으로 오케스트레이터가 매핑
  // 단순화: 임계값 기본값을 전달 (오케스트레이터에서 실제 FireAlert 참조)
  goal_.max_temperature_kelvin = 873.15f;
  goal_.severity = "high";
  goal_.confidence = 1.0f;

  RCLCPP_INFO(
    node_->get_logger(),
    "[ReportFire] 화점 보고 전송 — 로봇: %s, 위치: (%.2f, %.2f)",
    robot_id.c_str(),
    fire_pose.pose.position.x,
    fire_pose.pose.position.y);
}

BT::NodeStatus ReportFireAction::on_success()
{
  // Action 결과에서 수락 여부 확인
  if (result_.result->accepted) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReportFire] 보고 수락됨 — alert_id: %s",
      result_.result->alert_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // 오케스트레이터가 거절한 경우 (중복 보고 등)
  RCLCPP_WARN(
    node_->get_logger(),
    "[ReportFire] 보고 거절됨 — 메시지: %s",
    result_.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace argos_bt

// ============================================================
// BT 플러그인 등록 매크로
// detect_fire_condition.cpp와 동일한 BT_REGISTER_NODES를
// 두 번 정의하면 링크 오류 발생 → 별도 등록 불필요.
// 두 노드 모두 CMakeLists에서 같은 shared library로 빌드하고
// detect_fire_condition.cpp의 BT_REGISTER_NODES 안에
// 아래처럼 함께 등록:
//
//   factory.registerNodeType<argos_bt::DetectFireCondition>("DetectFire");
//   factory.registerNodeType<argos_bt::ReportFireAction>(
//     "ReportFire", "report_fire_alert");
//
// 단, BtActionNode 서브클래스는 생성자 시그니처가 달라
// registerNodeType 대신 registerBuilder 사용 → detect_fire_condition.cpp 참조
// ============================================================
