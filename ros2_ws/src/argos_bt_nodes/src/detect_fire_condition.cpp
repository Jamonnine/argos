// ============================================================
// detect_fire_condition.cpp
// 화재 감지 BT Condition 노드 구현
//
// 설계 원칙:
//   - 노드는 얇게 유지: 토픽 구독 + 임계값 비교만 수행
//   - 실제 열화상 처리는 Python hotspot_detector_node.py가 담당
//     (FireAlert 메시지로 변환하여 /fire_alerts에 발행)
//   - KeepLast(1) QoS: 항상 최신 감지 결과만 평가
// ============================================================

#include "argos_bt_nodes/detect_fire_condition.hpp"
#include "argos_bt_nodes/report_fire_action.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace argos_bt
{

DetectFireCondition::DetectFireCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  // Nav2 BT Navigator가 "node" 키로 블랙보드에 주입한 노드 핸들 취득
  // 이 핸들을 통해 토픽 구독 등 ROS 통신 수행
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // BT XML 입력 포트에서 임계값 로드 (없으면 기본값 873.15 K 사용)
  getInput("threshold", threshold_);

  // /fire_alerts 토픽 구독 설정
  // KeepLast(1): 최신 메시지 1개만 유지 — BT 틱 주기와 토픽 주기 불일치 허용
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  sub_ = node_->create_subscription<argos_interfaces::msg::FireAlert>(
    "fire_alerts",
    qos,
    [this](const argos_interfaces::msg::FireAlert::SharedPtr msg) {
      last_alert_ = msg;
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "[DetectFire] 초기화 완료 — 임계값: %.2f K (%.2f°C)",
    threshold_,
    threshold_ - 273.15);
}

BT::NodeStatus DetectFireCondition::tick()
{
  // 아직 FireAlert 메시지를 수신하지 못한 경우
  if (!last_alert_) {
    return BT::NodeStatus::FAILURE;
  }

  // 최고 온도가 임계값 이상인지 확인
  if (last_alert_->max_temperature_kelvin >= static_cast<float>(threshold_)) {
    // 화점 좌표(PointStamped)를 PoseStamped로 변환하여 블랙보드에 기록
    // ReportFire 노드가 이 값을 읽어 Action Goal로 사용
    geometry_msgs::msg::PoseStamped fire_pose;
    fire_pose.header = last_alert_->location.header;
    fire_pose.pose.position.x = last_alert_->location.point.x;
    fire_pose.pose.position.y = last_alert_->location.point.y;
    fire_pose.pose.position.z = last_alert_->location.point.z;
    // orientation은 identity (화점 방향 불명)
    fire_pose.pose.orientation.w = 1.0;

    setOutput("fire_pose", fire_pose);

    RCLCPP_INFO(
      node_->get_logger(),
      "[DetectFire] 화점 감지 — 온도: %.2f K (%.2f°C), 위치: (%.2f, %.2f), "
      "심각도: %s, 신뢰도: %.2f",
      static_cast<double>(last_alert_->max_temperature_kelvin),
      static_cast<double>(last_alert_->max_temperature_kelvin) - 273.15,
      last_alert_->location.point.x,
      last_alert_->location.point.y,
      last_alert_->severity.c_str(),
      static_cast<double>(last_alert_->confidence));

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace argos_bt

// ============================================================
// BT 플러그인 등록 매크로
// 공유 라이브러리(.so)당 BT_REGISTER_NODES는 반드시 1개만 정의.
// 이 블록에서 argos_bt_nodes.so의 모든 노드를 한 번에 등록.
//
// Nav2 BT Navigator는 plugin_lib_names에 "argos_bt_nodes"를
// 지정하면 이 함수를 호출하여 DetectFire, ReportFire를 factory에 등록.
// ============================================================
BT_REGISTER_NODES(factory)
{
  // DetectFire: ConditionNode — registerNodeType으로 단순 등록 가능
  factory.registerNodeType<argos_bt::DetectFireCondition>("DetectFire");

  // ReportFire: BtActionNode 서브클래스 — 생성자가 action_name 인수를
  // 추가로 받으므로 registerBuilder로 명시적 등록
  BT::NodeBuilder report_fire_builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      // action_name은 BT XML의 server_name 포트 값을 읽어
      // BtActionNode 생성자로 전달. 기본값: "/report_fire_alert"
      std::string server_name = "/report_fire_alert";
      config.blackboard->get("report_fire_server_name", server_name);
      return std::make_unique<argos_bt::ReportFireAction>(name, server_name, config);
    };

  factory.registerBuilder<argos_bt::ReportFireAction>("ReportFire", report_fire_builder);
}
