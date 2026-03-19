#pragma once

// ============================================================
// report_fire_action.hpp
// 화점 보고 BT Action 노드 헤더
//
// 역할: DetectFire가 블랙보드에 기록한 fire_pose를 읽어
//       /report_fire_alert Action Server에 비동기 전송한다.
//       nav2_behavior_tree::BtActionNode 패턴 사용.
//
// BT XML 사용 예:
//   <Action ID="ReportFire"
//           fire_pose="{fire_pose}"
//           server_name="/report_fire_alert"
//           server_timeout="5000"/>
//
// 상태 전이:
//   Action Server 응답 대기 중 → RUNNING
//   수락 응답(accepted=true)   → SUCCESS
//   거절 또는 타임아웃          → FAILURE
// ============================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "argos_interfaces/action/report_fire_alert.hpp"

namespace argos_bt
{

// BtActionNode 템플릿 특수화: ReportFireAlert action 타입으로 고정
class ReportFireAction
  : public nav2_behavior_tree::BtActionNode<
      argos_interfaces::action::ReportFireAlert>
{
public:
  using Action = argos_interfaces::action::ReportFireAlert;
  using ActionGoal = Action::Goal;

  /**
   * @param xml_tag_name  BT XML의 ID 어트리뷰트
   * @param action_name   ROS 2 Action Server 이름
   * @param conf          BT NodeConfiguration (블랙보드 포함)
   */
  ReportFireAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  ~ReportFireAction() override = default;

  /**
   * @brief Goal 메시지 구성 — tick() 최초 호출 시 1회 실행
   *
   * 블랙보드의 fire_pose를 읽어 ActionGoal을 채운다.
   * robot_id는 ros__namespace 파라미터에서 추론.
   */
  void on_tick() override;

  /**
   * @brief Action 성공 시 처리 — accepted 필드로 최종 결과 판단
   * @return SUCCESS(accepted=true) / FAILURE(accepted=false)
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief BT XML 포트 정의
   *
   * 입력 포트:
   *   fire_pose  - DetectFire가 기록한 화점 PoseStamped
   *   robot_id   - 보고 로봇 ID (선택, 기본값: "ugv_0")
   */
  static BT::PortsList providedPorts()
  {
    // BtActionNode의 기본 포트(server_name, server_timeout, goal, result, feedback)를 상속
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "fire_pose",
          "DetectFire 노드가 블랙보드에 기록한 화점 좌표"),
        BT::InputPort<std::string>(
          "robot_id",
          "ugv_0",
          "보고 로봇 식별자 (기본값: ugv_0)")
      });
  }
};

}  // namespace argos_bt
