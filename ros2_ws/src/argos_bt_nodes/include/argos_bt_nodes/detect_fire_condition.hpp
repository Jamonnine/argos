#pragma once

// ============================================================
// detect_fire_condition.hpp
// 화재 감지 BT Condition 노드 헤더
//
// 역할: /fire_alerts 토픽(FireAlert)을 구독하여 온도 임계값
//       초과 여부를 BT 틱마다 평가한다.
//       - SUCCESS: 임계값(threshold) 이상 화점 감지
//       - FAILURE: 미감지 또는 토픽 수신 없음
//
// BT XML 사용 예:
//   <Condition ID="DetectFire"
//              threshold="873.15"
//              fire_pose="{fire_pose}"/>
// ============================================================

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "argos_interfaces/msg/fire_alert.hpp"

namespace argos_bt
{

class DetectFireCondition : public BT::ConditionNode
{
public:
  /**
   * @param condition_name  BT XML의 name 어트리뷰트
   * @param conf            BT NodeConfiguration (블랙보드 포함)
   */
  DetectFireCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~DetectFireCondition() override = default;

  /**
   * @brief BT 틱 핸들러 — 매 BT 주기마다 호출됨
   * @return SUCCESS(화점 감지) / FAILURE(미감지)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief BT XML 포트 정의
   *
   * 입력 포트:
   *   threshold  - 화점 판단 최소 온도 (켈빈, 기본값: 873.15 = 600°C)
   *
   * 출력 포트:
   *   fire_pose  - 감지된 화점의 map 프레임 PoseStamped
   *               (BT 블랙보드를 통해 ReportFire 노드에 전달)
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "threshold",
        873.15,
        "화점으로 판단할 최소 온도 (켈빈, 기본값 873.15 = 600°C)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "fire_pose",
        "감지된 화점의 map 프레임 좌표 (PoseStamped)")
    };
  }

private:
  // Nav2 BT Navigator가 블랙보드에 주입하는 공유 노드 핸들
  rclcpp::Node::SharedPtr node_;

  // /fire_alerts 토픽 구독자 (KeepLast 1 — 항상 최신 메시지만 유지)
  rclcpp::Subscription<argos_interfaces::msg::FireAlert>::SharedPtr sub_;

  // 마지막으로 수신한 FireAlert 메시지 (nullptr = 아직 수신 없음)
  argos_interfaces::msg::FireAlert::SharedPtr last_alert_;

  // 화점 판단 온도 임계값 (켈빈)
  double threshold_{873.15};
};

}  // namespace argos_bt
