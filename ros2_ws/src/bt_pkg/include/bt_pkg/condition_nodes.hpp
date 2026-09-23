#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include "bt_pkg/scene_data.hpp"

namespace bt_pkg {

// ─────────────────────────────────────────────────────────────────────────────
// TargetVisible
// Checks that /yolo/target_centroid has been published within staleness_sec
// AND the live centroid is within search_radius_m of the initial target centroid
// stored on the blackboard ("target_centroid" key).
// SUCCESS = target still where we expect it.
// FAILURE = target lost or hasn't been seen recently → triggers Pick Recovery.
// ─────────────────────────────────────────────────────────────────────────────
class TargetVisible : public BT::ConditionNode {
public:
  TargetVisible(const std::string& name, const BT::NodeConfig& config,
                std::shared_ptr<SceneData> scene,
                rclcpp::Clock::SharedPtr clock,
                double search_radius_m,
                double staleness_sec);

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<SceneData> scene_;
  rclcpp::Clock::SharedPtr   clock_;
  double search_radius_m_;
  double staleness_sec_;
};

// ─────────────────────────────────────────────────────────────────────────────
// HasDestinationCentroid
// Place-phase gate. Reads the blackboard flag ParseScene sets.
// Keys on the measured CENTROID, not destination_spec — "put it in the box"
// with no box in view populates the spec but leaves the centroid absent, and a
// spec-keyed guard would then place at {0,0,0}, the robot's own base.
// ─────────────────────────────────────────────────────────────────────────────
class HasDestinationCentroid : public BT::ConditionNode {
public:
  HasDestinationCentroid(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

}  // namespace bt_pkg
