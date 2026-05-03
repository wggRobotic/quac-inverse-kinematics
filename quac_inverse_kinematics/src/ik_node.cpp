#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <algorithm>

class QuacIKNode : public rclcpp::Node {
public:
  QuacIKNode() : Node("ik_node") {
    declare_parameter<double>("l1", 0.1);
    declare_parameter<double>("l2", 0.1);
    declare_parameter<double>("l3", 0.1);
    declare_parameter<double>("gripper_radius", 0.01);
    declare_parameter<double>("gripper_offset", 0.01);

    ee_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "ee_pose", 10,
      std::bind(&QuacIKNode::eeContactsCallback, this, std::placeholders::_1)
    );

    gripper_sub_ = create_subscription<std_msgs::msg::Float64>(
      "gripper_width", 10,
      std::bind(&QuacIKNode::gripperCallback, this, std::placeholders::_1)
    );

    flipper_sub_ = create_subscription<std_msgs::msg::Float64>(
      "flipper_pos", 10,
      std::bind(&QuacIKNode::flipperCallback, this, std::placeholders::_1)
    );

    joint_commands_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_joint_trajectory", 
      10
    );
  }

private:
  void eeContactsCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    ee_position_ = *msg;
    computeAndPublishJointAngles();
  }

  void flipperCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = now();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back( msg->data);
    point.time_from_start.sec = 0.5;

    joint_trajectory.points.push_back(point);
    joint_trajectory.joint_names.push_back("flipper_servo_joint");

    joint_commands_pub_->publish(joint_trajectory);
  }

  inline double safeAsin(double x) {
    return std::asin(std::clamp(x, -1.0, 1.0));
  }

  void gripperCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = now();

    trajectory_msgs::msg::JointTrajectoryPoint point;

    double width = msg->data + 2. * get_parameter("gripper_offset").as_double();
    double angle = safeAsin(width / get_parameter("gripper_radius").as_double());

    point.positions.push_back(angle);
    point.time_from_start.sec = 0.5;

    joint_trajectory.points.push_back(point);
    joint_trajectory.joint_names.push_back("gripper_servo_joint");

    joint_commands_pub_->publish(joint_trajectory);
  }

  inline double safeAcos(double x) {
    return std::acos(std::clamp(x, -1.0, 1.0));
  }

  std::array<double, 2> inverseKinematics(double x, double /*y*/, double z) {
    std::array<double, 2> theta;

    const double l1 = get_parameter("l1").as_double();
    const double l2 = get_parameter("l2").as_double();
    const double l3 = get_parameter("l3").as_double();

    const double r = std::hypot(x, z);
    const double l12 = std::hypot(l1, l2);

    // ===== First angle =====
    const double cos_alpha = (l12*l12 + r*r - l3 * l3) / (2.0 * l12 * r);

    theta[0] = std::atan2(z, x) + safeAcos(cos_alpha) + std::atan2(l2, l1) - M_PI;

    // ===== Second angle =====
    const double cos_beta = (l3 * l3 + l12 * l12 - r * r) / (2.0 * l3 * l12);

    theta[1] = safeAcos(cos_beta) + std::atan2(l1, l2) - M_PI / 2.0;

    return theta;
  }

  void computeAndPublishJointAngles() {
    // std::cout << "Computing joint angles" << std::endl;

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = now();

    trajectory_msgs::msg::JointTrajectoryPoint point;

    double x = ee_position_.position.x;
    double y = ee_position_.position.y;
    double z = ee_position_.position.z;

    std::array<double, 2> angles = inverseKinematics(x, y, z);

    if (std::any_of(angles.begin(), angles.end(),
                    [](double val) { return std::isnan(val); })) {
      RCLCPP_WARN(get_logger(), "IK failed for robot arm");
      return;
    }

    point.positions.push_back(angles[0]);
    point.positions.push_back(angles[1]);
    point.time_from_start.sec = 2;

    joint_trajectory.points.push_back(point);
    joint_trajectory.joint_names.push_back("arm_servo_0_joint");
    joint_trajectory.joint_names.push_back("arm_servo_1_joint");

    joint_commands_pub_->publish(joint_trajectory);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ee_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr flipper_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_pub_;

  geometry_msgs::msg::Pose ee_position_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuacIKNode>());
  rclcpp::shutdown();
  return 0;
}