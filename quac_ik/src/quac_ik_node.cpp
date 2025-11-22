#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class QuacIKNode : public rclcpp::Node {
public:
  QuacIKNode() : Node("quac_ik_node") {
    this->declare_parameter<double>("l1", 0.105);
    this->declare_parameter<double>("l2", 0.165);
    this->declare_parameter<double>("l3", 0.065);

    ee_pos_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/ee_pos", 10,
        std::bind(&QuacIKNode::eeContactsCallback, this,
                  std::placeholders::_1));
    joint_commands_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_commands", 10);
  }

private:
  void eeContactsCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    ee_position_ = *msg;
    computeAndPublishJointAngles();
  }

  inline double safeAcos(double x) {
    return std::acos(std::clamp(x, -1.0, 1.0));
  }

  std::array<double, 2> inverseKinematics(double x, double /*y*/, double z) {
    std::array<double, 2> theta;

    const double l1 = this->get_parameter("l1").as_double();
    const double l2 = this->get_parameter("l2").as_double();
    const double l3 = this->get_parameter("l3").as_double();

    const double r = std::hypot(x, z);

    // ===== First angle =====
    const double cos_alpha =
        (l3 * l3 + r * r - l1 * l1 - l2 * l2) / (2.0 * l3 * r);

    theta[0] = std::atan2(z, x) + safeAcos(cos_alpha) + std::atan2(l2, l1);

    // ===== Second angle =====
    const double cos_beta =
        (l3 * l3 + l1 * l1 + l2 * l2 - r * r) / (2.0 * l3 * std::hypot(l1, l2));

    theta[1] = safeAcos(cos_beta) + std::atan2(l1, l2);

    return theta;
  }

  void computeAndPublishJointAngles() {
    // std::cout << "Computing joint angles" << std::endl;

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = this->now();

    trajectory_msgs::msg::JointTrajectoryPoint point;

    double x = ee_position_.position.x;
    double y = ee_position_.position.y;
    double z = ee_position_.position.z;

    std::array<double, 2> angles = inverseKinematics(x, y, z);

    if (std::any_of(angles.begin(), angles.end(),
                    [](double val) { return std::isnan(val); })) {
      RCLCPP_WARN(this->get_logger(), "IK failed for robot arm");
      return;
    }

    point.positions.push_back(angles[0]);
    point.positions.push_back(angles[1]);

    joint_trajectory.points.push_back(point);

    joint_commands_pub_->publish(joint_trajectory);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ee_pos_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      joint_commands_pub_;

  geometry_msgs::msg::Pose ee_position_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuacIKNode>());
  rclcpp::shutdown();
  return 0;
}