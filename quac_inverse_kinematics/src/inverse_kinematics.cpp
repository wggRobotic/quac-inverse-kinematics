#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <algorithm>

struct segment
{
  std::string joint;
  double x;
  double y;
};

class IKNode : public rclcpp::Node
{
public:

  IKNode() : Node("inverse_kinematics")
  {

    declare_parameter<double>("gripper_radius", 0.01);
    gripper_radius = get_parameter("gripper_radius").as_double();

    declare_parameter<double>("gripper_offset", 0.01);
    gripper_offset = get_parameter("gripper_offset").as_double();

    declare_parameter<std::string>("gripper_joint");
    gripper_joint = get_parameter("gripper_joint").as_string();

    declare_parameter<int>("segment_count", 3);
    int segment_count = get_parameter("segment_count").as_int();

    if (segment_count != 2 && segment_count != 3) throw std::runtime_error("segment_count has to be 2 or 3");
    segments.resize(segment_count);

    for (int i = 0; i < segment_count; i++)
    {
      std::string seg_name = "segment_" + std::to_string(i);

      declare_parameter<double>(seg_name + ".x", 0.1);
      segments[i].x = get_parameter(seg_name + ".x").as_double();

      declare_parameter<double>(seg_name + ".y", 0.01);
      segments[i].y = get_parameter(seg_name + ".y").as_double();

      declare_parameter<std::string>(seg_name + ".joint", "joint");
      segments[i].joint = get_parameter(seg_name + ".joint").as_string();
    }

    pose_subscriber = create_subscription<geometry_msgs::msg::Pose>(
      "ee_pose", 10,
      std::bind(&IKNode::pose_callback, this, std::placeholders::_1)
    );

    gripper_width_subscriber = create_subscription<std_msgs::msg::Float64>(
      "gripper_width", 10,
      std::bind(&IKNode::gripper_callback, this, std::placeholders::_1)
    );

    joint_trajectory_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_joint_trajectory", 
      10
    );
  }

private:

  inline double safeAsin(double x) { return std::asin(std::clamp(x, -1.0, 1.0)); }
  inline double safeAcos(double x) { return std::acos(std::clamp(x, -1.0, 1.0)); }

  void gripper_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = now();

    trajectory_msgs::msg::JointTrajectoryPoint point;

    double width = msg->data + 2. * gripper_offset;
    double angle = safeAsin(width / gripper_radius / 2);

    point.positions.push_back(angle);
    point.time_from_start.sec = 1;

    joint_trajectory.points.push_back(point);
    joint_trajectory.joint_names.push_back(gripper_joint);

    joint_trajectory_publisher->publish(joint_trajectory);
  }
  
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = now();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 2;

    double target_x = msg->position.x; double target_y = msg->position.z;

    double seg_0 = sqrt(segments[0].x * segments[0].x + segments[0].y * segments[0].y);
    double seg_1 = sqrt(segments[1].x * segments[1].x + segments[1].y * segments[1].y);
    double target = sqrt(target_x * target_x + target_y * target_y);

    if (seg_0 + seg_1 < target)
    {
      double factor = (seg_0 + seg_1) / target;
      target_x *= factor;
      target_y *= factor;
      target *= factor;
    }

    double angle_0 = atan2(target_y, target_x) + safeAcos((seg_0 * seg_0 + target * target - seg_1 * seg_1) / (2.0 * seg_0 * target));
    double angle_1 = safeAcos((seg_1 * seg_1 + seg_0 * seg_0 - target * target) / (2.0 * seg_0 * seg_1)) - M_PI;

    double joint_angle_0 = angle_0 - atan2(segments[0].y,  segments[0].x);
    double joint_angle_1 = angle_1 - atan2(segments[1].y,  segments[1].x) + atan2(segments[0].y,  segments[0].x);

    joint_trajectory.joint_names.push_back(segments[0].joint);
    joint_trajectory.joint_names.push_back(segments[1].joint);

    point.positions.push_back(joint_angle_0);
    point.positions.push_back(joint_angle_1);

    if (segments.size() == 3)
    {
      joint_trajectory.joint_names.push_back(segments[2].joint);
      point.positions.push_back(- joint_angle_0 - joint_angle_1 + msg->orientation.y);
    }

    joint_trajectory.points.push_back(point);
    joint_trajectory_publisher->publish(joint_trajectory);
  }

  double gripper_radius, gripper_offset;
  std::string gripper_joint;
  std::vector<segment> segments;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_width_subscriber;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();

  return 0;
}