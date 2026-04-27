# quac-inverse-kinematics
quac_inverse_kinematics package

## Summary

controls all the servos, so arm, gripper and flipper

## ik_node
### subscribers
- `ee_pose` : `geometry_msgs/msg/pose` target pose of the gripper
- `gripper_width` : `std_msgs/msg/float64` target width of how open the gripper is
- `flipper_pos` : `std_msgs/msg/float64` target angle of the flipper
### publishers
- `arm_joint_trajectory` : `trajectory_msgs/msg/joint_trajectory` target joint positions

### parameters
- `l1` : first segment
- `l2` : second segment
- `l3` : third segment
- `gripper_radius` : radius from joint to gripper parts
- `gripper_offset` : offset of gripper plates towards the middle
