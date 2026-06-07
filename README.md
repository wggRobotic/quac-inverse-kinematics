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

### parameter
```
l1: String                # first segment
l2: String                # second segment
l3: String                # third segment
gripper_radius: double    # radius from joint to gripper parts
gripper_offset: double    # offset of gripper plates towards the middle
joint_names: String_array # names of arm and gripper joints
```
