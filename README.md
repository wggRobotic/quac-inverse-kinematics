# quac-inverse-kinematics
quac_inverse_kinematics package

## Summary

controls all the servos, so arm, gripper and flipper

## ik_node
### subscribers
- `ee_pose` : `geometry_msgs/msg/pose` target pose of the gripper
- `gripper_width` : `std_msgs/msg/float64` target width of how open the gripper is
### publishers
- `arm_joint_trajectory` : `trajectory_msgs/msg/joint_trajectory` target joint positions

### parameter
```
gripper_radius: double    # radius from joint to gripper parts
gripper_offset: double    # offset of gripper plates towards the middle
gripper_joint: String     # name of gripper joint

segment_count: int        # count of arm segments

segment_<n>:              # for the nth arm segment
    x: double
    y: double
    joint: String
```
