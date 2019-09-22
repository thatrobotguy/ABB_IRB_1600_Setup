
# Notes on debugging frame transforms
When we want to look at all the frame transforms that make up the tf tree, we need to run this command:
```
rosrun tf view_frames
```

When you want to compare 2 different frame transforms, from the base frame to the child frame, run this command:
```
rosrun tf tf_echo [reference_frame] [target_frame]
```
# Example Frame Transforms
With the new ROS node, this is an example of the new publish comamnd for the ABB IRB1600 1.45 6kg:
```
rostopic pub /do_arm_traj geometry_msgs/Pose "position:
  x: 0.510
  y: -0.510
  z: 0.750
orientation:
  x: 0.50
  y: -0.50
  z: 0.50
  w: 0.50"
```

This is also a valid pose for the 1600 1.45 robot:
```
rostopic pub /do_arm_traj geometry_msgs/Pose "position:
  x: 0.510
  y: -0.510
  z: 0.750
orientation:
  x: -0.50
  y: 0.50
  z: -0.50
  w: 0.50"
```
