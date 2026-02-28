ros2 launch nav2_bringup bringup_launch.py map:=/home/kuri-tadaoki/map/13hall_test2.yaml
<< COMMENTOUT
sleep 5 
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 1703243123
    nanosec: 731538851
  frame_id: 'map'
pose:
  pose:
    position:
      x: 0.031218737363815308
      y: 0.10198897123336792
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.22263768943469295
      w: 0.974901256150171
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
"
COMMENTOUT
