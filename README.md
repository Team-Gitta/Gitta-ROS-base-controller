<p>**Base Controller** that make Gitta robot to move with (Vx, Vy, Vtheta)</p> 
-

# How to build
```
# Make a your catkin workspace
$ mkdir -p [YOUR WS]/src
$ cd [YOUR WS]/src

# Clone this package
$ git clone https://github.com/Team-Gitta/Gitta-ROS-base-controller.git

$ cd [YOUR WS]
$ catkin_make
```

# How to launch
```
Todo!
```
# Subscribed topics
### cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
- velocity commands to drive the Gitta robot
