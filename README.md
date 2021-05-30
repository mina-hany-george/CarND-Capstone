This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).


### Project Overview
For this project, I wrote ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following! 

I used a simulator that mimics the functionality on Carla.

I have attached images of system architecture and interaction between main nodes in folder "ros/architecture".

### Detailed Overview
The main ros packages/nodes are:
1. /ros/src/tl_detector/: contains the traffic light detection node "tl_detector.py" which takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

2. /ros/src/waypoint_updater/: contains the waypoint updater node "waypoint_updater.py". The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.

3. /ros/src/twist_controller/: contains the files that are responsible for control of the vehicle: the "node dbw_node.py" and the file "twist_controller.py", along with a pid and lowpass filter. The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.

4. waypoint_loader/: A package which loads the static waypoint data and publishes to /base_waypoints.

5. /ros/src/waypoint_follower/: A package containing code from Autoware which subscribes to /final_waypoints and publishes target vehicle linear and angular velocities in the form of twist commands to the /twist_cmd topic.

### Testing on Udacity's simulator

1. Make and run styx
```bash
cd /home/workspace/CarND-Capstone/ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
2. Run the simulator

### Future Improvements
1. PID hyper parameter values have been chosen by trial and error. Perhaps, I can use algorithms like Twiddle to fine tune these parameters.

2. Traffic Light Classifier to be later implemented.

3. Obstacle Detection to be later implemented.

### Useful References

The udacity project walkthroughs were very detailed and helpful:
https://classroom.udacity.com/nanodegrees/nd013/parts/b9040951-b43f-4dd3-8b16-76e7b52f4d9d/modules/85ece059-1351-4599-bb2c-0095d6534c8c/lessons/01cf7801-7665-4dc5-a800-2a9cca06b38b/concepts/e4ed7b44-6330-48a2-bfb0-fd65fff1b4d1

https://classroom.udacity.com/nanodegrees/nd013/parts/b9040951-b43f-4dd3-8b16-76e7b52f4d9d/modules/85ece059-1351-4599-bb2c-0095d6534c8c/lessons/01cf7801-7665-4dc5-a800-2a9cca06b38b/concepts/6546d82d-6028-4210-a4b0-9d559662a881

https://classroom.udacity.com/nanodegrees/nd013/parts/b9040951-b43f-4dd3-8b16-76e7b52f4d9d/modules/85ece059-1351-4599-bb2c-0095d6534c8c/lessons/01cf7801-7665-4dc5-a800-2a9cca06b38b/concepts/1776782c-5f60-4ada-b224-319cc61ef202

https://classroom.udacity.com/nanodegrees/nd013/parts/b9040951-b43f-4dd3-8b16-76e7b52f4d9d/modules/85ece059-1351-4599-bb2c-0095d6534c8c/lessons/01cf7801-7665-4dc5-a800-2a9cca06b38b/concepts/6e0119de-5a6f-4c62-a22b-a0659d0d235e

