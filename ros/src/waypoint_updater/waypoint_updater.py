#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about 
lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.


TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECELERATION = .5 
# Note: This flag is only for debugging
# It shouldnot be used in run-time/release
# As it causes high load on the system (many debug prints)
IS_DEBUG_ENABLE = 0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribe to current_pose topic to get car pose
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # Subscribe to base_waypoints topic to get the base way points
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # Subscribe to /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # Subscribe to obstacle_waypoint, it was optional so I did not implement it
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)
        # Publish the final_waypoints based on these values,
        # the subsciber node will generate and publish the twist_cmd topic
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Add other member variables we need
        self.waypoints_2d = None
        self.pose = None
        self.waypoint_tree = None
        self.stop_waypoint_indx = -1
        
        #Loop has been used to give us control over the publishing frequency
        self.loop()

    def loop(self):
        # Publishing frequency = 30 Hz
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                closest_waypoint_indx = self.get_closest_waypoint_indx()
                self.publish_waypoints(closest_waypoint_indx)
            rate.sleep()

    def get_closest_waypoint_indx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_indx = self.waypoint_tree.query([x,y],1)[1]
        
        # Check if the closest detected is ahead/behind the car
        closest_coordinates = self.waypoints_2d[closest_indx]
        previous_coordinates = self.waypoints_2d[closest_indx - 1]
        
        # Hyper plane equation
        closest_vector = np.array(closest_coordinates)
        previous_vector = np.array(previous_coordinates)
        pos_vector = np.array([x,y])
        
        val = np.dot(closest_vector-previous_vector , pos_vector-closest_vector)
        
        if val > 0:
            closest_indx = ((closest_indx + 1) % len(self.waypoints_2d))
        return  closest_indx
        
    def publish_waypoints(self, closest_indx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        closest_indx = self.get_closest_waypoint_indx()
        clipped_waypoints = self.base_waypoints.waypoints[closest_indx : closest_indx + LOOKAHEAD_WPS]
        
        if (self.stop_waypoint_indx == -1) or (self.stop_waypoint_indx >= (closest_indx + LOOKAHEAD_WPS)):
            # The traffic sign is far away for our farthest point so no need to change anything
            # Update the waypoints by clipping from closest_indx : closest_indx + LOOKAHEAD_WPS
            # This is done for better performance instead of passing the full list of points
            lane.waypoints = clipped_waypoints
        else : 
            lane.waypoints = self.decelerate_waypoints(clipped_waypoints, closest_indx)
        # Publish the final waypoints based on these values, new twist values will be published
        self.final_waypoints_pub.publish(lane)
    
    def decelerate_waypoints(self, waypoints, closest_index):
        # Use a temp list of waypoints as we do not want to mess with our base_waypoints
        # as it is ONLY sent ONCE
        waypoints_temp = []
        for indx, waypoint in enumerate(waypoints):
            # 4 was subtracted to prevent the front of the car from passing the stop line
            stop_indx = max(self.stop_waypoint_indx - closest_index - 4, 0)
            # Calculate the distance between our waypoint and the stop indx
            # This will be used in the equation of adjusting the velocity
            # As distance decreases (we are coming closer to our stop index point
            # the velocity needs to decrease too to eventually stop at the stop waypoint
            distance = self.distance(waypoints, indx, stop_indx)
            # Calculate new velocity after deceleration
            velocity = math.sqrt(2 * MAX_DECELERATION * distance) + (indx * (1.0 / LOOKAHEAD_WPS) )
            
            # Speed delimiter
            if velocity < 1:
                velocity = 0
        
            # Create new waypoint and initiaize it
            new_waypoint = Waypoint()
            # Keep the coordinate orientation of our waypoint the same
            new_waypoint.pose = waypoint.pose
            # Change velocity as the one obtained in the previous step
            # If the distance was big this may result in a high velocity which may be more than
            # the velocity assigned in the waypont itself so we choose the minimum velocity
            # so not to exceed velocity previously assigned for this waypoint
            new_waypoint.twist.twist.linear.x = min(velocity,waypoint.twist.twist.linear.x)
            # Append this new waypoint to our new list of decelerated waypoints 
            waypoints_temp.append(new_waypoint)
        
        return waypoints_temp

#########################Call back Functions##############################
    def pose_cb(self, msg):
        # Update the car position with the received one
        self.pose = msg
        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the car position X,Y
            rospy.logwarn("WayPoint Updater: X={}, Y={}".format(self.pose.pose.position.x, self.pose.pose.position.y))

    def waypoints_cb(self, waypoints):
        # Save the received waypoints to self.base_waypoints
        self.base_waypoints = waypoints
        
        if not self.waypoints_2d:
            self.waypoints_2d = [ [waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the base waypoints
            rospy.logwarn("WayPoint Updater: BaseWay points={}".format(self.base_waypoints))
    
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Save the stopping waypoint index we received from the traffic callback
        self.stop_waypoint_indx = msg.data
        if IS_DEBUG_ENABLE == 1:
            rospy.logwarn("WayPoint Updater: stop_waypoint_indx={}".format(self.stop_waypoint_indx))
        
    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. It was optional I did not implement it.
        pass
###########################Helper Functions###############################
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
##########################################################################
    
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
