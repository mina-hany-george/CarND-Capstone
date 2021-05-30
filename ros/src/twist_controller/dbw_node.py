#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from twist_controller import Controller

# Note: This flag is only for debugging
# It shouldnot be used in run-time/release
# As it causes high load on the system (many debug prints)
IS_DEBUG_ENABLE = 0

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # Get parameters otherwise revert to defaults
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        # Not really effective in simulation. It eventually affects car total mass 
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = rospy.get_param('~min_speed', 0.1)
        
        # Define variables needed
        self.current_velocity = None
        self.is_dbw_enabled = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.rate = 50 # 50Hz
        
        # variables that will be published by our node
        self.steer = 0
        self.throttle = 0
        self.brake = 0
        
        # Define the topics that will be published by the node (steering_cmd/throttle_cmd/brake_cmd)
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Subscribe to all the topics you need to
        # We need to subscribe to twist_cmd topic to get the car's velocity
        rospy.Subscriber('/twist_cmd',TwistStamped,self.twist_cmd_cb)
        # We need to subscribe to current_velocity topic to get the car's velocity
        rospy.Subscriber('/current_velocity',TwistStamped,self.current_velocity_cb)
        # We need to subscribe to dbw_enabled topic to know if the driver is in control or drive by wire is enabled
        rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_enabled_cb)
        
        # Create and Initialize `Controller` object
        self.controller = Controller(self.rate,
                                     wheel_base,
                                     steer_ratio,
                                     min_speed,
                                     max_lat_accel,
                                     max_steer_angle,
                                     wheel_radius,
                                     decel_limit,
                                     vehicle_mass,
                                     fuel_capacity)
        
        self.loop()

    def loop(self):
        # If the rate is below 10 Hz, the control will be given to the driver
        # This is done as a safety mechanism in case any software failures happen
        rate = rospy.Rate(self.rate) # 50Hz
        while not rospy.is_shutdown():
            # Sanity check to make sure we have values for current/linear/angular velocities
            # before passing it to the control function
            if not None in(self.current_velocity, self.linear_velocity, self.angular_velocity):
                # Call the control function to update throttle/brake/steering values using `twist_controller.py`
                self.throttle, self.brake, self.steering = self.controller.control(self.current_velocity,
                                                                                   self.is_dbw_enabled,
                                                                                   self.linear_velocity,
                                                                                   self.angular_velocity)
            
            # Publish the throttle/brake/steering values in case dbw_enabled is true
            # Otherwise the driver is in control now, therefore no need to publish any messages
            if self.is_dbw_enabled:
                # Publish throttle/brake/steering to their corresponding topics
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

#########################Call back Functions##############################
    # twist_cmd topic callback function
    def twist_cmd_cb(self,msg):
        # Update linear and angular velocities by the message received
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z
        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the linear/angular velocities
            rospy.logwarn("DBW_node: linear_velocity={}, angular_velocity={}".format(self.linear_velocity, self.angular_velocity))
    
    # current_velocity topic callback function
    def current_velocity_cb(self,msg):
        # Update the variable current_velocity by the message received
        self.current_velocity = msg.twist.linear.x
        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the current velocities
            rospy.logwarn("DBW_node: current_velocity={}".format(self.current_velocity))
    
    # /vehicle/dbw_enabled topic call back
    def dbw_enabled_cb(self,msg):
        # Update variable is_dbw_enabled to know if the car is controlled 
        # by wire or driver took over the vehicle
        self.is_dbw_enabled = msg
        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the current velocities
            rospy.logwarn("DBW_node: IS_DEBUG_ENABLE={}".format(IS_DEBUG_ENABLE))
##########################################################################
if __name__ == '__main__':
    DBWNode()
