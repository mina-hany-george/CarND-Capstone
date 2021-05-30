#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

# Debounce counter to make sure the light is stable
# Not so important in simulation as I am not implementing Traffic classification 
STATE_COUNT_THRESHOLD = 3

# Note: This flag is only for debugging
# It shouldnot be used in run-time/release
# As it causes high load on the system (many debug prints)
IS_DEBUG_ENABLE = 0

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x : position to match a waypoint in x coordinate
            y : position to match a waypoint in y coordinate

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Use KDTree to get the closest waypoint
        return self.waypoint_tree.query([x,y],1)[1]

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # I did not implement light classifier it was optional To do as future improvements
        # I just return the light state provided from the simulator
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Gives the closest traffic light/line to our car
        closest_light_traffic = None
        line_waypoint_indx = None
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_waypoint_indx = self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y)
            
            #TODO find the closest visible traffic light (if one exists)
            # Initialize it with the maximum value at first
            diff = len(self.waypoints.waypoints)
            #Iterate through list containing traffic lights
            for indx, light in enumerate(self.lights):
                waypoint_indx_temp = self.get_closest_waypoint(stop_line_positions[indx][0],stop_line_positions[indx][1])
        
                # Get closest stopline waypoint index
                # If positive the waypoint is ahead of the car
                # If negative the waypoint is behind the car
                waypoint_indx_diff = waypoint_indx_temp - car_waypoint_indx
        
                if waypoint_indx_diff >= 0 and waypoint_indx_diff < diff:
                    # keep looping until we get the closest waypoint
                    diff = waypoint_indx_diff
                    closest_light_traffic = light
                    line_waypoint_indx = waypoint_indx_temp
        
        # In case we found a close traffic light
        if closest_light_traffic:
            state = self.get_light_state(closest_light_traffic)
        else:
            line_waypoint_indx = -1
            state = TrafficLight.UNKNOWN
        # For debugging
        rospy.logwarn("Detected Light: line_waypoint_indx={}, state={}".format(line_waypoint_indx, self.to_string(state)))
        return line_waypoint_indx, state
    
    def to_string(self, state):
        out = "unknown"
        if state == TrafficLight.GREEN:
            out = "green"
        elif state == TrafficLight.YELLOW:
            out = "yellow"
        elif state == TrafficLight.RED:
            out = "red"
        return out
#########################Call back Functions##############################
    def pose_cb(self, msg):
        self.pose = msg
        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the car position X,Y
            rospy.logwarn("tl_detector: X={}, Y={}".format(self.pose.pose.position.x, self.pose.pose.position.y))

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d =  [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        
        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the base waypoints
            rospy.logwarn("tl_detector: BaseWay points={}".format(self.waypoints))
    
    def traffic_cb(self, msg):
        self.lights = msg.lights
        
        if IS_DEBUG_ENABLE == 1:
            # For sake of debugging ensure we received the traffic_cb
            rospy.logwarn("tl_detector: traffic_cb={}".format(self.lights))
            
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        # keep counting state to ensure the state is stable (debouncing counter)
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
##########################################################################

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
