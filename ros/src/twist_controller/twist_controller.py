
import rospy
from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
MAX_BRAKE_VALUE = 700.0
# Note: This flag is only for debugging
# It shouldnot be used in run-time/release
# As it causes high load on the system (many debug prints)
IS_DEBUG_ENABLE = 0

class Controller(object):
    def __init__(self, rate,
                       wheel_base,
                       steer_ratio,
                       min_speed,
                       max_lat_accel,
                       max_steering_angle,
                       wheel_radius,
                       decel_limit,
                       vehicle_mass,      
                       fuel_capacity):
        
        # Rate = 50Hz ==> sample_time = 1/50 ==> 0.02 sec
        self.sample_time = 1.0/rate
        self.lp_tau = 0.5 # Cut off frequency = 1/(2pi*lp_tau)
        
        # Configure a low pass filter for the velocity
        # As stated in the video walkthrough the simulator has some noise on velcoity values provided
        # The low pass filter is used to filter high frequency noise (sudden changes in velocity values)
        self.velocity_lowpass_filter = LowPassFilter(self.lp_tau,self.sample_time)
        
        # For brake calculation
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit
        
        # Parameters needed by Yaw Controller
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steering_angle = max_steering_angle
        
        # Yaw Controller
        self.yaw_controller = YawController(self.wheel_base,
                                            self.steer_ratio,
                                            self.min_speed,
                                            self.max_lat_accel,
                                            self.max_steering_angle)
        
        # Initialze PID hyper parameters obtained by trial and error
        K_prop = 0.35
        K_integral = 0.15
        K_diff = 0.0001
        min_throttle = 0
        max_throttle = 0.3
        
        # PID Controller Initialization
        self.throttle_controller = PID(K_prop, K_integral, K_diff, min_throttle, max_throttle)
        
        # Is not really effective in the simulation gas mass can be ignored
        self.vehicle_mass = vehicle_mass + (GAS_DENSITY * fuel_capacity)        
        # Save the current timestamp
        self.last_timestamp = rospy.get_time()

    def control(self,
                current_velocity,
                is_dbw_enabled,
                linear_velocity,
                angular_velocity):
        
        # In case drive-y-wire is disabled, no need to run as the driver has full control on the vehicle
        # the PID will accumulate error if the driver is in control resulting in sudden changes
        # when is_dbw_enabled returns active again due to high accumulative error value
        if not is_dbw_enabled:
            
            # Clear the integral part to prevent accumulating error
            self.throttle_controller.reset()
            # Clear throttle, brake, steer as driver is in charge now
            throttle = 0.
            brake = 0.
            steering = 0.
        else:
            # Remove noise from velocity by running it over the lowpass filter
            current_velocity = self.velocity_lowpass_filter.filt(current_velocity)
        
            # Get error to pass it to the throttle PID controller
            velocity_error = linear_velocity - current_velocity
            # Get delta time to pass it to the throttle PID controller
            current_time_stamp = rospy.get_time()
            sample_time = current_time_stamp - self.last_timestamp
            # Save the time stamp for next time
            self.last_timestamp = current_time_stamp
        
            # Get throttle value from the PID controller
            throttle = self.throttle_controller.step(velocity_error,sample_time)
        
            # Get steering value from yaw controller
            steering = self.yaw_controller.get_steering(linear_velocity,
                                                       angular_velocity,
                                                       current_velocity)

            # Brake calculation
            brake = 0.0
            if linear_velocity == 0.0 and current_velocity < 0.1:
                throttle = 0.0
                brake = MAX_BRAKE_VALUE  # N*m - to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2
            elif throttle < 0.1 and velocity_error < 0:
                throttle = 0.0
                decelerate = max(velocity_error, self.decel_limit)
                brake = min(MAX_BRAKE_VALUE, (abs(decelerate) * self.vehicle_mass * self.wheel_radius))  # Torque N*m
        # Return throttle, brake, steer
        return throttle, brake, steering
