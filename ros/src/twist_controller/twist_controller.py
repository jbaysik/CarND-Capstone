from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        
        # setup yaw_controller for steering
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        # setup throttle controller for acceleration
        # pid coefficiants
        kp = 0.4
        ki = 0.001
        kd = 0.02
        
        #throttle min and max
        mn = 0.0
        mx = 0.2
        
        # PID for throttle to controls
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts) # use low-pass filter to cut-out noise in velocity
        
        # store car settings
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        # track ros u
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # if dbw status is False, stay idle/immobile
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        current_vel = self.vel_lpf.filt(current_vel) # use low-pass filter to filter out any noise in velocity
        
        #rospy.logwarn("Angular Veloctiy: {0}".format(angular_vel))
        #rospy.logwarn("Linear Veloctiy: {0}".format(linear_vel))
        #rospy.logwarn("Current Veloctiy: {0}".format(current_vel))
        
        # get steering amount from yaw controller
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # track velocity and error for throttle's PID
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        # track update times
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        # get throttle amount from twist_controller's PID
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        # physics calculations for braking
        if linear_vel == 0.0 and current_vel < 0.1:
            # keep brake set for stops
            throttle = 0
            brake = 400
            
        elif vel_error < 0:
            # decelerate for when moving faster than target (vel_error is negative)
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
            
        #rospy.logwarn("throttle {} vel_error {} brake {} cur_vel {} liner_vel {} steering {}".format(throttle, vel_error, brake, current_vel, linear_vel, steering))
           
        return throttle, brake, steering