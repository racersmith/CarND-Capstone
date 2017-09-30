import rospy
from pid import PID
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):

        self.brake_deadband = brake_deadband
        # Maximum braking torque
        # F = m*a
        # T = F*r
        # T = m*a*r
        maximum_braking_torque = (vehicle_mass+fuel_capacity*GAS_DENSITY)*decel_limit*wheel_radius

        # Initialize PID controllers for throttle, brake and steering
        self.throttle_controller = PID(kp=0.2, kd=0.0, ki=1e-8, mn=0.0, mx=1.0)
        self.brake_controller = PID(kp=0.2, kd=0.0, ki=0.0, mn=brake_deadband, mx=maximum_braking_torque)
        self.steering_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # time_step value
        self.last_time = rospy.get_time()


    # Determine control commands from target velocities
    def control(self, linear_target, linear_current, angular_target, angular_current, dbw_enabled):
        # if dbw is not enabled don't let PID controllers windup.
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.brake_controller.reset()

        linear_error = linear_target - linear_current
        current_time = rospy.get_time()
        # step_time = current_time - self.last_time
        self.last_time = current_time
        step_time = 1./50.
        throttle_cmd = self.throttle_controller.step(linear_error, step_time)
        brake_cmd = self.brake_controller.step(-linear_error, step_time)
        if brake_cmd == self.brake_deadband:
            # no need to ride the brakes.
            brake_cmd = 0.0

        steering_cmd = self.steering_controller.get_steering(linear_current, angular_target, angular_current)

        # Return throttle, brake, steer
        return throttle_cmd, brake_cmd, steering_cmd
        # return 0.5, 0.0, 0.0
