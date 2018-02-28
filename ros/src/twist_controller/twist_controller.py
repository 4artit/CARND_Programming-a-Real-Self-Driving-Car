from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args):
    	# for calcuate brake(torque[N/m])
        self.total_mass = args[4] + args[5] * GAS_DENSITY
        self.wheel_radius = args[6]

        self.yaw_controller = YawController(args[0], args[1], 0.15, args[2], args[3])
        self.pid = PID(0.0198, 0.0002, 0.0064, -1.0, 0.35)
        # tau = 0.01, ts = 50Hz -> 20ms
        self.lowpass_filter = LowPassFilter(0.01, 0.02)

    def control(self, *args):
        target_linear_vel = args[0]
        target_angular_vel = args[1]
        current_linear_vel = args[2]
        is_enable_dbw = args[3]
        throttle = 0.0
        brake = 0.0
        steer = 0.0
        if is_enable_dbw is False:
            self.pid.reset()
        else:
            steer = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, current_linear_vel)
            cte = ((target_linear_vel - current_linear_vel) / ONE_MPH)
            throttle = self.lowpass_filter.filt(self.pid.step(cte, 0.02))
            # if car need decel
            if throttle < 0.0:
                throttle = 0.0
                decel = (current_linear_vel - target_linear_vel) / 0.02
                # calculate about car braking torque in wheel
                brake = self.total_mass * decel * self.wheel_radius
        return throttle, brake, steer