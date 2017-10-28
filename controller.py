import numpy as np
from sim.elevator import sim_run

# Simulator Options
options = {}
options['FIG_SIZE'] = [8, 8] # [Width, Height]
options['PID_DEBUG'] = False

# Physics Options
options['GRAVITY'] = True
options['FRICTION'] = True
options['MASS_RATIO'] = 0.9

# Controller Options
options['CONTROLLER'] = True
options['START_LOC'] = 3.0
options['SET_POINT'] = 27.0


class PidController:
    def __init__(self, reference):
        self.integral = 0
        self.prev_error = 0
        self.r = reference
        self.prev_time = 0
        self.finished = 0
        self.max_i = 2.5
        self.count = 0
        self.output = 0
        self.output_data = np.array([[0, 0, 0, 0]])
        self.output_min_max = True
        self.output_offset = True

    def run(self, x, t):
        kp = 1.5
        ki = 0.5
        kd = 2.5

        # Controller run time.
        if t - self.prev_time < 0.05:
            return self.output
        else:
            e = self.r - x

            dt = t - self.prev_time
            self.prev_time = t

            # Set Max I to Prevent Windup
            self.integral += e * dt
            if self.integral > self.max_i:
                self.integral = self.max_i
            if self.integral < -self.max_i:
                self.integral = -self.max_i

            P_out = kp * e
            I_out = ki * self.integral
            if dt > 0 and self.prev_error != 0:
                D_out = kd * (e - self.prev_error)/(dt)
            else:
                D_out = 0
            self.prev_error = e

            output = P_out + I_out + D_out

            if self.output_min_max:
                if output > 3.5:
                    output = 3.5
                if output < -3.5:
                    output = -3.5
            if self.output_offset:
                output += 9.8

            self.output = output
            self.output_data = np.concatenate((self.output_data, \
                np.array([[t, P_out, I_out, D_out]])))

            return self.output

sim_run(options, PidController)