import numpy as np
from sim.elevator import sim_run

# Simulator Options
options = {}
options['FIG_SIZE'] = [8, 8]  # [Width, Height]
options['PID_DEBUG'] = True

# Physics Options
options['GRAVITY'] = True
options['FRICTION'] = True
options['ELEVATOR_MASS'] = 1000
options['COUNTERWEIGHT_MASS'] = 1000
options['PEOPLE_MASS'] = 100

# Controller Options
options['CONTROLLER'] = True
options['START_LOC'] = 3.0
options['SET_POINT'] = 27.0
options['OUTPUT_GAIN'] = 2000


class Controller:
    def __init__(self, reference):
        self.target_value = reference
        self.prev_time = 0
        self.prevent_err = None
        self.prevent_diff_err = None
        self.output = 0
        self.output_max = 5

        self.integr_err = 0
        self.integration_windup = 5 

        self.kp = 2
        self.kd = 2.4
        self.ki = 0.2
        self.output_data = np.array([[0, 0, 0, 0]])

    def get_current_error(self):
        return self.current_err

    def run(self, current_value, t):
        # Controller run time.
        if t - self.prev_time < 0.01:
            return self.output

        dt = t - self.prev_time
        self.prev_time = t

        current_err = self.target_value - current_value

        p_out = self.kp*current_err

        diff_err = 0

        if self.prevent_err != None:
            diff_err = current_err - self.prevent_err
            d_out = self.kd*diff_err/dt
            
        else:
            d_out = 0

        print (dt)
        self.integr_err = self.integr_err + current_err*dt
        if self.integr_err * self.ki > self.output_max/self.integration_windup:
            self.integr_err = (self.output_max/self.integration_windup)/self.ki    

        i_out = self.ki*self.integr_err

        self.output = p_out + d_out + i_out
        self.prevent_err = current_err
        self.prevent_diff_err = diff_err

        if self.output > self.output_max:
            self.output = self.output_max
        if self.output_max < - self.output_max:
            self.output_max = - self.output_max

        
        self.output_data = np.concatenate((self.output_data,np.array([[t, p_out, i_out, d_out]])))

        # INSERT CODE ABOVE.
        return self.output


sim_run(options, Controller)
