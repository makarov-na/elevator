import numpy as np
from sim.elevator import sim_run

# Simulator Options
options = {}
options['FIG_SIZE'] = [8, 8]  # [Width, Height]
options['PID_DEBUG'] = False

# Physics Options
options['GRAVITY'] = True
options['FRICTION'] = True
options['ELEVATOR_MASS'] = 1000
options['COUNTERWEIGHT_MASS'] = 1000
options['PEOPLE_MASS'] = 0

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
        self.output = 0

        self.integr_err = 0
       
        self.kp = 0.2
        self.kd = 0.75
        self.ki = 0

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

        #self.integr_err = self.integr_err + self.current_err
        
        if self.prevent_err != None:
           
            diff_err = current_err - self.prevent_err
            d_out = self.kd*diff_err/dt
        else:
            d_out = 0

        #self.diff_err = self.prevent_err - self.current_err
        # if (abs(self.current_err) < 19):
        #    self.integr_err = 0
        # if (abs(self.current_err) == 19 and abs(self.prevent_err) == 19):
        #    self.diff_err = self.current_err
        self.output = p_out + d_out + self.ki*self.integr_err
        self.prevent_err = current_err

        # INSERT CODE ABOVE.
        return self.output


sim_run(options, Controller)
