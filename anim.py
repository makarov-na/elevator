import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import odeint

def update_plot(num, data, data2, line, line2):
    line.set_data(data[:,:int(num/10)])
    line2.set_data(20,state[num,0])

    time_text.set_text(str(num/100))

    return line, line2, time_text,



def ElevatorPhysics(state, t):
    # State vector.
    x = state[0]
    x_dot = state[1]

    # Mass of elevator in kg.
    m = 100.0
    # Acceleration of gravity.
    g = -9.8

    x_dot_dot = g

    # Output state derivatives.
    return [x_dot, x_dot_dot]


state_initial = [50.0, 0.0]
t = np.arange(0.0, 50.0, 0.01)

state = odeint(ElevatorPhysics, state_initial, t)

data = np.array([np.ones(51)*39, np.arange(51)])



el_width = 10
el_height = 15
el_start = np.array([15, 35])
elevator = np.array([[0, el_width, el_width, 0, 0],
                      [0, 0, el_height, el_height, 0]])

elevator[0] += el_start[0]
elevator[1] += el_start[1]

fig1 = plt.figure(figsize=(4,8))

ax = fig1.add_subplot(111)
time_text = ax.text(.5, .5, '', fontsize=15)


l, l2, = plt.plot([], [], 'r-', [], [], 'k^')

plt.xlim(0, 40)
plt.ylim(0, 50)
#plt.xlabel('x')
plt.title('test')

elevator_ani = animation.FuncAnimation(fig1, update_plot, int(5.02*100), fargs=(data, state, l, l2),
    interval=10, repeat = False, blit=True)
#line_ani.save('lines.mp4')

plt.show()
