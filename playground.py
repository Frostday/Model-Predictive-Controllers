import numpy as np
from sim.sim_play import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class Run:
    def __init__(self):
        self.dt = 0.2
        # Reference or set point the controller will achieve.
        self.reference1 = [10, 4, 1.5]
        self.reference2 = None  # [10, 2, 3.14/2]

    def run(self, current_state):
        x_t = current_state[0] # X Location [m]
        y_t = current_state[1] # Y Location [m]
        psi_t = current_state[2] # Angle [rad]
        v_t = current_state[3] # Speed [m/s]
        pedal = 0 # Max: 5, Min: -5
        steering = 0 # Max: 0.8, Min: -0.8

        if x_t > self.reference1[0]:
            pedal=-1
        else:
            pedal=2

        if v_t < 0:
            pedal=0

        if y_t > self.reference1[1]:
            pedal=-1

        if psi_t < self.reference1[2]:
            steering=0.2
        else:
            steering=-0.2

        return [pedal, steering]

sim_run(options, Run)
