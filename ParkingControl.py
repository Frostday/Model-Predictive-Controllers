import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8, 8]
options['OBSTACLES'] = False
options['FULL_RECALCULATE'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 15
        self.dt = 0.2
        # we are predicting dt*horizon seconds ahead

        # Reference or set point the controller will achieve.
        self.reference1 = [11, 11, 0]
        self.reference2 = [11, 1, 3*3.14/2]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        a_t = pedal
        steer_angle = steering

        x_t_1 = x_t + v_t*np.cos(psi_t)*dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*dt
        v_t_1 = v_t + a_t*dt - v_t/25.0
        psi_t_1 = psi_t + v_t*dt*(np.tan(steer_angle)/2.5)  # length of car is 2.5

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(self.horizon):
            v_start = state[3]

            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])
            # u input vector is designed like = [(pedal for t1), (steering for t1), (pedal for t2), (steering for t2)...... (pedal for t(horizon)), (steering for t(horizon))]

            cost += ((state[0] - ref[0])**2)*1.5
            cost += ((state[1] - ref[1])**2)*1.5
            # position cost

            cost += ((state[2] - ref[2])**2)
            # angle cost

            # cost += ((state[3] - v_start)**2)
            # accelaration cost - cost for changing accelaration quickly so passengers don't get motion sickness

            # cost += (u[i*2+1]**2)*self.dt
            # steering input cost - cost for moving the steering wheel

        return cost

sim_run(options, ModelPredictiveControl)
