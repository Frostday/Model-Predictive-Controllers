import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 15
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        a_t = pedal
        steer_angle = steering

        x_t_1 = x_t + v_t*np.cos(psi_t)*dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*dt
        v_t_1 = v_t + a_t*dt - v_t/25.0
        psi_t_1 = psi_t + v_t*dt*(np.tan(steer_angle)/2.5)

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])

            cost += ((state[0] - ref[0])**2)
            cost += ((state[1] - ref[1])**2)

            cost += ((state[2] - ref[2])**2)

            dist = np.sqrt(((state[0] - self.x_obs)**2)+((state[1] - self.y_obs)**2))
            if dist < 2:
                cost += (1/dist)*30
            else:
                cost += 15

        return cost

sim_run(options, ModelPredictiveControl)
