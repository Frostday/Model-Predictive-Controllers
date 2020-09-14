import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8, 8]
options['FULL_RECALCULATE'] = True

# drive the car from a start position to an end position quickest while driving under the speed limit

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2
        # every element in input array u will be remain for dt seconds
        # here with horizon 20 and dt 0.2 we will predict 4 seconds ahead(20*0.2)
        # we can't predict too much ahead in time because that might be pointless and take too much computational time
        # we can't predict too less ahead in time because that might end up overshooting from end point as it won't be able to see the end goal in time

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        a_t = pedal

        x_t_1 = x_t + v_t*dt  # distance = speed*time
        v_t_1 = v_t + a_t*dt - v_t/25.0  # v = u + at (-v_t/25 is a rough estimate for friction)

        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        x_end = ref[0]
        cost = 0.0

        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])
            # u input vector is designed like = [(pedal for t1), (steering for t1), (pedal for t2), (steering for t2)...... (pedal for t(horizon)), (steering for t(horizon))]
            x_current = state[0]
            v_current = state[3]

            cost += (x_end - x_current)**2  # so we keep getting closer to end point

            if v_current*3.6 > 10:  # speed limit is 10km/h, 3.6 is multiplied to convert m/s to km/h
                cost += 100*v_current

            if x_current > x_end:  # so we don't overshoot further than end point
                cost += 10000

        return cost

sim_run(options, ModelPredictiveControl)
