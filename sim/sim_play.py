import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from scipy.optimize import minimize
import time


def plant_model(prev_state, dt, pedal, steering):
    x_t = prev_state[0]
    y_t = prev_state[1]
    psi_t = prev_state[2]
    v_t = prev_state[3]

    beta = steering
    a_t = pedal

    x_dot = v_t*np.cos(psi_t)
    y_dot = v_t*np.sin(psi_t)
    psi_dot = v_t * np.tan(beta)/2.5
    v_dot = (-v_t + a_t)/5.0

    x_t += x_dot*dt
    y_t += y_dot*dt
    psi_t += psi_dot*dt
    v_t += v_dot*dt
    return [x_t, y_t, psi_t, v_t]

def sim_run(options, RUN):
    start = time.process_time()
    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    OBSTACLES = options['OBSTACLES']

    run = RUN()

    num_inputs = 2
    bounds = []


    ref_1 = run.reference1
    ref_2 = run.reference2
    ref = ref_1

    state_i = np.array([[0,0,0,0]])
    u_i = np.array([[0,0]])
    sim_total = 250
    predict_info = [state_i]

    for i in range(1,sim_total+1):
        start_time = time.time()

        print('Step ' + str(i) + ' of ' + str(sim_total) + '   Time ' + str(round(time.time() - start_time,5)))
        [pedal, steering] = run.run(state_i[-1])

        # Bound inputs.
        pedal = max(min(pedal, 5), -5)
        steering = max(min(steering, 0.8), -0.8)

        y = plant_model(state_i[-1], run.dt, pedal, steering)
        if (i > 130 and ref_2 != None):
            ref = ref_2
        predicted_state = np.array([y])
        predict_info += [predicted_state]
        state_i = np.append(state_i, np.array([y]), axis=0)
        u_i = np.append(u_i, np.array([(pedal, steering)]), axis=0)


    ###################
    # SIMULATOR DISPLAY

    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(8,8)

    # Elevator plot settings.
    ax = fig.add_subplot(gs[:8, :8])

    plt.xlim(-3, 17)
    ax.set_ylim([-3, 17])
    plt.xticks(np.arange(0,11, step=2))
    plt.yticks(np.arange(0,11, step=2))
    plt.title('MPC 2D')

    # Time display.
    time_text = ax.text(6, 0.5, '', fontsize=15)

    # Main plot info.
    car_width = 1.0
    patch_car = mpatches.Rectangle((0, 0), car_width, 2.5, fc='k', fill=False)
    patch_goal = mpatches.Rectangle((0, 0), car_width, 2.5, fc='b',
                                    ls='dashdot', fill=False)

    ax.add_patch(patch_car)
    ax.add_patch(patch_goal)
    predict, = ax.plot([], [], 'r--', linewidth = 1)

    # Car steering and throttle position.
    telem = [3,14]
    patch_wheel = mpatches.Circle((telem[0]-3, telem[1]), 2.2)
    ax.add_patch(patch_wheel)
    wheel_1, = ax.plot([], [], 'k', linewidth = 3)
    wheel_2, = ax.plot([], [], 'k', linewidth = 3)
    wheel_3, = ax.plot([], [], 'k', linewidth = 3)
    throttle_outline, = ax.plot([telem[0], telem[0]], [telem[1]-2, telem[1]+2],
                                'b', linewidth = 20, alpha = 0.4)
    throttle, = ax.plot([], [], 'k', linewidth = 20)
    brake_outline, = ax.plot([telem[0]+3, telem[0]+3], [telem[1]-2, telem[1]+2],
                            'b', linewidth = 20, alpha = 0.2)
    brake, = ax.plot([], [], 'k', linewidth = 20)
    throttle_text = ax.text(telem[0], telem[1]-3, 'Forward', fontsize = 15,
                        horizontalalignment='center')
    brake_text = ax.text(telem[0]+3, telem[1]-3, 'Reverse', fontsize = 15,
                        horizontalalignment='center')

    # Speed Indicator
    speed_text = ax.text(telem[0]+7, telem[1], '0', fontsize=15)
    speed_units_text = ax.text(telem[0]+8.5, telem[1], 'km/h', fontsize=15)

    # Obstacles
    if OBSTACLES:
        patch_obs = mpatches.Circle((mpc.x_obs, mpc.y_obs),0.5)
        ax.add_patch(patch_obs)

    # Shift xy, centered on rear of car to rear left corner of car.
    def car_patch_pos(x, y, psi):
        #return [x,y]
        x_new = x - np.sin(psi)*(car_width/2)
        y_new = y + np.cos(psi)*(car_width/2)
        return [x_new, y_new]

    def steering_wheel(wheel_angle):
        wheel_1.set_data([telem[0]-3, telem[0]-3+np.cos(wheel_angle)*2],
                         [telem[1], telem[1]+np.sin(wheel_angle)*2])
        wheel_2.set_data([telem[0]-3, telem[0]-3-np.cos(wheel_angle)*2],
                         [telem[1], telem[1]-np.sin(wheel_angle)*2])
        wheel_3.set_data([telem[0]-3, telem[0]-3+np.sin(wheel_angle)*2],
                         [telem[1], telem[1]-np.cos(wheel_angle)*2])
        speed_text.set_x(telem[0]+7)
        speed_units_text.set_x(telem[0]+8.5)

    def update_plot(num):
        # Car.
        patch_car.set_xy(car_patch_pos(state_i[num,0], state_i[num,1], state_i[num,2]))
        patch_car.angle = np.rad2deg(state_i[num,2])-90
        # Car wheels
        steering_wheel(u_i[num,1]*2)
        throttle.set_data([telem[0],telem[0]],
                        [telem[1]-2, telem[1]-2+max(0,u_i[num,0]/5*4)])
        brake.set_data([telem[0]+3, telem[0]+3],
                        [telem[1]-2, telem[1]-2+max(0,-u_i[num,0]/5*4)])

        speed = state_i[num,3]*3.6
        speed_text.set_text(str(round(speed,1)))
        if speed > 10.1:
            speed_text.set_color('r')
        else:
            speed_text.set_color('k')

        # Goal.
        if (num <= 130 or ref_2 == None):
            patch_goal.set_xy(car_patch_pos(ref_1[0],ref_1[1],ref_1[2]))
            patch_goal.angle = np.rad2deg(ref_1[2])-90
        else:
            patch_goal.set_xy(car_patch_pos(ref_2[0],ref_2[1],ref_2[2]))
            patch_goal.angle = np.rad2deg(ref_2[2])-90

        #print(str(state_i[num,3]))
        predict.set_data(predict_info[num][:,0],predict_info[num][:,1])
        # Timer.
        #time_text.set_text(str(100-t[num]))

        return patch_car, time_text


    print("Compute Time: ", round(time.process_time() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, update_plot, frames=range(1,len(state_i)), interval=100, repeat=True, blit=False)
    #car_ani.save('mpc-video.mp4')

    plt.show()
