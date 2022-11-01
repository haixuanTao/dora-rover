from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import fot_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import argparse
from pathlib import Path


# Run fot planner
def fot(show_animation=False,
        show_info=False,
        num_threads=0,
        save_frame=False):
    conds = {
        's0':
        0,
        'target_speed':
        4,
        'wp': [[0, 0], [5, 0]],
        'obs': np.empty((0, 4)),
        'pos': [0.14, 0.2],
        'vel': [1, 0],
    }  # paste output from debug log

    initial_conditions = {
        'ps': conds['s0'],
        'target_speed': conds['target_speed'],
        'pos': np.array(conds['pos']),
        'vel': np.array(conds['vel']),
        'wp': np.array(conds['wp']),
        'obs': np.array(conds['obs'])
    }

    hyperparameters = {
        "max_speed": 25.0,
        "max_accel": 15.0,
        "max_curvature": 30.0,
        "max_road_width_l": 5.0,
        "max_road_width_r": 5.0,
        "d_road_w": 0.5,
        "dt": 0.2,
        "maxt": 10.0,
        "mint": 2.0,
        "d_t_s": 0.5,
        "n_s_sample": 2.0,
        "obstacle_clearance": 0.1,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 0.1,
        "klat": 1.0,
        "klon": 1.0,
        "num_threads": num_threads,  # set 0 to avoid using threaded algorithm
    }

    wx = initial_conditions['wp'][:, 0]
    wy = initial_conditions['wp'][:, 1]
    obs = np.array(conds['obs'])

    # simulation config

    result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
            speeds_y, misc, costs, success = \
            fot_wrapper.run_fot(initial_conditions, hyperparameters)

        # reconstruct initial_conditions
    if success:
        if show_info:
            print(costs)
    else:
        print("Failed unexpectedly")
    print(result_x)
    print(result_y)
    area = 7
    if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None])
            plt.plot(wx, wy)
            if obs.shape[0] == 0:
                obs = np.empty((0, 4))
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title("v[m/s]:" +
                      str(np.linalg.norm(initial_conditions['vel']))[0:4])
            plt.grid(True)

            plt.pause(10)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="show animation, ensure you have X11 forwarding server open")
    parser.add_argument("-v",
                        "--verbose",
                        action="store_true",
                        help="verbose mode, show all state info")
    parser.add_argument("-s",
                        "--save",
                        action="store_true",
                        help="save each frame of simulation")
    parser.add_argument("-t",
                        "--thread",
                        type=int,
                        default=0,
                        help="set number of threads to run with")
    args = parser.parse_args()

    # run planner with args passed in
    fot(args.display, args.verbose, args.thread, args.save)