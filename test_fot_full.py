from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import fot_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import argparse
from pathlib import Path

from numpy import array, float64

# Run fot planner
def fot(show_animation=False, show_info=False, num_threads=0, save_frame=False):

    initial_conditions = {
        "ps": 0,
        "target_speed": 0.2,
        "pos": array([0.10691724, 0.67502031]),
        "vel": array([0.02967398, 0.49911868]),
        "wp": array([[0.10691724, 0.67502031], [0.0327772, 20.49946837]]),
        "obs": array([[0, 3, 0.5, 3.5]], dtype=float64),
    }

    hyperparameters = {
        "max_speed": 5.0,
        "max_accel": 5.0,
        "max_curvature": 35.0,
        "max_road_width_l": 50.0,
        "max_road_width_r": 50.0,
        "d_road_w": 0.5,
        "dt": 0.5,
        "maxt": 5.0,
        "mint": 1.0,
        "d_t_s": 0.05,
        "n_s_sample": 2.0,
        "obstacle_clearance": 0.2,
        "kd": 0.1,
        "kv": 0.01,
        "ka": 0.01,
        "kj": 0.01,
        "kt": 0.01,
        "ko": 0.01,
        "klat": 0.1,
        "klon": 0.1,
        "num_threads": 2,  # set 0 to avoid using threaded algorithm
    }

    # static elements of planner
    wx = initial_conditions["wp"][:, 0]
    wy = initial_conditions["wp"][:, 1]
    obs = np.array(initial_conditions["obs"])

    # simulation config
    sim_loop = 200
    area = 7
    total_time = 0
    time_list = []
    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        (
            result_x,
            result_y,
            speeds,
            ix,
            iy,
            iyaw,
            d,
            s,
            speeds_x,
            speeds_y,
            misc,
            costs,
            success,
        ) = fot_wrapper.run_fot(initial_conditions, hyperparameters)
        end_time = time.time() - start_time
        print("Time taken: {}".format(end_time))
        total_time += end_time
        time_list.append(end_time)

        # reconstruct initial_conditions
        if success:
            initial_conditions["pos"] = np.array(
                [
                    result_x[1] + 0.01 * np.random.random(1),
                    result_y[1] + 0.01 * np.random.random(1),
                ]
            )
            initial_conditions["pos"] = np.array([result_x[1], result_y[1]])
            print(initial_conditions["pos"])
            initial_conditions["vel"] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions["ps"] = misc["s"]
            initial_conditions["wp"][0] = initial_conditions["pos"]
            if show_info:
                print(costs)
        else:
            print("Failed unexpectedly")
            break

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
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
            plt.title("v[m/s]:" + str(np.linalg.norm(initial_conditions["vel"]))[0:4])
            plt.grid(True)
            if save_frame:
                Path("img/frames").mkdir(parents=True, exist_ok=True)
                plt.savefig("img/frames/{}.jpg".format(i))
            plt.pause(0.1)

    print("Finish")

    print("======================= SUMMARY ========================")
    print("Total time for {} iterations taken: {}".format(i, total_time))
    print("Average time per iteration: {}".format(total_time / i))
    print("Max time per iteration: {}".format(max(time_list)))

    return time_list


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="show animation, ensure you have X11 forwarding server open",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="verbose mode, show all state info"
    )
    parser.add_argument(
        "-s", "--save", action="store_true", help="save each frame of simulation"
    )
    parser.add_argument(
        "-t", "--thread", type=int, default=0, help="set number of threads to run with"
    )
    args = parser.parse_args()

    # run planner with args passed in
    fot(args.display, args.verbose, args.thread, args.save)
