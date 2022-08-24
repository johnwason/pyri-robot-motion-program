import sys
import os

import pickle
from pathlib import Path

import numpy as np
# from matplotlib.pyplot import *
# from mpl_toolkits.mplot3d.axes3d import Axes3D
# import matplotlib.pyplot as plt
# from pandas import *
# from fitting_toolbox import *
# from toolbox_circular_fit import *
from robots_def import *
# from general_robotics_toolbox import *
# from error_check import *
# from lambda_calc import *
from greedy import greedy_fit

def main():
    print("Running greedy_fitting_opt.py script")

    opt_params_fname = Path(sys.argv[1])/ "greedy_fitting_opt_input.pickle"

    with open(opt_params_fname,"rb") as f:
        opt_params = pickle.load(f)

    print(f"input rox_robot.P: {opt_params['rox_robot'].P}")
    print(f"input rox_robot.H: {opt_params['rox_robot'].H}")
    print(f"input rox_tool_pose.R: {opt_params['rox_tool_pose'].R}")
    print(f"input rox_tool_pose.P: {opt_params['rox_tool_pose'].p}")
    print(f"input trajectory_format: {opt_params['trajectory_format']}")
    print(f"input input_trajectory.shape: {opt_params['input_trajectory'].shape}")

    assert opt_params['trajectory_format'] == "trajectory-data-format-joints6", "Trajectory data must be in Joints 6 axis (q1,q2,q3,q4,q5,q6) format"

    curve_js = opt_params['input_trajectory']
    max_error_threshold = opt_params['max_error_threshold']

    # TODO: use input robot kinematics
    robot=abb6640(d=50)

    greedy_fit_obj=greedy_fit(robot,curve_js,max_error_threshold)

    greedy_fit_obj.primitives={'movel_fit':greedy_fit_obj.movel_fit_greedy,'movej_fit':greedy_fit_obj.movej_fit_greedy}

    breakpoints,primitives_choices,points,q_bp=greedy_fit_obj.fit_under_error()

    print('slope diff js (deg): ', greedy_fit_obj.get_slope_js(greedy_fit_obj.curve_fit_js,breakpoints))

    ############insert initial configuration#################
    primitives_choices.insert(0,'movej_fit')
    points.insert(0,[greedy_fit_obj.curve_fit[0]])
    q_bp.insert(0,[greedy_fit_obj.curve_fit_js[0]])

    print(f"len(breakpoints): {len(breakpoints)}")
    print(f"len(primitives_choices): {len(primitives_choices)}")
    print(f"len(points): {len(points)}")
    print(f"len(q_bp): {len(q_bp)}")

    output = {
        "breakpoints": breakpoints,
        "primitive_choices": primitives_choices,
        "points": points,
        "q_bp": q_bp,
        "curve_fit": greedy_fit_obj.curve_fit,
        "curve": greedy_fit_obj.curve,
        "curve_fit_js": greedy_fit_obj.curve_fit_js
    }

    output_fname = Path(sys.argv[1])/ "greedy_fitting_opt_output.pickle"

    with open(output_fname, "wb") as f:
        pickle.dump(output, f)

if __name__ == "__main__":
    main()

