import numpy as np
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('../toolbox')
from robots_def import *
from utils import *
from lambda_calc import *
from pathlib import Path
import pickle
from baseline import *
#from pandas import *
import io
    
def motion_program_generation(curve_js, robot, total_seg):
    step=int((len(curve_js)-1)/total_seg)

    breakpoints = [0]
    primitives = ['movej_fit']
    q_bp = [[curve_js[0]]]
    p_bp = [[robot.fwd(curve_js[0]).p]]
    for i in range(step,len(curve_js),step):
        breakpoints.append(i)
        primitives.append('movej_fit')
        q_bp.append([curve_js[i]])
        p_bp.append([robot.fwd(curve_js[i]).p])

    return breakpoints,primitives,q_bp,p_bp

def main():

    opt_params_fname = Path(sys.argv[1])/ "motion_opt_input.pickle"

    with open(opt_params_fname,"rb") as f:
        opt_params = pickle.load(f)

    robot=abb6640(d=50)

    curve_js = opt_params["curve_js"]
    total_seg = opt_params["total_seg"]

    breakpoints,primitive_choices,q_bp,p_bp = motion_program_generation(curve_js, robot, total_seg)

    print(f"len(breakpoints): {len(breakpoints)}")
    print(f"len(primitive_choices): {len(primitive_choices)}")
    print(f"len(p_bp): {len(p_bp)}")
    print(f"len(q_bp): {len(q_bp)}")

    output = {
        "breakpoints": breakpoints,
        "primitive_choices": primitive_choices,
        "p_bp": p_bp,
        "q_bp": q_bp,
    }

    output_fname = Path(sys.argv[1])/ "motion_opt_output.pickle"

    with open(output_fname, "wb") as f:
        pickle.dump(output, f)

if __name__ == "__main__":
    main()