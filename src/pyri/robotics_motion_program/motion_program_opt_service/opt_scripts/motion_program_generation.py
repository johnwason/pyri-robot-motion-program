import numpy as np
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_motion_program_opt.toolbox.robots_def import *
from robot_motion_program_opt.toolbox.utils import *
from robot_motion_program_opt.toolbox.lambda_calc import *
from robot_motion_program_opt.cmd_gen.cmd_gen import motion_program_generation_greedy
from pathlib import Path
import pickle
#from pandas import *
import io
from pyri.robotics_motion_program.motion_program_opt_service.opt_script_util import load_rox_robot_obj, rr_varvalue_to_bytes
from robot_motion_program_opt.toolbox.robotraconteur_mp_utils import MotionSendRobotRaconteurMP
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources

def main():

    RRC.RegisterStdRobDefServiceTypes(RRN)
    register_service_types_from_resources(RRN, 'pyri.robotics_motion_program',
        ["experimental.robotics.motion_program", "experimental.abb_robot", "experimental.abb_robot.motion_program"])
    

    opt_params_fname = Path(sys.argv[1])/ "motion_opt_input.pickle"

    with open(opt_params_fname,"rb") as f:
        opt_params = pickle.load(f)

    robot = load_rox_robot_obj(opt_params)

    curve_js = opt_params["curve_js"]
    greedy_threshold = opt_params["greedy_threshold"]
    velocity = opt_params["velocity"]*1e3
    blend_radius = opt_params["blend_radius"]*1e3

    breakpoints,primitive_choices,q_bp,p_bp = motion_program_generation_greedy(curve_js, robot, greedy_threshold, True)

    print(f"len(breakpoints): {len(breakpoints)}")
    print(f"len(primitive_choices): {len(primitive_choices)}")
    print(f"len(p_bp): {len(p_bp)}")
    print(f"len(q_bp): {len(q_bp)}")

    mp_exec = MotionSendRobotRaconteurMP(node=RRN)
    rr_mp = mp_exec.convert_motion_program(robot,primitive_choices,breakpoints, p_bp, q_bp,velocity,blend_radius)

    rr_mp_bytes = rr_varvalue_to_bytes(RR.VarValue(rr_mp, "experimental.robotics.motion_program.MotionProgram"),RRN)

    output = {
        "breakpoints": breakpoints,
        "primitive_choices": primitive_choices,
        "p_bp": p_bp,
        "q_bp": q_bp,
        "motion_program": rr_mp_bytes
    }

    output_fname = Path(sys.argv[1])/ "motion_opt_output.pickle"

    with open(output_fname, "wb") as f:
        pickle.dump(output, f)

if __name__ == "__main__":
    main()