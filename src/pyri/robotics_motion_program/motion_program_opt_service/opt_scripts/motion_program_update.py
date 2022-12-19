import numpy as np
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_motion_program_opt.toolbox.robots_def import *
from robot_motion_program_opt.toolbox.utils import *
from robot_motion_program_opt.toolbox.lambda_calc import *
from robot_motion_program_opt.motion_update.motion_update import motion_program_update
from pathlib import Path
import pickle
#from pandas import *
import io
from pyri.robotics_motion_program.motion_program_opt_service.opt_script_util import load_rox_robot_obj, rr_varvalue_to_bytes
from robot_motion_program_opt.toolbox.robotraconteur_mp_utils import MotionSendRobotRaconteurMP
from RobotRaconteur.Client import *
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources
from pyri.device_manager_client import DeviceManagerClient

def main():

    RRC.RegisterStdRobDefServiceTypes(RRN)
    register_service_types_from_resources(RRN, 'pyri.robotics_motion_program',
        ["experimental.robotics.motion_program", "experimental.abb_robot", "experimental.abb_robot.motion_program"])
    
    device_manager = DeviceManagerClient()
    device_manager.refresh_devices()
    time.sleep(1)

    
    opt_params_fname = Path(sys.argv[1])/ "motion_opt_input.pickle"

    with open(opt_params_fname,"rb") as f:
        opt_params = pickle.load(f)

    robot = load_rox_robot_obj(opt_params)

    robot_local_device_name = opt_params["robot_local_device_name"]
    rr_robot = device_manager.get_device_client(robot_local_device_name, timeout = 5)
    robot_motion_send = MotionSendRobotRaconteurMP(rr_robot)


    breakpoints = opt_params["breakpoints"]
    primitive_choices = [x.data for x in opt_params["primitive_choices"]]
    p_bp = [x.data for x in opt_params["p_bp"]]
    q_bp = [x.data for x in opt_params["q_bp"]]
    curve = opt_params["curve_base"]
    error_tol = opt_params["error_tol"]
    angerr_tol = opt_params["angerr_tol"]
    velstd_tol = opt_params["velstd_tol"]
    iter_max = opt_params["iter_max"]
    ext_start = opt_params["ext_start"]
    ext_end = opt_params["ext_end"]
    real_robot = opt_params["real_robot"]        
    velocity = opt_params["velocity"]*1e3
    blend_radius = opt_params["blend_radius"]*1e3
    safe_q = None    

    iter_count = [0]
    def iter_cb(error, angle_error, ave_speed, figs):
        iter_count[0] += 1
        fig_bytes = []
        for fig in figs:
            fig_io = io.BytesIO()
            fig.savefig(fig_io,format="svg")
            fig_bytes.append(fig_io.getvalue())
        fig_output = {
            "figures": {
                f"Iteration {iter_count[0]}": fig_bytes
            }
        }
        
        fig_fname = Path(sys.argv[1])/ f"motion_opt_figure{iter_count[0]}.pickle"
        with open(fig_fname, "wb") as f:
            pickle.dump(fig_output, f)

        print(f"Output figure saved: motion_opt_figure{iter_count[0]}.pickle")
        

    curve_exe_js,speed,error,angle_error,breakpoints,primitive_choices,q_bp,p_bp = \
    motion_program_update(robot, robot_motion_send, iter_cb, breakpoints, primitive_choices,p_bp,q_bp,velocity,curve,
        error_tol,angerr_tol,velstd_tol,iter_max,ext_start,ext_end,real_robot,False,safe_q)

    print(f"len(breakpoints): {len(breakpoints)}")
    print(f"len(primitive_choices): {len(primitive_choices)}")
    print(f"len(p_bp): {len(p_bp)}")
    print(f"len(q_bp): {len(q_bp)}")
    print(f"speed: {speed}")
    print(f"error: {error}")
    print(f"angle_error: {angle_error}")

    mp_exec = MotionSendRobotRaconteurMP(node=RRN)
    rr_mp = mp_exec.convert_motion_program(robot,primitive_choices,breakpoints, p_bp, q_bp,velocity,blend_radius)

    rr_mp_bytes = rr_varvalue_to_bytes(RR.VarValue(rr_mp, "experimental.robotics.motion_program.MotionProgram"),RRN)

    output = {
        "breakpoints": breakpoints,
        "primitive_choices": primitive_choices,
        "p_bp": p_bp,
        "q_bp": q_bp,
        "motion_program": rr_mp_bytes,
        "speed": speed,
        "error": error,
        "angle_error": angle_error
    }

    output_fname = Path(sys.argv[1])/ "motion_opt_output.pickle"

    with open(output_fname, "wb") as f:
        pickle.dump(output, f)

if __name__ == "__main__":
    main()