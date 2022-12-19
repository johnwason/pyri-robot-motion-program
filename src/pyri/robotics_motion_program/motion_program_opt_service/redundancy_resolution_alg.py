from .__main__ import *
import pickle
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from .opt_script_util import pack_robot_and_tool_dict

def run_redundancy_resolution_algorithm(algorithm, input_parameters, device_manager, node):
    opt_exec = MotionOptExec(device_manager, _save_inputs, _load_and_save_result, _load_progress, node)

    return opt_exec.run_alg("redundancy_resolution", "redundancy_resolution.py", input_parameters, ["data","toolbox"], "data")

def _save_inputs(device_manager, input_parameters, var_storage, f, node):
    curve = input_parameters["curve"].data
    
    opt_params = {
        "curve": curve,
    }

    opt_params_robot = pack_robot_and_tool_dict(input_parameters, device_manager)
    opt_params.update(opt_params_robot)

    pickle.dump(opt_params, f)

    return opt_params

def H_to_rr_pose(H, node):
    geom_util = GeometryUtil(node=node)
    R = H[0:3,0:3]
    p = H[0:3,3]

    return geom_util.rox_transform_to_pose(rox.Transform(R,p))

def rr_convert_plots(plots):
    ret = {k: np.frombuffer(v,dtype=np.uint8) for k, v in plots.items()}
    return ret

def _load_and_save_result(f, input_parameters, opt_params, save_result_var, node):
    results = pickle.load(f)

    def H_to_rr_pose2(H):
        return H_to_rr_pose(H,node)

    save_result_var.save_result_var2(results, input_parameters, "curve", title = "Input Curve (position, normal)")
    save_result_var.save_result_var2(results, input_parameters, "curve_js", title = "Computed Curve (joint space)")
    save_result_var.save_result_var2(results, input_parameters, "curve_base", title = "Curve in Robot Base Frame (cartesian)")
    save_result_var.save_result_var2(results, input_parameters, "curve_pose", title= "Curve pose in robot frame (cartesian)", convert_fn = H_to_rr_pose2, rr_type="com.robotraconteur.geometry.Pose")
    save_result_var.save_result_var2(results, input_parameters, "plots", title = "Saved plots", convert_fn = rr_convert_plots, rr_type="uint8[]{string}")

    plots = {
        "curve_3d": np.frombuffer(results["plots"]["curve_3d"],dtype=np.uint8)
    }

    result = node.GetStructureType("tech.pyri.robotics.motion_program_opt.MotionOptResult")()
    result.result_global_variables = save_result_var.result_vars
    return result, plots

def _load_progress():
    pass