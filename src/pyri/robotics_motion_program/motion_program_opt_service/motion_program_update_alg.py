from .__main__ import *
import pickle
from .opt_script_util import pack_robot_and_tool_dict, rr_varvalue_from_bytes

def run_motion_program_update_algorithm(algorithm, input_parameters, device_manager, node):
    opt_exec = MotionOptExec(device_manager, _save_inputs, _load_and_save_result, _load_progress, node)

    return opt_exec.run_alg("motion_program_update", "motion_program_update.py", input_parameters, ["data","toolbox"], "data")

def _save_inputs(device_manager, input_parameters, var_storage, f, node):
    curve_base = input_parameters["curve_base"].data
    mp_gen_params_var_name = input_parameters["mp_gen_params"].data
    mp_gen_params = var_storage.getf_variable_value("globals", mp_gen_params_var_name).data
    velocity = input_parameters["velocity"].data[0]
    blend_radius = input_parameters["blend_radius"].data[0]
    error_tol = input_parameters["error_tol"].data[0]
    angerr_tol = input_parameters["angerr_tol"].data[0]
    velstd_tol = input_parameters["velstd_tol"].data[0]
    iter_max = input_parameters["iter_max"].data[0]
    ext_start = input_parameters["ext_start"].data[0]
    ext_end = input_parameters["ext_end"].data[0]
    real_robot = input_parameters["real_robot"].data[0]
    robot_local_device_name = input_parameters["robot_local_device_name"].data  

    opt_params = {
        "curve_base": curve_base,
        "velocity": velocity,
        "blend_radius": blend_radius,
        "error_tol": error_tol,
        "angerr_tol": angerr_tol,
        "velstd_tol": velstd_tol,
        "iter_max": iter_max,
        "ext_start": ext_start,
        "ext_end": ext_end,
        "real_robot": real_robot,
        "breakpoints": mp_gen_params["breakpoints"].data,
        "primitive_choices": mp_gen_params["primitive_choices"].data,
        "p_bp": mp_gen_params["p_bp"].data,
        "q_bp": mp_gen_params["q_bp"].data,
        "robot_local_device_name": robot_local_device_name
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
    opt_output = pickle.load(f)

    motion_program = rr_varvalue_from_bytes(opt_output["motion_program"],node).data

    def fix_nested_list(a):
        return [np.array(a1,dtype=np.float64) for a1 in a]

    motion_program_parameters = {
        "breakpoints": RR.VarValue(np.array(opt_output["breakpoints"],dtype=np.uint32), "uint32[]"),
        "primitive_choices": RR.VarValue(opt_output["primitive_choices"], "string{list}"),
        "p_bp": RR.VarValue(fix_nested_list(opt_output["p_bp"]), "double[*]{list}"),
        "q_bp": RR.VarValue(fix_nested_list(opt_output["q_bp"]), "double[*]{list}"),
        "speed": RR.VarValue(opt_output["speed"], "double"),
        "error": RR.VarValue(opt_output["error"], "double"),
        "angle_error": RR.VarValue(opt_output["angle_error"], "double")
    }

    save_result_var.save_result_var2(opt_output, input_parameters, "motion_program", value = motion_program, 
        title = "Robot motion program", rr_type = "experimental.robotics.motion_program.MotionProgram")
    save_result_var.save_result_var2(opt_output, input_parameters, "motion_program_parameters", 
        value=motion_program_parameters, title = "Robot motion update parameters", rr_type="varvalue{string}")

    result = node.GetStructureType("tech.pyri.robotics.motion_program_opt.MotionOptResult")()
    result.result_global_variables = save_result_var.result_vars
    return result, {}

def _load_progress():
    pass