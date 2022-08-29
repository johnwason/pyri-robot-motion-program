from .__main__ import *
import pickle

def run_redundancy_resolution_algorithm(algorithm, input_parameters, device_manager, node):
    opt_exec = MotionOptExec(device_manager, _save_inputs, _load_and_save_result, _load_progress, node)

    return opt_exec.run_alg("redundancy_resolution", "redundancy_resolution.py", input_parameters, ["data","toolbox"], "data")

def _save_inputs(input_parameters, var_storage, f, node):
    curve = input_parameters["curve"].data

    opt_params = {
        "curve": curve
    }

    pickle.dump(opt_params, f)

def _load_and_save_result(f, input_parameters, save_result_var, node):
    results = pickle.load(f)

    save_result_var.save_result_var2(results, input_parameters, "curve_js", title = "Generated Trajectory (joint space)")

    plots = {
        "j_minimum": np.frombuffer(results["plots"]["j_minimum"],dtype=np.uint8)
    }

    result = node.GetStructureType("tech.pyri.robotics.motion_program_opt.MotionOptResult")()
    result.result_global_variables = save_result_var.result_vars
    return result, plots

def _load_progress():
    pass