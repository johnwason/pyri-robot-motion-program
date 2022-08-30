from .__main__ import *
import pickle

def run_motion_program_generation_algorithm(algorithm, input_parameters, device_manager, node):
    opt_exec = MotionOptExec(device_manager, _save_inputs, _load_and_save_result, _load_progress, node)

    return opt_exec.run_alg("motion_program_generation", "motion_program_generation.py", input_parameters, ["data","toolbox"], "data")

def _save_inputs(input_parameters, var_storage, f, node):
    curve_js = input_parameters["curve_js"].data
    total_seg = input_parameters["total_seg"].data[0]

    opt_params = {
        "curve_js": curve_js,
        "total_seg": total_seg
    }

    pickle.dump(opt_params, f)

def H_to_rr_pose(H, node):
    geom_util = GeometryUtil(node=node)
    R = H[0:3,0:3]
    p = H[0:3,3]

    return geom_util.rox_transform_to_pose(rox.Transform(R,p))

def rr_convert_plots(plots):
    ret = {k: np.frombuffer(v,dtype=np.uint8) for k, v in plots.items()}
    return ret

def _load_and_save_result(f, input_parameters, save_result_var, node):
    opt_output = pickle.load(f)

    # def H_to_rr_pose2(H):
    #     return H_to_rr_pose(H,node)

    # TODO: Use non-hardcoded robot and tool
    opt_robot = util.abb6640(d=50)
    convert_motion_plan = util.ConvertMotionProgram(None, opt_robot, node)
    motion_program = convert_motion_plan.convert_motion_program(opt_output["primitive_choices"], opt_output["breakpoints"], 
        opt_output["p_bp"], opt_output["q_bp"], input_parameters["velocity"].data[0], input_parameters["blend_radius"].data[0])
         

    def fix_nested_list(a):
        return [np.array(a1,dtype=np.float64) for a1 in a]

    motion_program_parameters = {
        "breakpoints": RR.VarValue(np.array(opt_output["breakpoints"],dtype=np.uint32), "uint32[]"),
        "primitive_choices": RR.VarValue(opt_output["primitive_choices"], "string{list}"),
        "p_bp": RR.VarValue(fix_nested_list(opt_output["p_bp"]), "double[*]{list}"),
        "q_bp": RR.VarValue(fix_nested_list(opt_output["q_bp"]), "double[*]{list}"),
        "velocity": input_parameters["velocity"],
        "blend_radius": input_parameters["blend_radius"]
    }

    save_result_var.save_result_var2(opt_output, input_parameters, "motion_program", value = motion_program, 
        title = "Robot motion program", rr_type = "experimental.robotics.motion_program.MotionProgram")
    save_result_var.save_result_var2(opt_output, input_parameters, "motion_program_parameters", 
        value=motion_program_parameters, title = "Robot motion program parameters", rr_type="varvalue{string}")

    result = node.GetStructureType("tech.pyri.robotics.motion_program_opt.MotionOptResult")()
    result.result_global_variables = save_result_var.result_vars
    return result, {}

def _load_progress():
    pass