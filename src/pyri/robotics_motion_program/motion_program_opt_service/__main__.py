from datetime import datetime
import importlib
import io
from pathlib import Path
import pickle
import queue
import subprocess
import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import argparse
import RobotRaconteurCompanion as RRC
from pyri.device_manager_client import DeviceManagerClient
import importlib.resources as resources
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from pyri.util.service_setup import PyriServiceNodeSetup

import general_robotics_toolbox as rox
import copy

import time
import threading
import os
import importlib_resources
import tempfile
from .. import util
import matplotlib.pyplot as plt


class RobotMPOpt_impl(object):
    def __init__(self, device_manager, device_info = None, node: RR.RobotRaconteurNode = None):

        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self._node.RequestTimeout = 1000000
        self.device_info = device_info
        self.device_manager = device_manager
        self.device_manager.connect_device_type("experimental.robotics.motion_program.MotionProgramRobot")
        self.device_manager.connect_device_type("tech.pyri.variable_storage.VariableStorage")

        self.service_path = None
        self.ctx = None

        self._action_consts = self._node.GetConstants("com.robotraconteur.action")
        self._robot_util = RobotUtil(node=self._node)
        self._geom_util = GeometryUtil(node = self._node)

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx

    def motion_program_opt(self, algorithm, input_parameters):

        timestamp = datetime.now().strftime("%Y-%m-%d--%H-%M-%S.%f")

        print(f"timestamp: {timestamp}")


        if algorithm == "redundancy_resolution":
            from .redundancy_resolution_alg import run_redundancy_resolution_algorithm
            return run_redundancy_resolution_algorithm(algorithm, input_parameters, self.device_manager, self._node)
        elif algorithm == "motion_program_generation":
            from .motion_program_generation_alg import run_motion_program_generation_algorithm
            return run_motion_program_generation_algorithm(algorithm, input_parameters, self.device_manager, self._node)
        else:
            assert False, f"Invalid motion program optimization algorithm: {algorithm}"

        #raise RR.NotImplementedException("Not implemented")

        # robot = self.device_manager.get_device_client(robot_local_device_name)
        # robot_info = robot.motion_program_robot_info.robot_info
        # rox_robot = self._robot_util.robot_info_to_rox_robot(robot_info,0)

        # assert trajectory_format == "trajectory-data-format-joints6", "Trajectory data must be in Joints 6 axis (q1,q2,q3,q4,q5,q6) format"
        # assert frame == "robot" or frame=="world", "Frame must be \"robot\" or \"world\""

        # if frame == "world":
        #     robot_origin_pose = var_storage.getf_variable_value("globals",robot_origin_calib_global_name)
        #     T_rob = self._geom_util.named_pose_to_rox_transform(robot_origin_pose.data.pose)

        #     rox_robot2 = copy.deepcopy(rox_robot)
        #     rox_robot2.P = np.array([(T_rob.R @ p1 + T_rob.p).flatten() for p1 in rox_robot.P.T]).T
        #     rox_robot2.H = np.array([ (T_rob.R @ h1[0:3]) for h1 in rox_robot.H.T]).T

        #     print(f"rox_robot.P: {rox_robot.P}")
        #     print(f"rox_robot2.P: {rox_robot2.P}")
        #     print(f"rox_robot.H: {rox_robot2.H}")
        #     print(f"rox_robot2.H: {rox_robot2.H}")

        #     rox_robot = rox_robot2

        # rox_tool_pose = self._geom_util.pose_to_rox_transform(tool_pose)

        # tolerance_var = opt_params["max_error_threshold"]
        # assert tolerance_var.datatype == "double[]"
        # max_error_threshold = tolerance_var.data[0]

        # blend_radius_var = opt_params["blend_radius"]
        # assert blend_radius_var.datatype == "double[]"
        # blend_radius = blend_radius_var.data[0]

        # velocity_var = opt_params["velocity"]
        # assert velocity_var.datatype == "double[]"
        # velocity = velocity_var.data[0]

        # max_error_threshold = 0.02
        # blend_radius = 0.2
        # velocity = 0.2

        # opt_params = {
        #     "input_trajectory": input_trajectory,
        #     "trajectory_format": trajectory_format,
        #     "rox_robot": rox_robot,
        #     "rox_tool_pose": rox_tool_pose,
        #     "max_error_threshold": max_error_threshold,
        #     "blend_radius": blend_radius,
        #     "velocity": velocity
        # }

    def motion_program_exec(self, robot_local_device_name, motion_program_global_name, exec_parameters):

        timestamp = datetime.now().strftime("%Y-%m-%d--%H-%M-%S.%f")

        print(f"timestamp: {timestamp}")

        robot_mp = self.device_manager.get_device_client(robot_local_device_name,0.1)
        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        motion_program = var_storage.getf_variable_value("globals",motion_program_global_name)
        exec_mp_gen = robot_mp.execute_motion_program_log(motion_program.data)

        gen = ExecuteMotionProgramGen(robot_mp, exec_mp_gen, exec_parameters, self.device_manager)

        return gen


class MotionOptExec:
    def __init__(self, device_manager, save_input_parameters_fn, load_and_save_result_fn, load_progress_fn, node):
        self.node = node
        self.device_manager = device_manager
        self.save_input_parameters_fn = save_input_parameters_fn
        self.load_and_save_result_fn = load_and_save_result_fn
        self.load_progress_fn = load_progress_fn

        opt_python_exe = os.environ.get("PYRI_MOTION_PROGRAM_OPT_PYTHON_EXE",None)
        opt_motion_dir = os.environ.get("PYRI_MOTION_PROGRAM_OPT_DIR",None)

        assert opt_python_exe, "PYRI_MOTION_PROGRAM_OPT_PYTHON_EXE must point to Python executable for greedy algorithm"
        assert opt_motion_dir, "PYRI_MOTION_PROGRAM_OPT_DIR must point to root of the motion optimization repository"

        self.opt_python_exe = opt_python_exe
        self.opt_motion_dir = opt_motion_dir


    def run_alg(self, alg_name, alg_script_fname, input_parameters, py_subdirs = [], cwd = None):

        timestamp = datetime.now().strftime("%Y-%m-%d--%H-%M-%S.%f")

        print(f"Running motion program algorithm {alg_name} at {timestamp}")
        
        var_storage = self.device_manager.get_device_client("variable_storage",0.1)
        
        try:
            temp_data_dir = None
            temp_data_dir_env = os.environ.get("PYRI_MOTION_PROGRAM_OPT_TEMP_DIR",None)
            if temp_data_dir_env is None:
                temp_data_dir = tempfile.TemporaryDirectory()
                data_dir = Path(temp_data_dir)
            else:
                
                data_dir = Path(temp_data_dir_env) / timestamp
                data_dir.mkdir(exist_ok=True)

            with open(data_dir / "motion_opt_input.pickle", "wb") as f:
                self.save_input_parameters_fn(input_parameters, var_storage, f, self.node)

            opt_python_exe = Path(self.opt_python_exe)
            opt_motion_dir = Path(self.opt_motion_dir)

            if cwd is not None:
                cwd = Path(cwd)
                if not cwd.is_absolute():
                    cwd = opt_motion_dir / cwd
            else:
                cwd = opt_motion_dir

            sub_env = copy.copy(os.environ)
            # if "PYTHONPATH" in sub_env:
            #     del sub_env["PYTHONPATH"]

            
            py_path = [str(opt_motion_dir)]
            for py_subdir1 in py_subdirs:
                py_path.append(str(opt_motion_dir / py_subdir1))
            sub_env["PYTHONPATH"] = os.pathsep.join(py_path)

            opt_script = importlib_resources.files(__package__) / "opt_scripts" / alg_script_fname

            #subprocess.check_call([opt_python_exe, opt_script, data_dir],cwd=cwd, env=sub_env)

            opt_process = subprocess.Popen([opt_python_exe, opt_script, data_dir], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, cwd=cwd, env=sub_env)

            opt_output_fname = data_dir / "motion_opt_output.pickle"

            opt_gen = MotionAlgGen(self, opt_process, input_parameters, opt_output_fname, self.device_manager, self.node)
            opt_gen.log_queue.put(f"Running motion program algorithm {alg_name} at {timestamp}")
            opt_gen.log_queue.put("")
            opt_gen.start()
            return opt_gen
        finally:
            if temp_data_dir is not None:
                temp_data_dir.cleanup()

    

class MotionAlgGen:
    def __init__(self,opt_exec,opt_process,input_parameters,opt_output_fname,device_manager,node):
        self.opt_exec = opt_exec
        self.opt_process = opt_process
        self.node = node
        self.device_manager = device_manager
        self.input_parameters = input_parameters
        self.opt_output_fname = opt_output_fname
        self.closed=False
        self.aborted=False
        self.thread = None
        self.action_codes = self.node.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self.completion_status=self.action_codes["error"]
        self.log_queue = queue.Queue()
        self.motion_opt_result_type = self.node.GetStructureType("tech.pyri.robotics.motion_program_opt.MotionOptResult")
        self.motion_opt_status_type = self.node.GetStructureType("tech.pyri.robotics.motion_program_opt.MotionOptStatus")
        self.run_exp = None
        self.result = None
        self.result_plots = None

    def start(self):
        self.thread = threading.Thread(target=self._run)
        self.thread.start()

    def _run(self):
        try:
            while True:
                if self.closed:
                    try:
                        self.opt_process.terminate()
                    finally:
                        pass
                    return
                opt_process_retcode = self.opt_process.poll()
                if opt_process_retcode is not None:
                    if opt_process_retcode == 0:
                        self._save_results()                    
                        self.completion_status = self.action_codes["complete"]
                    else:
                        self.completion_status = self.action_codes["error"]
                    self.log_queue.put(None)
                    return
                
                line = self.opt_process.stdout.readline()
                if len(line) > 0:
                    self.log_queue.put(line.decode('utf-8'))
                else:
                    time.sleep(0.01)
        except Exception as exp:
            self.run_exp = exp
            self.log_queue.put(None)

    def _save_results(self):
        var_storage = self.device_manager.get_device_client("variable_storage",0.1)
        save_result_var = SaveResultVar(var_storage)
        
        with open(self.opt_output_fname, "rb") as f:
            self.result, self.result_plots = self.opt_exec.load_and_save_result_fn(f, self.input_parameters, save_result_var, self.node)

       
        # # TODO: Use non-hardcoded robot and tool
        # opt_robot = util.abb6640(d=50)
        # convert_motion_plan = util.ConvertMotionProgram(self.device_manager, opt_robot, self.node)
        # motion_program = convert_motion_plan.convert_motion_program(opt_output["primitive_choices"], opt_output["breakpoints"], 
        #     opt_output["points"], opt_output["q_bp"], self.opt_input["velocity"], self.opt_input["blend_radius"])

        # save_result_var("motion_program", motion_program, title="Motion Program", suffix=False, 
        #     rr_type = "experimental.robotics.motion_program.MotionProgram")

        # def fix_nested_list(a):
        #     return [np.array(a1,dtype=np.float64) for a1 in a]

        
        # save_result_var("curve_js", self.opt_input["input_trajectory"], title= "Input Trajectory (joint space)")
        # save_result_var("breakpoints", title = "Breakpoints", convert_fn = lambda a: np.array(a, dtype=np.float64))
        # save_result_var("primitive_choices", title = "Primitive Choices", rr_type="string{list}")
        # save_result_var("points", title = "Points", rr_type="double[*]{list}", convert_fn = fix_nested_list)
        # save_result_var("q_bp", title = "Joint Breakpoints", rr_type="double[*]{list}", convert_fn = fix_nested_list)
        # save_result_var("curve_fit", title = "Curve Fit (cartesion)")
        # save_result_var("curve", title = "Curve (cartesian)")
        # save_result_var("curve_fit_js", title = "Curve Fit (joint space)")

        # result = self.greedy_fitting_result_type()
        # result.result_global_variables = save_result_var.result_vars
        # self.result= result



    def Next(self):
        if self.aborted:
            raise RR.OperationAbortedException("")
        if self.closed:
            raise RR.StopIterationException("")

        if self.run_exp:
            raise self.run_exp
        
        lines = []

        queue_get_timeout = 2.5
        log_done = False

        try:
            while True:
                line = self.log_queue.get(block=True, timeout=queue_get_timeout)
                queue_get_timeout = 0.0
                if line is not None:
                    lines.append(line.rstrip())
                else:
                    log_done = True
                    break
        except queue.Empty:
            pass

        if self.aborted:
            raise RR.OperationAbortedException("")

        if self.run_exp:
            raise self.run_exp

        ret = self.motion_opt_status_type()
        ret.log_output = lines

        if log_done:
            ret.action_status = self.completion_status
            ret.result = self.result
            ret.plots = self.result_plots
            self.closed = True

            if ret.result:
                if ret.result.result_global_variables:
                    ret.log_output.append("")
                    for r in ret.result.result_global_variables:
                        ret.log_output.append(f"Created global variable: {r.global_name}")
        else:
            ret.action_status = self.action_codes["running"]

 
        return ret

    def Abort(self):
        self.aborted = True
        self.closed = True
        self.log_queue.put(None)        
        try:
            self.opt_process.terminate()
        finally:
            pass
        return

    def Close(self):
        self.closed = True
        try:
            self.opt_process.terminate()
        finally:
            pass
        return


class SaveResultVar:

    def __init__(self, variable_storage):
        self.node = variable_storage.RRGetNode()
        self.var_storage = variable_storage
        self.var_consts = self.node.GetConstants('tech.pyri.variable_storage', variable_storage)
        self.variable_persistence = self.var_consts["VariablePersistence"]
        self.variable_protection_level = self.var_consts["VariableProtectionLevel"]
        self.result_vars = []
        self.opt_var_type = self.node.GetStructureType("tech.pyri.robotics.motion_program_opt.OptResultGlobalVariable")

    def save_result_var(self, opt_output, output_global_name, key, value = None, contents=None,title="",desc="", rr_type= "double[*]", convert_fn = None, suffix = False):
        if value is None:
            value = opt_output[key]
        if convert_fn is not None:
            value = convert_fn(value)
        if contents is None:
            contents = key
        save_name = output_global_name
        if suffix:
            save_name = f"{output_global_name}_{key}"
        self.var_storage.add_variable2("globals", save_name, rr_type, \
            RR.VarValue(value,rr_type), ["motion_program_opt"], {}, self.variable_persistence["const"], 
            None, self.variable_protection_level["read_write"], \
            [], "Motion program optimization output", False)

        var_info = self.opt_var_type()
        var_info.var_contents = contents
        var_info.title = title
        var_info.global_name = save_name
        var_info.short_description = desc
        self.result_vars.append(var_info)

    def save_result_var2(self, opt_output, input_parameters, key, value = None, contents=None,title="",desc="", rr_type= "double[*]", convert_fn = None, suffix = False):

        output_global_name1 = input_parameters.get(f"{key}_global_name", None)
        if output_global_name1 is None:
            return
        
        assert output_global_name1.datatype == "string", "Output global name must be a string"

        if output_global_name1.data == "":
            return

        output_global_name = output_global_name1.data
        

        self.save_result_var(opt_output, output_global_name, key, value, contents, title, desc, rr_type, convert_fn, suffix)

    def __call__(self, *args, **kwargs):
        self.save_result_var(*args, **kwargs)

class ExecuteMotionProgramGen:
    def __init__(self, robot_mp, exec_mp_gen, exec_parameters, device_manager):
        self.robot_mp = robot_mp
        self.exec_mp_gen = exec_mp_gen
        self.exec_parameters = exec_parameters
        self.exec_mp_gen_closed = False
        self.closed = False
        self.action_status_code = RRN.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self.exec_status_type = RRN.GetStructureType("tech.pyri.robotics.motion_program_opt.MotionProgramExecStatus")
        self.device_manager = device_manager
        self.started = False

    def Next(self):
        if self.closed:
            raise RR.StopIterationException("")
        mp_ret = self.exec_mp_gen.Next()
        ret = self.exec_status_type()
        ret.action_status = mp_ret.action_status
        ret.current_command = mp_ret.current_command
        ret.log_output = []
        ret.plots = {}
        if not self.started:
            ret.log_output.append("Starting motion program execution")
            self.started = True
        if mp_ret.action_status == self.action_status_code["complete"]:
            try:
                save_mp_exec_results(self.robot_mp, self.device_manager, ret, mp_ret, self.exec_parameters)
                ret.log_output.append("Done!")
            finally:
                try:
                    self.exec_mp_gen_closed = True
                    self.exec_mp_gen.Close()
                except:
                    pass
            self.closed = True
        return ret

    def Close(self):
        if not self.exec_mp_gen_closed:
            self.exec_mp_gen.Close()

    def Abort(self):
        self.exec_mp_gen.Abort()

def save_mp_exec_results(robot_mp, device_manager, ret, mp_ret, exec_parameters):

    #TODO: handle case where log is more than one part
    robot_log1 = robot_mp.read_log(mp_ret.log_handle).NextAll()[0]
    robot_log = np.hstack((np.expand_dims(robot_log1.time,1), np.expand_dims(robot_log1.command_number.astype(np.float64),1), robot_log1.joints))

    var_storage = device_manager.get_device_client("variable_storage",1)
    save_result_var = SaveResultVar(var_storage)

    save_result_var.save_result_var2({}, exec_parameters, "joint_log", value = robot_log, title="Robot motion execution joint position log")

    for r in save_result_var.result_vars:
        ret.log_output.append(f"Created global variable: {r.global_name}")

    robot = util.abb6640(d=50)

    fig = plt.figure()
    plt.plot(robot_log1.time, robot_log1.joints)
    plt.title("Joint Position Log")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (deg)")
    plt.legend([f"Joint {i+1}" for i in range(robot_log1.joints.shape[1])])

    joint_plot_io = io.BytesIO()
    plt.savefig(joint_plot_io,format="svg")

    pos = np.array([robot.fwd(np.deg2rad(q)).p for q in robot_log1.joints])
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    nominal_curve_global_name = exec_parameters.get("nominal_curve_global_name", None)
    if nominal_curve_global_name:
        nominal_curve = var_storage.getf_variable_value("globals", nominal_curve_global_name.data).data

    util.fill_curve_plot(ax, pos, nominal_curve[:,0:3])

    #ax.set_box_aspect((world_limits[1]-world_limits[0],world_limits[3]-world_limits[2],world_limits[5]-world_limits[4]))
    plt.title("Output Position Curve")

    curve_plot_io = io.BytesIO()
    plt.savefig(curve_plot_io,format="svg")

    # retf_out.write(f_in.readline())

    ret.plots = {
        "joints": np.frombuffer(joint_plot_io.getvalue(),dtype=np.uint8),
        "curve": np.frombuffer(curve_plot_io.getvalue(),dtype=np.uint8)
    }
    



def main():

    parser = argparse.ArgumentParser(description="PyRI Robotics Motion Program Optimization Service")

    with PyriServiceNodeSetup("tech.pyri.robotics.motion_program_opt", 55922, \
        extra_service_defs=[(__package__,'tech.pyri.robotics.motion_program_opt.robdef'), \
        ('pyri.robotics_motion_program','experimental.robotics.motion_program.robdef')], \
        default_info = (__package__,"pyri_robotics_motion_program_opt_service_default_info.yml"), \
        display_description="PyRI Robotics Motion Program Optimization Service", device_manager_autoconnect=False, \
        distribution_name="pyri-robotics-motion-program",
        arg_parser=parser) as service_node_setup:
        
        # create object
        c = RobotMPOpt_impl(service_node_setup.device_manager, device_info=service_node_setup.device_info_struct, node = RRN)
        # register service with service name "robotics_motion", type "tech.pyri.robotics.motion.RoboticsMotionService",
        # actual object: VisionArucoDetection_inst
        service_node_setup.register_service("robotics_motion_program_opt","tech.pyri.robotics.motion_program_opt.RoboticsMotionProgramOptimizationService",c)

        #Wait for the user to shutdown the service
        service_node_setup.wait_exit()

if __name__ == '__main__':
    main()

