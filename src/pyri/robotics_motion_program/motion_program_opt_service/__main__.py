from datetime import datetime
import importlib
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


class RobotMPOpt_impl(object):
    def __init__(self, device_manager, temp_data_dir = None, device_info = None, node: RR.RobotRaconteurNode = None):

        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self._node.RequestTimeout = 1000000
        self.device_info = device_info
        self.device_manager = device_manager
        self.temp_data_dir = temp_data_dir
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

    def greedy_fitting_motion_program_opt(self, input_trajectory, trajectory_format, frame, robot_local_device_name, 
        robot_origin_calib_global_name, tool_pose, opt_params, output_global_name):

        timestamp = datetime.now().strftime("%Y-%m-%d--%H-%M-%S.%f")

        print(f"timestamp: {timestamp}")
        print(f"input_trajectory.shape: {input_trajectory.shape}")
        print(f"trajectory_format: {trajectory_format}")
        print(f"frame: {frame}")
        print(f"robot_local_device_name: {robot_local_device_name}")
        print(f"robot_origin_calib_global_name: {robot_origin_calib_global_name}")
        print(f"output_global_name: {output_global_name}")
        #raise RR.NotImplementedException("Not implemented")

        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        robot = self.device_manager.get_device_client(robot_local_device_name)
        robot_info = robot.motion_program_robot_info.robot_info
        rox_robot = self._robot_util.robot_info_to_rox_robot(robot_info,0)

        assert trajectory_format == "trajectory-data-format-joints6", "Trajectory data must be in Joints 6 axis (q1,q2,q3,q4,q5,q6) format"
        assert frame == "robot" or frame=="world", "Frame must be \"robot\" or \"world\""

        if frame == "world":
            robot_origin_pose = var_storage.getf_variable_value("globals",robot_origin_calib_global_name)
            T_rob = self._geom_util.named_pose_to_rox_transform(robot_origin_pose.data.pose)

            rox_robot2 = copy.deepcopy(rox_robot)
            rox_robot2.P = np.array([(T_rob.R @ p1 + T_rob.p).flatten() for p1 in rox_robot.P.T]).T
            rox_robot2.H = np.array([ (T_rob.R @ h1[0:3]) for h1 in rox_robot.H.T]).T

            print(f"rox_robot.P: {rox_robot.P}")
            print(f"rox_robot2.P: {rox_robot2.P}")
            print(f"rox_robot.H: {rox_robot2.H}")
            print(f"rox_robot2.H: {rox_robot2.H}")

            rox_robot = rox_robot2

        rox_tool_pose = self._geom_util.pose_to_rox_transform(tool_pose)

        # tolerance_var = opt_params["max_error_threshold"]
        # assert tolerance_var.datatype == "double[]"
        # max_error_threshold = tolerance_var.data[0]

        # blend_radius_var = opt_params["blend_radius"]
        # assert blend_radius_var.datatype == "double[]"
        # blend_radius = blend_radius_var.data[0]

        # velocity_var = opt_params["velocity"]
        # assert velocity_var.datatype == "double[]"
        # velocity = velocity_var.data[0]

        max_error_threshold = 0.02
        blend_radius = 0.2
        velocity = 0.2

        opt_params = {
            "input_trajectory": input_trajectory,
            "trajectory_format": trajectory_format,
            "rox_robot": rox_robot,
            "rox_tool_pose": rox_tool_pose,
            "max_error_threshold": max_error_threshold,
            "blend_radius": blend_radius,
            "velocity": velocity
        }

        try:
            temp_data_dir = None
            if self.temp_data_dir is None:
                temp_data_dir = tempfile.TemporaryDirectory()
                data_dir = Path(temp_data_dir)
            else:
                
                data_dir = Path(self.temp_data_dir) / timestamp
                data_dir.mkdir(exist_ok=True)

            with open(data_dir / "greedy_fitting_opt_input.pickle", "wb") as f:
                pickle.dump(opt_params, f)

            opt_python_exe = os.environ.get("PYRI_MOTION_PROGRAM_OPT_PYTHON_EXE",None)
            opt_greedy_dir = os.environ.get("PYRI_MOTION_PROGRAM_OPT_GREEDY_FITTING_DIR",None)

            assert opt_python_exe, "PYRI_MOTION_PROGRAM_OPT_PYTHON_EXE must point to Python executable for greedy algorithm"
            assert opt_greedy_dir, "PYRI_MOTION_PROGRAM_OPT_GREEDY_FITTING_DIR must point to greedy algorithm directory"

            opt_python_exe = Path(opt_python_exe)
            opt_greedy_dir = Path(opt_greedy_dir)

            sub_env = copy.copy(os.environ)
            # if "PYTHONPATH" in sub_env:
            #     del sub_env["PYTHONPATH"]
            sub_env["PYTHONPATH"] = os.pathsep.join([
                str(opt_greedy_dir),
                str((opt_greedy_dir.parent / "toolbox").resolve())
            ])

            opt_script = importlib_resources.files(__package__) / "opt_scripts" / "greedy_fitting_opt.py"

            opt_process = subprocess.Popen([opt_python_exe, opt_script, data_dir], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, cwd=opt_greedy_dir, env=sub_env)

            opt_output_fname = data_dir / "greedy_fitting_opt_output.pickle"

            opt_gen = GreedyAlgGen(opt_process, opt_params, opt_output_fname, output_global_name, self.device_manager, self._node)
            opt_gen.start()
            return opt_gen
        finally:
            if temp_data_dir is not None:
                temp_data_dir.cleanup()

class GreedyAlgGen:
    def __init__(self,opt_process,opt_input,opt_output_fname,output_global_name,device_manager,node):
        self.opt_process = opt_process
        self.node = node
        self.output_global_name = output_global_name
        self.device_manager = device_manager
        self.opt_input = opt_input
        self.opt_output_fname = opt_output_fname
        self.closed=False
        self.aborted=False
        self.thread = None
        self.action_codes = self.node.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self.completion_status=self.action_codes["error"]
        self.log_queue = queue.Queue()
        self.greedy_fitting_result_type = self.node.GetStructureType("tech.pyri.robotics.motion_program_opt.GreedyFittingResult")
        self.greedy_fitting_status_type = self.node.GetStructureType("tech.pyri.robotics.motion_program_opt.GreedyFittingStatus")
        self.opt_var_type = self.node.GetStructureType("tech.pyri.robotics.motion_program_opt.OptResultGlobalVariable")
        self.run_exp = None
        self.result = None

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
                    self._save_results()                    
                    if opt_process_retcode == 0:
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
        with open(self.opt_output_fname, "rb") as f:
            opt_output = pickle.load(f)

        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        var_consts = var_storage.RRGetNode().GetConstants('tech.pyri.variable_storage', var_storage)
        variable_persistence = var_consts["VariablePersistence"]
        variable_protection_level = var_consts["VariableProtectionLevel"]

        result_vars = []
        
        def save_result_var(key, value = None, contents=None,title="",desc="", rr_type= "double[*]", convert_fn = None, suffix = True):
            if value is None:
                value = opt_output[key]
            if convert_fn is not None:
                value = convert_fn(value)
            if contents is None:
                contents = key
            save_name = self.output_global_name
            if suffix:
                save_name = f"{self.output_global_name}_{key}"
            var_storage.add_variable2("globals", save_name, rr_type, \
                RR.VarValue(value,rr_type), ["motion_program_opt"], {}, variable_persistence["const"], 
                None, variable_protection_level["read_write"], \
                [], "Motion program optimization output", False)

            var_info = self.opt_var_type()
            var_info.var_contents = contents
            var_info.title = title
            var_info.global_name = f"{self.output_global_name}_{key}"
            var_info.short_description = desc
            result_vars.append(var_info)
       
        # TODO: Use non-hardcoded robot and tool
        opt_robot = util.abb6640(d=50)
        convert_motion_plan = util.ConvertMotionProgram(self.device_manager, opt_robot, self.node)
        motion_program = convert_motion_plan.convert_motion_program(opt_output["primitive_choices"], opt_output["breakpoints"], 
            opt_output["points"], opt_output["q_bp"], self.opt_input["velocity"], self.opt_input["blend_radius"])

        save_result_var("motion_program", motion_program, title="Motion Program", suffix=False, 
            rr_type = "experimental.robotics.motion_program.MotionProgram")

        def fix_nested_list(a):
            return [np.array(a1,dtype=np.float64) for a1 in a]

        
        save_result_var("curve_js", self.opt_input["input_trajectory"], title= "Input Trajectory (joint space)")
        save_result_var("breakpoints", title = "Breakpoints", convert_fn = lambda a: np.array(a, dtype=np.float64))
        save_result_var("primitive_choices", title = "Primitive Choices", rr_type="string{list}")
        save_result_var("points", title = "Points", rr_type="double[*]{list}", convert_fn = fix_nested_list)
        save_result_var("q_bp", title = "Joint Breakpoints", rr_type="double[*]{list}", convert_fn = fix_nested_list)
        save_result_var("curve_fit", title = "Curve Fit (cartesion)")
        save_result_var("curve", title = "Curve (cartesian)")
        save_result_var("curve_fit_js", title = "Curve Fit (joint space)")

        result = self.greedy_fitting_result_type()
        result.result_global_variables = result_vars
        self.result= result



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

        ret = self.greedy_fitting_status_type()

        if log_done:
            ret.action_status = self.completion_status
            ret.result = self.result
            self.closed = True
        else:
            ret.action_status = self.completion_status = self.action_codes["running"]

        ret.log_output = lines
 
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



def main():

    parser = argparse.ArgumentParser(description="PyRI Robotics Motion Program Optimization Service")
    parser.add_argument("--temp-data-dir",type=str,required=False,default=None,help="Location for temporary files. Defaults to $TEMP")

    with PyriServiceNodeSetup("tech.pyri.robotics.motion_program_opt", 55922, \
        extra_service_defs=[(__package__,'tech.pyri.robotics.motion_program_opt.robdef'), \
        ('pyri.robotics_motion_program','experimental.robotics.motion_program.robdef')], \
        default_info = (__package__,"pyri_robotics_motion_program_opt_service_default_info.yml"), \
        display_description="PyRI Robotics Motion Program Optimization Service", device_manager_autoconnect=False, \
        distribution_name="pyri-robotics-motion-program",
        arg_parser=parser) as service_node_setup:
        
        # create object
        c = RobotMPOpt_impl(service_node_setup.device_manager, temp_data_dir=service_node_setup.argparse_results.temp_data_dir, device_info=service_node_setup.device_info_struct, node = RRN)
        # register service with service name "robotics_motion", type "tech.pyri.robotics.motion.RoboticsMotionService",
        # actual object: VisionArucoDetection_inst
        service_node_setup.register_service("robotics_motion_program_opt","tech.pyri.robotics.motion_program_opt.RoboticsMotionProgramOptimizationService",c)

        #Wait for the user to shutdown the service
        service_node_setup.wait_exit()

if __name__ == '__main__':
    main()

