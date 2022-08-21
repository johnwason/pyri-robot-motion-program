from pathlib import Path
import pickle
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

        self._greedy_fitting_result_type = self._node.GetStructureType("tech.pyri.robotics.motion_program_opt.GreedyFittingResult")
        self._greedy_fitting_status_type = self._node.GetStructureType("tech.pyri.robotics.motion_program_opt.GreedyFittingStatus")
        self._action_consts = self._node.GetConstants("com.robotraconteur.action")
        self._robot_util = RobotUtil(node=self._node)
        self._geom_util = GeometryUtil(node = self._node)

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx

    def greedy_fitting_motion_program_opt(self, input_trajectory, trajectory_format, frame, robot_local_device_name, 
        robot_origin_calib_global_name, tool_pose, opt_params, output_global_name):
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

        assert trajectory_format == "trajectory-data-format-p-n", "Trajectory data must be in (xyz),(ijk) format"
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

        opt_params = {
            "input_trajectory": input_trajectory,
            "trajectory_format": trajectory_format,
            "rox_robot": rox_robot,
            "rox_tool_pose": rox_tool_pose
        }

        with tempfile.TemporaryDirectory() as data_dir1:
            data_dir = Path(data_dir1)
            data_dir = Path(r"C:\Users\wasonj\Documents\pyri\experiments\motion_program_opt")

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
            try:
                sub_output = subprocess.check_output([opt_python_exe, opt_script, data_dir], stderr=subprocess.STDOUT, cwd=opt_greedy_dir, env=sub_env)
            except subprocess.CalledProcessError as err:
                status = self._greedy_fitting_status_type()
                status.action_status = self._action_consts["ActionStatusCode"]["error"]
                status.log_output = err.output.decode('utf-8').splitlines()
                return GreedyAlgGen(status)


            status = self._greedy_fitting_status_type()
            # result = self._greedy_fitting_result_type()
            status.action_status = self._action_consts["ActionStatusCode"]["complete"]
            status.log_output = sub_output.decode('utf-8').splitlines()
            # status.result = result

            return GreedyAlgGen(status)

class GreedyAlgGen:
    def __init__(self,ret):
        self.ret = ret
        self.closed=False

    def Next(self):
        if self.closed:
            raise RR.StopIterationException("")
        self.closed=True
        return self.ret

    def Abort(self):
        self.closed = True

    def Close(self):
        self.closed = True



def main():

    with PyriServiceNodeSetup("tech.pyri.robotics.motion_program_opt", 55922, \
        extra_service_defs=[(__package__,'tech.pyri.robotics.motion_program_opt.robdef'), \
        ('pyri.robotics_motion_program','experimental.robotics.motion_program.robdef')], \
        default_info = (__package__,"pyri_robotics_motion_program_opt_service_default_info.yml"), \
        display_description="PyRI Robotics Motion Program Optimization Service", device_manager_autoconnect=False, \
        distribution_name="pyri-robotics-motion-program") as service_node_setup:
        
        # create object
        c = RobotMPOpt_impl(service_node_setup.device_manager, device_info=service_node_setup.device_info_struct, node = RRN)
        # register service with service name "robotics_motion", type "tech.pyri.robotics.motion.RoboticsMotionService",
        # actual object: VisionArucoDetection_inst
        service_node_setup.register_service("robotics_motion_program_opt","tech.pyri.robotics.motion_program_opt.RoboticsMotionProgramOptimizationService",c)

        #Wait for the user to shutdown the service
        service_node_setup.wait_exit()

if __name__ == '__main__':
    main()

