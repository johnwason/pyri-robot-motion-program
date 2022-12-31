import pickle
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
import numpy as np
import io

def load_rox_robot_obj(opt_params):
    from robot_motion_program_opt.toolbox.robots_def import robot_obj
    robot_name = opt_params["robot_name"]
    acc_dict = opt_params["robot_acc_data"]
    rox_robot = opt_params["rox_robot"]
    tool_H = opt_params["tool_H"]
    tool_surface_offset = opt_params["tool_surface_offset"]
    rox_robot.R_tool=tool_H[:3,:3]
    rox_robot.p_tool=tool_H[:3,-1]*1e3
    rox_robot.P = rox_robot.P*1e3
    robot = robot_obj(robot_name, rox_robot, acc_dict=acc_dict, d=tool_surface_offset*1e3)

    return robot

def pack_robot_and_tool_dict(input_parameters, device_manager):
    robot_local_device_name = input_parameters["robot_local_device_name"].data
    tool_local_device_name = input_parameters["tool_local_device_name"].data
    tool_surface_offset = input_parameters["tool_surface_offset"].data

    robot_info = device_manager.get_device_client(robot_local_device_name,timeout=1).robot_info
    tool_info = device_manager.get_device_client(tool_local_device_name,timeout=1).tool_info

    robot_util = RobotUtil()
    rox_robot = robot_util.robot_info_to_rox_robot(robot_info,0)

    geom_util = GeometryUtil()
    

    tool_H = np.eye(4,dtype=np.float64)
    tool_T = geom_util.transform_to_rox_transform(tool_info.tcp)
    tool_H[:3,:3] = tool_T.R
    tool_H[:3,-1] = tool_T.p

    with io.BytesIO(input_parameters["robot_acc_data"].data) as acc_f:
        acc_data = pickle.load(acc_f)

    opt_params = {
        "robot_name": robot_local_device_name,
        "robot_acc_data": acc_data,
        "rox_robot": rox_robot,
        "tool_H": tool_H,
        "tool_surface_offset": tool_surface_offset
    }

    return opt_params

from RobotRaconteur.RobotRaconteurPython import MessageElementToBytes, MessageElementFromBytes
from RobotRaconteur.RobotRaconteurPythonUtil import PackMessageElement, UnpackMessageElement


def rr_varvalue_to_bytes(val, node):
    m = PackMessageElement(val, "varvalue", node=node)
    return MessageElementToBytes(m)

def rr_varvalue_from_bytes(b, node):
    m = MessageElementFromBytes(b)
    return UnpackMessageElement(m, "varvalue", node=node)