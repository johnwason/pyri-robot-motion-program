from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time
import general_robotics_toolbox as rox
import RobotRaconteur as RR

def robot_mp_set_active_robot(robot_name):
    """
    Set the active motion program robot. All robot functions will act on this robot.
    This has not effect on robots that are executing an asynchronous
    operation. The default robot device name is `robot_mp`.

    Parameters:

    * robot_name (str): The motion program robot name
    """

    PyriSandboxContext.context_vars["active_robot_mp"] = robot_name

def _get_active_robot_name():
    # TODO: verify robot exists
    if "active_robot_mp" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_robot_mp"]
    else:
        return "robot_mp"

def _get_active_robot():
    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    robot = device_manager.get_device_client(robot_name, 1)
    return robot

def robot_mp_robot_pose(tcp_pose, joint_position_seed):
    robot = _get_active_robot()
    node = PyriSandboxContext.node

    robot_pose_type = node.GetStructureType("experimental.robotics.motion_program.RobotPose",robot)

    ret = robot_pose_type()
    ret.tcp_pose = tcp_pose
    ret.joint_position_seed = np.deg2rad(joint_position_seed)
    return ret

def robot_mp_begin():
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    # robot_pose_type = node.GetStructureType("experimental.robotics.motion_program.RobotPose",robot)
    # moveabsj_type = node.GetStructureType("experimental.robotics.motion_program.MoveAbsJ",robot)
    # movej_type = node.GetStructureType("experimental.robotics.motion_program.MoveJ",robot)
    # movel_type = node.GetStructureType("experimental.robotics.motion_program.MoveL",robot)
    # movec_type = node.GetStructureType("experimental.robotics.motion_program.MoveC",robot)
    settool_type = node.GetStructureType("experimental.robotics.motion_program.SetTool",robot)
    motionprogram_type = node.GetStructureType("experimental.robotics.motion_program.MotionProgram",robot)
    toolinfo_type = node.GetStructureType("com.robotraconteur.robotics.tool.ToolInfo",robot)
    transform_dt = node.GetNamedArrayDType("com.robotraconteur.geometry.Transform",robot)
    spatialinertia_dt = node.GetNamedArrayDType("com.robotraconteur.geometry.SpatialInertia",robot)

    toolinfo = toolinfo_type()
    toolinfo.tcp = node.ArrayToNamedArray([1,0,0,0,0,0,0.001],transform_dt)
    toolinfo.inertia = node.ArrayToNamedArray([0.1,0,0,0.01,.001,0,0,.001,0,.001],spatialinertia_dt)

    settool = settool_type()
    settool.tool_info = toolinfo

    mp = motionprogram_type()
    mp.motion_program_commands = [RR.VarValue(settool,"experimental.robotics.motion_program.SetTool")]

    PyriSandboxContext.context_vars["robot_motion_program"] = mp


def _get_robot_motion_program():
    mp = PyriSandboxContext.context_vars.get("robot_motion_program",None)
    assert mp is not None, "Motion program not begun"
    return mp

def robot_mp_move_absj(joint_position, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    moveabsj_type = node.GetStructureType("experimental.robotics.motion_program.MoveAbsJ",robot)
    cmd = moveabsj_type()
    cmd.joint_position = np.deg2rad(joint_position)
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJ"))

def robot_mp_movel(robot_pose, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    movel_type = node.GetStructureType("experimental.robotics.motion_program.MoveL",robot)
    cmd = movel_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveL"))

def robot_mp_movej(robot_pose, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    movej_type = node.GetStructureType("experimental.robotics.motion_program.MoveJ",robot)
    cmd = movej_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJ"))

def robot_mp_movec(robot_via_pose, robot_pose, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    movec_type = node.GetStructureType("experimental.robotics.motion_program.MoveC",robot)
    cmd = movec_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_via_pose = robot_via_pose
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveC"))

def robot_mp_execute(wait):
    mp = _get_robot_motion_program()
    robot_name = _get_active_robot_name()
    robot = _get_active_robot()

    mp_gen = robot.execute_motion_program(mp)
    
    PyriSandboxContext.action_runner.run_action(robot_name,mp_gen,wait)

def robot_mp_load(motion_program):
    PyriSandboxContext.context_vars["robot_motion_program"] = motion_program


def _get_sandbox_functions():
    return {
        "robot_mp_set_active_robot": robot_mp_set_active_robot,
        "robot_mp_robot_pose": robot_mp_robot_pose,
        "robot_mp_begin": robot_mp_begin,
        "robot_mp_move_absj": robot_mp_move_absj,
        "robot_mp_movel": robot_mp_movel,
        "robot_mp_movej": robot_mp_movej,
        "robot_mp_movec": robot_mp_movec,
        "robot_mp_execute": robot_mp_execute,
        "robot_mp_load": robot_mp_load

    }

class RoboticsMPSandboxFunctionsPluginFactory(PyriSandboxFunctionsPluginFactory):
    def get_plugin_name(self):
        return "pyri-robotics-motion-program"

    def get_sandbox_function_names(self):
        return list(_get_sandbox_functions().keys())

    def get_sandbox_functions(self):
        return _get_sandbox_functions()


def get_sandbox_functions_factory():
    return RoboticsMPSandboxFunctionsPluginFactory()