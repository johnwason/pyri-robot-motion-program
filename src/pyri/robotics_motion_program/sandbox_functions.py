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
    operation. The default robot device name is `robot`.

    Parameters:

    * robot_name (str): The motion program robot name
    """

    PyriSandboxContext.context_vars["active_robot_mp"] = robot_name

def robot_mp_set_active_tool(tool_name):
    """
    Set the active motion program tool. All robot functions will use this as the tool.
    This has not effect on robots that are executing an asynchronous
    operation. The default robot device name is `tool`.

    Parameters:

    * tool_name (str): The motion program tool name
    """

    PyriSandboxContext.context_vars["active_tool_mp"] = tool_name

def _get_active_robot_name():
    # TODO: verify robot exists
    if "active_robot_mp" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_robot_mp"]
    else:
        return "robot"

def _get_active_tool_name():
    # TODO: verify robot exists
    if "active_tool_mp" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_tool_mp"]
    else:
        return "tool"

def _get_active_robot():
    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    robot = device_manager.get_device_client(robot_name, 1)
    return robot

def _get_active_tool():
    tool_name = _get_active_tool_name()

    device_manager = PyriSandboxContext.device_manager
    tool = device_manager.get_device_client(tool_name, 1)
    return tool

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
    settool_type = node.GetStructureType("experimental.robotics.motion_program.SetToolCommand",robot)
    motionprogram_type = node.GetStructureType("experimental.robotics.motion_program.MotionProgram",robot)
    # toolinfo_type = node.GetStructureType("com.robotraconteur.robotics.tool.ToolInfo",robot)
    transform_dt = node.GetNamedArrayDType("com.robotraconteur.geometry.Transform",robot)
    spatialinertia_dt = node.GetNamedArrayDType("com.robotraconteur.geometry.SpatialInertia",robot)



    toolinfo = _get_active_tool().tool_info
    
    settool = settool_type()
    settool.tool_info = toolinfo

    mp = motionprogram_type()
    mp.motion_setup_commands = [RR.VarValue(settool,"experimental.robotics.motion_program.SetToolCommand")]
    mp.motion_program_commands = []

    PyriSandboxContext.context_vars["robot_motion_program"] = mp


def _get_robot_motion_program():
    mp = PyriSandboxContext.context_vars.get("robot_motion_program",None)
    assert mp is not None, "Motion program not begun"
    return mp

def robot_mp_move_absj(joint_position, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    moveabsj_type = node.GetStructureType("experimental.robotics.motion_program.MoveAbsJCommand",robot)
    cmd = moveabsj_type()
    cmd.joint_position = np.deg2rad(joint_position)
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJCommand"))

def robot_mp_movel(robot_pose, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    movel_type = node.GetStructureType("experimental.robotics.motion_program.MoveLCommand",robot)
    cmd = movel_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveLCommand"))

def robot_mp_movej(robot_pose, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    movej_type = node.GetStructureType("experimental.robotics.motion_program.MoveJCommand",robot)
    cmd = movej_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJCommand"))

def robot_mp_movec(robot_via_pose, robot_pose, speed, blend_radius, fine_point=True):
    mp = _get_robot_motion_program()
    robot = _get_active_robot()
    node = PyriSandboxContext.node
    movec_type = node.GetStructureType("experimental.robotics.motion_program.MoveCCommand",robot)
    cmd = movec_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_via_pose = robot_via_pose
    cmd.tcp_velocity = speed
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    mp.motion_program_commands.append(RR.VarValue(cmd,"experimental.robotics.motion_program.MoveCCommand"))

def robot_mp_execute(wait):
    mp = _get_robot_motion_program()
    robot_name = _get_active_robot_name()
    robot = _get_active_robot()

    robot_mp_state,_ = robot.motion_program_robot_state.PeekInValue()

    node = PyriSandboxContext.node
    mp_const = node.GetConstants("experimental.robotics.motion_program",robot)
    mp_flags = mp_const["MotionProgramRobotStateFlags"]

    if (robot_mp_state.motion_program_robot_state_flags & mp_flags["motion_program_mode_enabled"]) == 0:
        robot.disable_motion_program_mode()
        time.sleep(0.01)
        robot.enable_motion_program_mode()

    mp_gen = robot.execute_motion_program(mp, False)
    
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