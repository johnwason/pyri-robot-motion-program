from typing import List
from RobotRaconteurCompanion.Util import RobDef as robdef_util
from pyri.plugins.robdef import PyriRobDefPluginFactory

class RoboticsMPRobDefPluginFactory(PyriRobDefPluginFactory):
    def __init__(self):
        super().__init__()

    def get_plugin_name(self):
        return "pyri-robotics-motion-program"

    def get_robdef_names(self) -> List[str]:
        return ["experimental.robotics.motion_program"]

    def  get_robdefs(self) -> List[str]:
        return get_robotics_mp_robdef()

def get_robdef_factory():
    return RoboticsMPRobDefPluginFactory()

def get_robotics_mp_robdef():
    return robdef_util.get_service_types_from_resources(__package__,["experimental.robotics.motion_program"])