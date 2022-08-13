from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory, \
    add_blockly_block, add_blockly_category
from . import sandbox_functions
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
                    "type": "robot_mp_set_active_robot",
                    "message0": "set active motion program robot %1",
                    "args0": [
                        {
                        "type": "field_input",
                        "name": "ROBOT_NAME",
                        "text": "robot_mp"
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "Set active motion program robot by device name",
                    "helpUrl": ""
                    },
        sandbox_function=(sandbox_functions.robot_mp_set_active_robot,"ROBOT_NAME")
    )

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
                "type": "robot_mp_robot_pose",
                "message0": "new robot pose %1 robot tcp pose %2 joint position seed %3",
                "args0": [
                    {
                    "type": "input_dummy"
                    },
                    {
                    "type": "input_value",
                    "name": "TCP_POSE",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "JOINT_POSITION_SEED",
                    "align": "RIGHT"
                    }
                ],
                "output": None,
                "colour": 340,
                "tooltip": "Create new robot pose from tcp pose and joint position seed",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.robot_mp_robot_pose, "TCP_POSE", "JOINT_POSITION_SEED")
    )

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
                    "type": "robot_mp_begin",
                    "message0": "begin robot motion program",
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "Begin robot motion program",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_mp_begin,) 
    )

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
                    "type": "robot_mp_execute",
                    "lastDummyAlign0": "RIGHT",
                    "message0": "execute motion program %1 wait %2",
                    "args0": [
                        {
                        "type": "input_dummy"
                        },
                        {
                        "type": "field_checkbox",
                        "name": "WAIT",
                        "checked": True
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "Execute robot motion program",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_mp_execute,"WAIT")
    )

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
                    "type": "robot_mp_move_absj",
                    "message0": "motion program MoveAbsJ with speed %1 (m/s) %2 blend radius %3 (m) %4 fine point %5 %6 to joint position %7",
                    "args0": [
                        {
                        "type": "field_number",
                        "name": "SPEED",
                        "value": 1,
                        "min": 0
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "field_number",
                        "name": "BLEND_RADIUS",
                        "value": 0
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "field_checkbox",
                        "name": "FINE_POINT",
                        "checked": True
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "input_value",
                        "name": "JOINT_POSITION",
                        "align": "RIGHT"
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "motion program MoveAbsJ",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_mp_move_absj, "JOINT_POSITION", "SPEED", "BLEND_RADIUS", "FINE_POINT")
    )

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
                    "type": "robot_mp_movej",
                    "message0": "motion program MoveJ with speed %1 (m/s) %2 blend radius %3 (m) %4 fine point %5 %6 to robot pose %7",
                    "args0": [
                        {
                        "type": "field_number",
                        "name": "SPEED",
                        "value": 1,
                        "min": 0
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "field_number",
                        "name": "BLEND_RADIUS",
                        "value": 0
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "field_checkbox",
                        "name": "FINE_POINT",
                        "checked": True
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "input_value",
                        "name": "ROBOT_POSE",
                        "align": "RIGHT"
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "motion program MoveJ",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_mp_movej, "ROBOT_POSE", "SPEED", "BLEND_RADIUS", "FINE_POINT")
    )

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
                    "type": "robot_mp_movel",
                    "message0": "motion program MoveL with speed %1 (m/s) %2 blend radius %3 (m) %4 fine point %5 %6 to robot pose %7",
                    "args0": [
                        {
                        "type": "field_number",
                        "name": "SPEED",
                        "value": 1,
                        "min": 0
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "field_number",
                        "name": "BLEND_RADIUS",
                        "value": 0
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "field_checkbox",
                        "name": "FINE_POINT",
                        "checked": True
                        },
                        {
                        "type": "input_dummy",
                        "align": "RIGHT"
                        },
                        {
                        "type": "input_value",
                        "name": "ROBOT_POSE",
                        "align": "RIGHT"
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "motion program MoveL",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_mp_movel,"ROBOT_POSE","SPEED","BLEND_RADIUS","FINE_POINT")
    )

    add_blockly_block(blocks,
        category = "Motion Program",
        blockly_json = {
        "type": "robot_mp_movec",
        "message0": "motion program MoveC with speed %1 (m/s) %2 blend radius %3 (m) %4 fine point %5 %6 via robot pose %7 to robot pose %8",
        "args0": [
            {
            "type": "field_number",
            "name": "SPEED",
            "value": 1,
            "min": 0
            },
            {
            "type": "input_dummy",
            "align": "RIGHT"
            },
            {
            "type": "field_number",
            "name": "BLEND_RADIUS",
            "value": 0
            },
            {
            "type": "input_dummy",
            "align": "RIGHT"
            },
            {
            "type": "field_checkbox",
            "name": "FINE_POINT",
            "checked": True
            },
            {
            "type": "input_dummy",
            "align": "RIGHT"
            },
            {
            "type": "input_value",
            "name": "ROBOT_VIA_POSE",
            "align": "RIGHT"
            },
            {
            "type": "input_value",
            "name": "ROBOT_POSE",
            "align": "RIGHT"
            }
        ],
        "previousStatement": None,
        "nextStatement": None,
        "colour": 340,
        "tooltip": "motion program MoveC",
        "helpUrl": ""
        },
        sandbox_function = (sandbox_functions.robot_mp_movec,"ROBOT_VIA_POSE", "ROBOT_POSE", "SPEED", "BLEND_RADIUS", "FINE_POINT")
    )

    

    return blocks

def _get_categories() -> Dict[str,PyriBlocklyCategory]:
    categories = {}
    add_blockly_category(categories, "Motion Program", 0)
    return categories

class PyriRoboticsMPBlocklyPluginFactory(PyriBlocklyPluginFactory):
    def get_plugin_name(self):
        return "pyri-robotics"

    def get_category_names(self) -> List[str]:
        return ["Motion Program"]

    def get_categories(self) -> List[PyriBlocklyCategory]:
        return _get_categories()

    def get_block_names(self) -> List[str]:
        return list(_get_blocks().keys())

    def get_block(self,name) -> PyriBlocklyBlock:
        return _get_blocks()[name]

    def get_all_blocks(self) -> Dict[str,PyriBlocklyBlock]:
        return _get_blocks()

def get_blockly_factory():
    return PyriRoboticsMPBlocklyPluginFactory()

