from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}

    blocks["robot_mp_set_active_robot"] = PyriBlocklyBlock(
        name = "robot_mp_set_active_robot",
        category = "Motion Program",
        doc = "Set active motion program robot by device name",
        json = """{
                    "type": "robot_mp_set_active_robot",
                    "message0": "set active motion program robot %1",
                    "args0": [
                        {
                        "type": "field_input",
                        "name": "ROBOT_NAME",
                        "text": "robot_mp"
                        }
                    ],
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "Set active robot",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_mp_set_active_robot'] = function(block) {
                            var text_robot_name = block.getFieldValue('ROBOT_NAME');
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_mp_set_active_robot(\"' + text_robot_name + '\")\\n';
                            return code;
                            };
                            """ 
    )

    blocks["robot_mp_robot_pose"] = PyriBlocklyBlock(
        name = "robot_mp_robot_pose",
        category = "Motion Program",
        doc = "Create new robot pose from tcp pose and joint position seed",
        json = """
                {
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
                "output": null,
                "colour": 340,
                "tooltip": "",
                "helpUrl": ""
                }                
                """,
        python_generator = """
                    Blockly.Python['robot_mp_robot_pose'] = function(block) {
                    var value_tcp_pose = Blockly.Python.valueToCode(block, 'TCP_POSE', Blockly.Python.ORDER_ATOMIC);
                    var value_joint_position_seed = Blockly.Python.valueToCode(block, 'JOINT_POSITION_SEED', Blockly.Python.ORDER_ATOMIC);                           
                    // TODO: Assemble Python into code variable.
                    var code = 'robot_mp_robot_pose(' + value_tcp_pose + ', ' + value_joint_position_seed + ')';
                    return [code, Blockly.Python.ORDER_NONE];
                    };
                    """ 
    )

    blocks["robot_mp_begin"] = PyriBlocklyBlock(
        name = "robot_mp_begin",
        category = "Motion Program",
        doc = "Begin robot motion program",
        json = """{
                    "type": "robot_mp_begin",
                    "message0": "begin robot motion program",
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_mp_begin'] = function(block) {                            
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_mp_begin()\\n';
                            return code;
                            };
                            """ 
    )

    blocks["robot_mp_execute"] = PyriBlocklyBlock(
        name = "robot_mp_execute",
        category = "Motion Program",
        doc = "Execute robot motion program",
        json = """{
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
                        "checked": true
                        }
                    ],
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_mp_execute'] = function(block) {
                            var checkbox_wait = block.getFieldValue('WAIT') == 'TRUE'  ? 'True':'False';
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_mp_execute(' + checkbox_wait + ')\\n';
                            return code;
                            };
                            """ 
    )

    blocks["robot_mp_move_absj"] = PyriBlocklyBlock(
        name = "robot_mp_move_absj",
        category = "Motion Program",
        doc = "Move to absolute joint position",
        json = """{
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
                        "checked": true
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
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "motion program MoveAbsJ",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_mp_move_absj'] = function(block) {
                            var number_speed = block.getFieldValue('SPEED');
                            var number_blend_radius = block.getFieldValue('BLEND_RADIUS');
                            var checkbox_fine_point = block.getFieldValue('FINE_POINT') == 'TRUE'  ? 'True':'False';
                            var value_joint_position = Blockly.Python.valueToCode(block, 'JOINT_POSITION', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_mp_move_absj(' + value_joint_position + ', ' + number_speed + ', ' + number_blend_radius + ', ' + checkbox_fine_point + ')\\n';
                            return code;
                            };
                            """
    )

    blocks["robot_mp_movej"] = PyriBlocklyBlock(
        name = "robot_mp_movej",
        category = "Motion Program",
        doc = "Move to pose using joint interpolation",
        json = """{
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
                        "checked": true
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
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "motion program MoveJ",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_mp_movej'] = function(block) {
                            var number_speed = block.getFieldValue('SPEED');
                            var number_blend_radius = block.getFieldValue('BLEND_RADIUS');
                            var checkbox_fine_point = block.getFieldValue('FINE_POINT') == 'TRUE'  ? 'True':'False';
                            var value_robot_pose = Blockly.Python.valueToCode(block, 'ROBOT_POSE', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_mp_movej(' + value_robot_pose + ', ' + number_speed + ', ' + number_blend_radius + ', ' + checkbox_fine_point + ')\\n';
                            return code;
                            };
                            """
    )

    blocks["robot_mp_movel"] = PyriBlocklyBlock(
        name = "robot_mp_movel",
        category = "Motion Program",
        doc = "Move to pose using linear interpolation",
        json = """{
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
                        "checked": true
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
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "motion program MoveL",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_mp_movel'] = function(block) {
                            var number_speed = block.getFieldValue('SPEED');
                            var number_blend_radius = block.getFieldValue('BLEND_RADIUS');
                            var checkbox_fine_point = block.getFieldValue('FINE_POINT') == 'TRUE'  ? 'True':'False';
                            var value_robot_pose = Blockly.Python.valueToCode(block, 'ROBOT_POSE', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_mp_movel(' + value_robot_pose + ', ' + number_speed + ', ' + number_blend_radius + ', ' + checkbox_fine_point + ')\\n';
                            return code;
                            };
                            """
    )

    blocks["robot_mp_movec"] = PyriBlocklyBlock(
        name = "robot_mp_movec",
        category = "Motion Program",
        doc = "Move to pose using circular interpolation",
        json = """
        {
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
            "checked": true
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
        "previousStatement": null,
        "nextStatement": null,
        "colour": 340,
        "tooltip": "",
        "helpUrl": ""
        }
        
        """,
        python_generator = """
                            Blockly.Python['robot_mp_movec'] = function(block) {
                            var number_speed = block.getFieldValue('SPEED');
                            var number_blend_radius = block.getFieldValue('BLEND_RADIUS');
                            var checkbox_fine_point = block.getFieldValue('FINE_POINT') == 'TRUE'  ? 'True':'False';
                            var value_robot_via_pose = Blockly.Python.valueToCode(block, 'ROBOT_VIA_POSE', Blockly.Python.ORDER_ATOMIC);
                            var value_robot_pose = Blockly.Python.valueToCode(block, 'ROBOT_POSE', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_mp_movec(' + value_robot_via_pose + ', ' + value_robot_pose + ', ' +  number_speed + ', ' + number_blend_radius + ', ' + checkbox_fine_point + ')\\n';
                            return code;
                            };
                            """

    )

    

    return blocks

def _get_categories() -> Dict[str,PyriBlocklyCategory]:
    categories = {}
    categories["Motion Program"] = PyriBlocklyCategory(
        name = "Motion Program",
        json = '{"kind": "category", "name": "Motion Program", "colour": 0 }'
    )

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

