<p align="center">
<img src="./docs/figures/pyri_logo_web.svg" height="200"/>
</p>

# PyRI Open Source Teach Pendant Robotics Motion Program Package

This package is part of the PyRI project. See https://github.com/pyri-project/pyri-core#documentation for documentation. 

The `pyri-robotics-motion-program` package contains plugins and services for robotics motion programs. The 
`pyri-robotics` package supports devices that implement the `com.robotraconteur.robotics.robot.Robot` standard type.
This standard type is intended for use with robots that support streaming interfaces. The `pyri-robotics-motion-program`
package is intended to be used with devices that implement the new 
`experimental.robotics.motion_program.MotionProgramRobot` candidate standard robot type. These robots execute a 
"motion program" consisting of a sequence of motion commands, that are typically executed using the vendor controller's
motion control computer. For an ABB robot, these commands include `MoveAbsJ`, `MoveJ`, `MoveL`, and `MoveC`, among
others. The exact implementation varies between robot vendors, but generally the motion program driver will
create a file or memory buffer describing the commands to execute, and a software programming running on the
vendor controller will interpret the file and execute the commands. Some drivers like the 
`abb_robotraconteur_driver_hmp` driver are "hybrid" drivers implementing both `Robot` and `MotionProgramRobot`.

This package provides blockly and sandbox functions to build motion programs and request execution on the robot.

This package also provides a `motion_program_opt_service` that implements motion program optimization algorithms. 
These algorithms are designed to generate a sequence of optimal motion primitives that track an arbitrary dense
trajectory at a specified velocity. This type of operation is generally used for painting and cold-spray 
manufacturing processes. The corresponding `pyri-motion-program-robot-browser` package provides a graphical 
user interface frontend for the motion optimization service. See 
[motion_program_opt_gui.md](docs/motion_program_opt_gui.md) for instructions to use the gui.

## Installation

This package is not installed by default. After installing PyRI as described in the `pyri-core` readme, execute the
following to install:

```
python -m pip install  --extra-index-url=https://pyri-project.github.io/pyri-package-server/  pyri-robotics-motion-program

pyri-cli webui-install --extra-index-url=https://pyri-project.github.io/pyri-package-server/ pyri-robotics-motion-program-browser
```

## Blocks and Sandbox Functions

This package contains Blockly blocks and PyRI sandbox functions to build and execute motion programs. See 
[motion_program_robot_blocks_functions.md](docs/motion_program_robot_blocks_functions.md) for more information.

## Services

`pyri-robotics-motion-program` provides the `motion_program_opt_service`:

### motion_program_opt_service

The `motion_program_opt_service` implements the motion program optimization algorithm.

This service is not started automatically, and must be started from the command line.

Standalone service command line example:

```
python -m pyri.robotics_motion_program.motion_program_opt_service
```

The `pyri-variable-storage` and `pyri-device-manager` services must be running before use.

Command line options:

| Option | Type | Required | Description |
| ---    | ---  | ---      | ---         |
| `--device-manager-url=` | Robot Raconteur URL | No | Robot Raconteur URL of device manager service |
| `--device-manager-identifier=` | Identifier | No | Robot Raconteur device identifier in string format for device manager service |
| `--device-info-file=` | File | No | Robot Raconteur `DeviceInfo` YAML file. Defaults to contents of `pyri_robotics_jog_service_default_info.yml` |

This service may use any standard `--robotraconteur-*` service node options.

This service uses the `DeviceManagerClient`, which needs to connect to the device manager service to find other devices. This can be done using discovery based on a Robot Raconteur device identifier, or using a specified Robot Raconteur URL. If neither is specified, the `DeviceManagerClient` will search for the identifier named `pyri_device_manager` on the local machine.

## Acknowledgment

This work was supported in part by Subaward No. ARM-TEC-19-01-F-24 from the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. ARM Project Management was provided by Christopher Adams. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142. 

![](docs/figures/arm_logo.jpg) ![](docs/figures/nys_logo.jpg)

PyRI is developed by Rensselaer Polytechnic Institute, Wason Technology, LLC, and contributors.