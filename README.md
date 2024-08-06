# Stretch Embodied VLM
Embodied AI with the Hello Robot Stretch3 with the Learnign Systems Lab

## Hardware and software requirements
Hardware required:
* An iPhone Pro with Lidar sensors
* [Hello Robot Stretch](https://hello-robot.com/) with Dex Wrist installed
* A workstation with GPU to run pretrained models 

Software required:
* Python 3.9
* Record3D (>1.18.0)
* [CloudCompare](https://www.danielgm.net/cc/release/)

## Installation
* You need to get anygrasp [license and checkpoint](./ok-robot-manipulation/anygrasp_license_registration/README.md).
* [Install](./docs/workspace-installation.md) the necessary environment on workstation to run the navigation and manipulation modules
* [Verify the workspace installation](./docs/installation-verification.md) once the above steps are completed.
* [Install](./docs/robot-installation.md) the necessary packages on robot to be able to properly communicate with backend workstation.
* You might also need to get a [new calibrated URDF](./docs/robot-calibration.md) for accurate robot manipulation.

Once both the robot and workstation are complete. You are good to start the experiments.
