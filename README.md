<div align="center">
  
__Add pano photo of drone room here__
# Stretch Embodied VLM
Embodied AI with the Hello Robot Stretch3 at the Learning Systems & Robotics Lab

[![PyTorch](https://img.shields.io/badge/Demonstration_Video-db6a4b.svg?style=for-the-badge&logo=airplayvideo)](https://shattereddisk.github.io/rickroll/rickroll.mp4)

</div>

## Temporary Demo Instructions for Aug 9


### Commanding Stretch using VLM Teleop
To begin the VLM Teleop demo, follow the instructions in the **Running VLM Teleop** Section below.

For each command/prompt, you must begin with "Amy". For example, "Amy, move forward 1 meter" will work.

VLM Teleop has 4 functions:
- **chat** - a simple conversation with Stretch
- **describe** - describes what Stretch's camera sees
- **move a certain distance and direction** - moves in a direction/turns by a certain distance or angle
- **move towards an object** - moves towards an object in the camera's current field of vision (this is very buggy and bad right now)

The code will automatically determine which function to perform based on the prompt you give. Simply begin to give your commands.



### Running VLM Teleop
You will need **3 Terminals open on the workstation** and **1 on Stretch**

On **Stretch**, run:
```
stretch_free_robot_process.py
stretch_robot_home.py

cd ~/ament_ws/src/path to launch file
ros2 launch the launch
```

On the **Workstation in the first terminal**, the following commands. This should activate the Speech to Text system. _**Make sure you see a blue spinner saying recording or transcribing after you run ```python3 new.py```**_ If you don't, close the terminal and run this again:
```
cd /repos/mic
source stt2/bin/activate
python3 new.py
```

On the **second terminal**, run:
```
cd ament_ws
source ./install/setup.bash
cd to the files
run the service
```

On the **third terminal**, run
```
cd ament_ws
source ./install/setup.bash
cd to the files
run the vlm code
```

Everything should be running then, and begin giving your commands!


## 🗺️ Table of Contents (everything under here is still work in progress)
- [<code>What Embodied VLM Can Do
 </code>](#-what-embodied-vlm-can-do)
- [<code>Hardware and Software Requirements</code>](#-hardware-and-software-requirements)
- [<code>Installation and Setup</code>](#-installation-and-setup)
- [<code>Running Embodied VLM</code>](#-running-embodied-vlm)
- [<code>Basic Troubleshooting </code>](#-basic-troubleshooting)
- [<code>Acknowledgments</code>](#-acknowledgments)


## <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/Smilies/Robot.png" alt="Robot" width="30" height="30" /> What Embodied VLM Can Do


__list them here__

*

## 📗 Hardware and Software Requirements
Hardware:
* [Hello Robot Stretch3](https://hello-robot.com/) 
* A workstation with GPU 

Software:
* Python 3.X __TO ADD LATER__
* OpenAI Key
* Ubuntu 22.04
* ROS2 Humble

## 📦 Installation and Setup

On the __workstation__, clone this repository:
```
git clone https://github.com/123orrin/stretch_embodied_vlm.git
```

Then cd into the directory and set up the environment with the required libraries using:
```
cd stretch_embodied_vlm
# python -m venv embodied_vlm_env   ADD THE VERSION 
# pip install requirements.txt or however else we do it
```


## <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/Objects/Desktop%20Computer.png" alt="Desktop Computer" width="30" height="30" /> Running Embodied VLM
Run the following commands on Stretch and the workstation:

### On the Robot

Before running anything on Stretch, first home it using
```
stretch_free_robot_process.py
stretch_robot_home.py
```

### On the Workstation

```
# Run services, launch launches, run other scripts
```


## <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/People%20with%20professions/Man%20Mechanic%20Light%20Skin%20Tone.png" alt="Man Mechanic Light Skin Tone" width="35" height="35" /> Basic Troubleshooting

If a "process has died" error is encountered, run ```stretch_free_robot_process.py``` and run the command again


## 📢 Acknowledgments
#### $${\color{#AC3097}Thanks \space to \space our \space amazing \space Hello \space Robot \space team \space at \space LSY\color{red} ❤️}$$
