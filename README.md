<div align="center">
  
__Add pano photo of drone room here__
# Stretch Embodied VLM
Embodied AI with the Hello Robot Stretch3 at the Learning Systems & Robotics Lab

[![PyTorch](https://img.shields.io/badge/Demonstration_Video-db6a4b.svg?style=for-the-badge&logo=airplayvideo)](https://shattereddisk.github.io/rickroll/rickroll.mp4)

</div>




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
First create a ROS2 Humble workspace and name it ament_ws
```
add instructions here
```

On the __workstation__ in the, clone this repository in src, and build it:
```
cd ~/ament_ws/src
git clone https://github.com/123orrin/stretch_embodied_vlm.git

cd ..
colcon build --packages-select lsy_laptop_dev mic
```
Now we have to set up the venv for the Speech to Text system:
```
cd ~/ament_ws/src/stretch_embodied_vlm/mic/mic
python3 -m venv mic_env   
pip install -r requirements.txt 
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



















## Temporary Instructions


### Commanding Stretch using VLM Teleop
To begin the VLM Teleop demo, follow the instructions in the [<code>Running VLM Teleop</code>](##-running-vlm-teleop) section below.

For each command/prompt, you must begin with "Amy". For example, "Amy, move forward 1 meter" will work.

VLM Teleop has 4 functions:
- **chat** - a simple conversation with Stretch
- **describe** - describes what Stretch's camera sees
- **move a certain distance and direction** - moves in a direction/turns by a certain distance or angle
- **move towards an object** - moves towards an object in the camera's current field of vision (this is very buggy and bad right now)

The code will automatically determine which function to perform based on the prompt you give. Simply begin to give your commands.



### Running VLM Teleop
- You will need **3 Terminals open on the workstation** and **1 on Stretch**
- Make sure to **connect your microphone to the workstation**

On **Stretch**, run:
```
stretch_free_robot_process.py
stretch_robot_home.py

cd ~/ament_ws/src/lsy_robot_dev/launch
ros2 launch vlm_teleop_launch
```

On the **Workstation in the first terminal**, the following commands. This should activate the Speech to Text system. _**Make sure you see a blue spinner saying recording or transcribing after you run ```python3 new.py```**_ If you don't, close the terminal and run this again:
```
cd mic/mic
python3 -m venv mic_env
source mic_env/bin/activate
pip install -r requirements.txt
python3 local_STT.py
```

On the **second terminal**, run:
```
colcon build --packages-select lsy_laptop_dev
source ./install/setup.bash
cd src/lsy_laptop_dev/lsy_laptop_dev
python3 vision_language_server_copy.py
```

On the **third terminal**, run
```
colcon build --packages-select lsy_laptop_dev
source ./install/setup.bash
cd src/lsy_laptop_dev/lsy_laptop_dev
python3 move_to_object.py
```

Everything should be running then, and begin giving your commands!
