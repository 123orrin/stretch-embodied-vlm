<div align="center">
  
__Add pano photo of drone room here__
# Stretch Embodied VLM
Embodied AI with the Hello Robot Stretch3 at the Learning Systems & Robotics Lab

[![PyTorch](https://img.shields.io/badge/Demonstration_Video-db6a4b.svg?style=for-the-badge&logo=airplayvideo)](https://shattereddisk.github.io/rickroll/rickroll.mp4)

</div>


### Running VLM Teleop (TEMPORARY AUG13)
- You will need **3 Terminals open on the workstation** and **1 on Stretch**
- Make sure to **connect your microphone to the workstation**

On **Stretch**, run:
```
stretch_free_robot_process.py
stretch_robot_home.py

cd ~/ament_ws/src/lsy_robot_dev/launch
ros2 launch vlm_teleop_launch
```

On the **Workstation in the first terminal**, run the following commands. This should activate the Speech to Text system make sure to also **plug in a microphone** to the workstation:
```
cd ~ament_ws/src/stretch_embodied_vlm/mic/mic
python3 -m venv mic_env
source mic_env/bin/activate
pip install vosk
pip install pyaudio
python3 local_STT.py
```

On the **second terminal**, run:
```
cd ~/ament_ws
source ./install/setup.bash
cd src/stretch_embodied_vlm/lsy_laptop_dev/lsy_laptop_dev
python3 vision_language_server_copy.py
```

On the **third terminal**, run
```
cd ~/ament_ws
source ./install/setup.bash
cd src/stretch_embodied_vlm/lsy_laptop_dev/lsy_laptop_dev
python3 move_to_object.py
```


<div align="center">
  
## 🗺️ Table of Contents (everything under here is still work in progress)
  
</div>

- [<code>What Embodied VLM Can Do
 </code>](#-what-embodied-vlm-can-do)
- [<code>Hardware and Software Requirements</code>](#-hardware-and-software-requirements)
- [<code>Installation and Setup</code>](#-installation-and-setup)
- [<code>Running Embodied VLM</code>](#-running-embodied-vlm)
- [<code>Basic Troubleshooting </code>](#-basic-troubleshooting)
- [<code>Acknowledgments</code>](#-acknowledgments)

<div align="center">

## <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/Smilies/Robot.png" alt="Robot" width="30" height="30" /> What Embodied VLM Can Do

</div>

__list them here__

*
<div align="center">

## 📗 Hardware and Software Requirements

</div>

Hardware:
* [Hello Robot Stretch3](https://hello-robot.com/) 
* A workstation with GPU 

Software:
* Python 3.10 
* OpenAI Key 
* Ubuntu 22.04
* ROS2 Humble


<div align="center">

## 📦 Installation and Setup

</div>


### _On the Workstation_
First create a ROS2 Humble workspace and name it ament_ws
```
add instructions here
```

Clone this repository in src:
```
cd ~/ament_ws/src
git clone https://github.com/123orrin/stretch_embodied_vlm.git
```

Now we have to set up the venvs for vlm_teleop and the Speech to Text system:
```
cd ~/ament_ws/src/stretch_embodied_vlm/mic/mic
python3 -m venv mic_env
pip install vosk
pip install pyaudio

cd ~/ament_ws/src/stretch_embodied_vlm/lsy_laptop_dev/lsy_laptop_dev
python3 -m venv vlm_env
pip install -r requirements.txt
```


Finally, build the packages to complete the installation on the workstation:
```
cd ~/ament_ws
colcon build --packages-select lsy_laptop_dev mic lsy_interfaces lsy_robot_dev
```

### _On the Robot_
Clone this repository in ament_ws in src:

```
cd ~/ament_ws/src
git clone https://github.com/123orrin/stretch_embodied_vlm.git

colcon build --packages-select lsy_laptop_dev mic lsy_interfaces lsy_robot_dev
```

<div align="center">

## <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/Objects/Desktop%20Computer.png" alt="Desktop Computer" width="30" height="30" /> Running Embodied VLM

</div>

- You will need **3 Terminals open on the workstation** and **1 on Stretch**
- Make sure to **connect your microphone to the workstation**

On **Stretch**, run:
```
stretch_free_robot_process.py
stretch_robot_home.py

cd ~/ament_ws/src/lsy_robot_dev/launch
ros2 launch vlm_teleop_launch
```

On the **Workstation in the first terminal**, run the following commands. This should activate the Speech to Text system make sure to also **plug in a microphone** to the workstation:
```
cd ~ament_ws/src/stretch_embodied_vlm/mic/mic
source mic_env/bin/activate

python3 local_STT.py
```

On the **second terminal**, run:
```
cd ~/ament_ws
source install/setup.bash

cd src/stretch_embodied_vlm/lsy_laptop_dev/lsy_laptop_dev
source vlm_env/bin/activate

python3 vision_language_server_copy.py
```

On the **third terminal**, run
```
cd ~/ament_ws
source install/setup.bash

cd src/stretch_embodied_vlm/lsy_laptop_dev/lsy_laptop_dev
source vlm_env/bin/activate

python3 move_to_object.py
```

<div align="center">

## <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/People%20with%20professions/Man%20Mechanic%20Light%20Skin%20Tone.png" alt="Man Mechanic Light Skin Tone" width="35" height="35" /> Basic Troubleshooting

</div>


If a "process has died" error is encountered, run ```stretch_free_robot_process.py``` and run the command again

<div align="center">

## 📢 Acknowledgments

</div>

#### $${\color{#AC3097}Thanks \space to \space our \space amazing \space Hello \space Robot \space team \space at \space LSY\color{red} ❤️}$$






