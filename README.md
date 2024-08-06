<div align="center">
  
__Add pano photo of drone room here__
# Stretch Embodied VLM
Embodied AI with the Hello Robot Stretch3 with the Learning Systems & Robotics Lab

[![PyTorch](https://img.shields.io/badge/Videos-db6a4b.svg?style=for-the-badge&logo=airplayvideo)](https://shattereddisk.github.io/rickroll/rickroll.mp4)

</div>


## 🗺️ Table of Contents 
- [<code>📦 Setup</code>](#-setup)
- [<code>📗 Hardware and Software Requirements</code>](#-hardware-and-software-requirements)
- [<code>🎧 Playlist management</code>](#-playlist-management)
- [<code>🚦️ Controls</code>](#-controls)
- [<code>⚙️ Configuration</code>](#-configuration)
- [<code>💾 Cache</code>](#-cache)
- [<code>🌐 Update</code>](#-update)
- [<code>📝 License</code>](#-license)
- [<code>📢 Acknowledgments</code>](#-acknowledgments)


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

## 📦 Setup

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
