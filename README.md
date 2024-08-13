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
pip install -r requirements.txt
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
