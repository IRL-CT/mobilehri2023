# Dancebot 2025
Code base for robot chair dance 2025

## StartUp
1. Connect the Raspberry Pi to the display or SSH into it
```
username: dancerobot
password: far1@FAR
```
2. Power on ODrive by connecting the hoverboard battery to the ODrive board.
3. ODrive Calibration:
- Make sure both wheels are off the ground and free to spin. (Put a book or a brick under the chassis).

- UPDATE (Nov 21st): I added a calibration script `startup_calibration.py`. You can just run that script from ~/dancerobot_ws/src/mobilehri2023/ and jump to next module.
- Open a terminal
```bash
$ odrivetool #(enter the odrive command interface)
```
- You should see something like the following:
<img src="docs/imgs/odrivetool.jpg">

If odrive is connected properly, you will see "Connected to Odrive [ID] as odrv0"

```python
# reset errors
$ odrv0.clear_errors()

# full calibration sequence includes motor calibration and hall encoder calibration.
$ odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
$ odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```
The left and right wheels should start spinning after a beeping sound. 
Make sure they are spinning freely.
```python
# Check if there are any errors. There shouldn't be any. 
$ dump_errors(odrv0)
```
```python
quit()
```

## Launch Dancing Module

In your terminal:
```bash
$ cd ~/dancerobot_ws/src/mobilehri2023
$ git pull # pull the latest changes. You can safely ignore local edits.
$ git checkout dancerobot
```

```bash
$ cd ~/dancerobot_ws
$ rm -r build/ install/ log/ # remove previously built packages, start with a clean start
$ colcon build --symlink-install 
# The symlink-install flag prevents rebuilding the package every time we make a small edit. 
$ source install/setup.bash
```

#### It is important that you source in every terminal you open:
source ~/dancerobot_ws/install/setup.bash

### Start dance server
```bash
ros2 launch dance_manager dance_server_launch.py
```
### Start mobile base (ROS starts to communicate with ODrive)
```bash
# in a seperate terminal
ros2 launch mobile_robot_control mobile_robot_launch.py
```

Now, the robot starts listening to the command to follow! You can test it with the following command:
```bash
# in a seperate terminal
ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"Zigzag", "params":"{\"direction\":\"forward\"}"}'
```
The robot should start walking forward.

### AI Choreographer
Generate full dance sequences from natural language using Google Gemini:
```bash
export GOOGLE_API_KEY="your-key-here"
ros2 run dance_manager choreographer "A playful, curious exploration"
```
Use `--dry-run` to preview without executing. See [docs/dance_manager.md](docs/dance_manager.md) for full details.

To add more moves, check out docs/dance_manager.md

---

## Simulation (Gazebo)

Make sure you have completed the [Build](#launch-dancing-module) steps and sourced the workspace before running the simulation. See [SIMULATION.md](SIMULATION.md) for full instructions.

---

## In field setup sequence for WiFi
1. start the Netgear Wifi hotspot
2. plug in the WiFi extender (big white box). Keep it next to the WiFi hotspot
3. Wait until the WiFi extender stops flashing, Netgear should reads ~ 3 devices connected
4. Turn on the RPis, they should connec to the hotspot automatically. SSH to each of them using the IP address written on the box.
5. Press the home button on the joystick, wait until it stops flashing (LED should be blue.) It should connect to the Joystick RPi directly.
6. Calibration
```bash
# In Hoverboard RPi
cd /home/dancerobot/dancerobot_ws/src/mobilehri2023
python3 startup_calibration.py
```

7. Now Launch!
```bash
# In Joystick RPi
ros2 launch mobile_robot_control joystick_launch.py
```

```bash
# In Hoverboard RPi
ros2 launch mobile_robot_control hoverboard_launch.py
```
