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
$ cd ~/dancebot_ws/src/mobilehri2023
$ git pull # pull the latest changes. You can safely ignore local edits.
$ git checkout dancerobot
```

```bash
$ cd ~/dancebot_ws
$ rm -r build/ install/ log/ # remove previously built packages, start with a clean start
$ colcon build --symlink-install 
# The symlink-install flag prevents rebuilding the package every time we make a small edit. 
$ source install/setup.bash
```

#### It is important that you source in every terminal you open:
source ~/dancebot_ws/install/setup.bash

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
ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"ZigZaggingForward"}'
```
The robot should start walking forward.

To add more moves, check out docs/dance_manager.md
