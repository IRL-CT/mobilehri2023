# dance_manager — Documentation

This document provides a deeper look at the `dance_manager` package: its purpose, components, example workflows, and development notes.

## Overview

`dance_manager` coordinates dance choreography for the robot. It exposes an action server (based on the `Dance` action in `dance_interfaces`) which clients can use to request dance routines. The manager is responsible for validating goals, scheduling moves, providing feedback, and returning a result when the dance completes or is canceled.

Where to look in the source tree

- `dance_manager/dance_manager/dance_server.py` — main action server logic and orchestration, waiting for the client to send moves.
- `dance_manager/dance_manager/dance_client.py` — helper client code for sending a series of dance moves and handling feedback.
- `dance_manager/dance_manager/dance_moves.py` — pre-defined moves, mostly to keep the workspace organized. Can think of it as the library of dance moves.
- `dance_manager/launch/dance_server_launch.py` — launch file that starts the server node defined in `dance_server.py`.

## Adding new dance moves

> [Logic]
>
> Implement the dance move logic/function in `dance_moves.py`
>
> Register the new move function in `dance_server.py` and give it a name
>
> Envoke it through the dance client or CLI.

If you want to add new dance moves (new named behaviours that the `dance_server` can execute), follow these steps. The package keeps move implementations and choreography helpers in `dance_manager/dance_manager/dance_moves.py`.

1) Implement the move

 - Open `dance_manager/dance_manager/dance_moves.py` and add a new function that performs the motion. Keep the interface simple: accept any parameters you need (for example, `duration`, `speed`) and return a small status object or raise an exception on failure.

Requirements for this function:
- The function must take `twist_pub` as an input, with arbitrary other input parameters. The twist_pub will be used to publish commands to the robot
- The robot is differential-drive, which means it takes a twist command. It only reads two numbers from the twist command, `t.linear.x` and `t.angular.z`, which represent linear velocity and angular velocity, respectively. 

Example:

```python
def inch_forward(twist_pub):
    """Short forward "inch" motion: accelerate briefly, then decelerate.

    Generates a small forward movement by linearly ramping up ``linear.x`` over
    ~1s, then ramping down over ~0.5s. Concludes with a brief braking pulse to
    counter residual motion.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.

    Returns:
        None
    """
    t = Twist() # Initialize a twist message with all elements equal to zero.
    start = time.time()
    while time.time() <= start + 1:
        t.linear.x = time.time() - start # We set linear velocity to be proportional to time elapsed, since the robot accelerates forward
        twist_pub.publish(t) # We send the command to the robot
        time.sleep(0.1) # Use a sleep here to control the publishing frequency, values ranging from 0.05 to 0.1 are all acceptable. Do not affect performance.
    start = time.time()
    while time.time() <= start + 0.5:
        t.linear.x = 1 - (time.time() - start) * 2 # We decrease the speed, the robot decelerates.
        twist_pub.publish(t)  # We send the command to the robot
        time.sleep(0.05)

    abs_brake(twist_pub, direction=-1)
```

2) Register the name the server expects

 - In `dance_server.py`, the `dance_server` selects moves by name (the action goal field `dance_move`). In the current implementation, this mapping is an inline dictionary inside `execute_callback` named `dance_moves`:

```python
from dance_manager.dance_moves import inch_forward, inch_backward, tap_on_side, zigzagging_forward
...
dance_moves = {
    "InchForward": lambda: inch_forward(self.twist_pub),
    "InchBackward": lambda: inch_backward(self.twist_pub),
    "TapOnLeft": lambda: tap_on_side(self.twist_pub, side="left"),
    "TapOnRight": lambda: tap_on_side(self.twist_pub, side="right"),
    "ZigZaggingForward": lambda: zigzagging_forward(self.twist_pub)
}
```

 - To add a new move, implement the function in `dance_moves.py`, import it at the top of `dance_server.py`, then append a new key/value pair to this `dance_moves` dictionary (e.g. `"Spin": lambda: spin(self.twist_pub)`). Keeping the mapping in one place ensures discoverability.

3) Test with the action server

 - Start the `dance_server` (launch file provided):

```bash
ros2 launch dance_manager dance_server_launch.py
```

 - Send the move name from the CLI to verify the server selects and runs the move:

```bash
ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"inch_forward"}'
```

 - Alternatively, use the package's `DanceActionClient` (see `dance_client.py`) to send the same string programmatically and observe logs/feedback.

## Workflow examples

1) Start the server (on a machine with ROS 2 and the workspace sourced):

```bash
$ cd ~/dancebot_ws/src/mobilehri2023
$ source install/setup.bash
$ ros2 launch dance_manager dance_server_launch.py
# in a seperate terminal, remember to source
$ ros2 launch mobile_robot_control mobile_robot_launch.py
```

2) Send a goal from the CLI (fields depend on the action definition):

```bash
$ ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"ZigZaggingForward"}'
$ ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"InchForward"}'
```

3) Programmatic client example:
If you want to see how it looks like with multiple moves executed sequentially, use `dance_client.py`.
```python
def main(args=None):
    rclpy.init(args=args)

    action_client = DanceActionClient()
    
    # Send goals using the improved method
    print("Sending first move...")
    success1 = action_client.send_goal_and_wait("InchForward")
    print(f"First goal completed: {success1}")

    print("Sending second move...")
    success2 = action_client.send_goal_and_wait("ZigZaggingForward")
    print(f"second goal completed: {success2}")
    
    action_client.destroy_node()
    rclpy.shutdown()
```

```bash
$ cd ~/dancebot_ws/src/mobilehri2023/dance_manager/dance_manager
$ source ~/dancebot_ws/install/setup.bash
$ python3 dance_client.py
```
