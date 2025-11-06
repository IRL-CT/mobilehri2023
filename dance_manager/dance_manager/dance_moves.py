import time
from geometry_msgs.msg import Twist

def abs_brake(twist_pub, direction, brake_times=10, pause_duration=0.05):
    '''
    direction: 1 for forward, -1 for backward
    '''
    t = Twist()
    for i in range(brake_times):
        t.linear.x = direction * 0.1
        twist_pub.publish(t)
        time.sleep(pause_duration)


def inch_forward(twist_pub):
    t = Twist()
    start = time.time()
    while time.time() <= start + 1:
        t.linear.x = time.time() - start
        twist_pub.publish(t)
        time.sleep(0.1)
    start = time.time()
    while time.time() <= start + 0.5:
        t.linear.x = 1 - (time.time() - start) * 2
        twist_pub.publish(t)
        time.sleep(0.05)

    abs_brake(twist_pub, direction=-1)

def inch_backward(twist_pub):
    t = Twist()
    start = time.time()
    while time.time() <= start + 1:
        t.linear.x = -(time.time() - start)
        twist_pub.publish(t)
        time.sleep(0.1)
    
    start = time.time()
    while time.time() <= start + 0.5:
        t.linear.x = -1 * (1 - (time.time() - start) * 2)
        twist_pub.publish(t)
        time.sleep(0.05)

    abs_brake(twist_pub, direction=1)

def tap_on_side(
    twist_pub,
    side="left",          # "left" or "right"
    tap_times=1,
    tap_duration=0.5,     # seconds per stroke (forward or backward)
    pause_duration=0.10,  # pause between taps
    v_mag=0.40,           # m/s linear speed of the moving wheel
    track=0.51,           # meters; distance between wheels
    cmd_dt=0.05           # command period
):
    """
    Tap on one wheel (left or right) while the other stays fixed.

    side = "left":  pivots about RIGHT wheel (v_R = 0)
        v =  v_L / 2
        w = -v_L / track

    side = "right": pivots about LEFT wheel (v_L = 0)
        v =  v_R / 2
        w =  v_R / track
    """
    t = Twist()

    def send_for(duration, v_side):
        # Determine v, w based on which wheel is active
        if side.lower() == "left":
            v = 0.5 * v_side
            w = -v_side / track
        elif side.lower() == "right":
            v = 0.5 * v_side
            w =  v_side / track
        else:
            raise ValueError("side must be 'left' or 'right'")
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    # Perform tap motions
    for _ in range(tap_times):
        # forward stroke
        send_for(tap_duration, v_mag)
        abs_brake(twist_pub, direction=-1)

        # backward stroke
        send_for(tap_duration, -v_mag)
        abs_brake(twist_pub, direction=1)


def zigzagging_forward(
    twist_pub, 
    turns=1,
    tap_duration=0.5,     # seconds per stroke (forward or backward)
    pause_duration=0.10,  # pause between taps
    v_mag=0.40,           # m/s linear speed of the moving wheel
    track=0.51,           # meters; distance between wheels
    cmd_dt=0.05           # command period
):
    t = Twist()

    def left_feet_forward(duration):
        v = 0.5 * v_mag
        w = -v_mag / track
        
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    def right_feet_forward(duration):
        v = 0.5 * v_mag
        w =  v_mag / track
        
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    # Perform tap motions
    for _ in range(turns):
        # forward stroke
        left_feet_forward(tap_duration)
        abs_brake(twist_pub, direction=-1)

        # backward stroke
        right_feet_forward(tap_duration)
        abs_brake(twist_pub, direction=-1)

# Simple roll
# Pivoting 
# zigzaggingforward and backwards
# Sweeping with pause (left and right) the looking
# Slalom (front and back)
# Spinning in place 
# Spinning in one foot (front and back)
# Free form skating around
# Pacing or patrolling