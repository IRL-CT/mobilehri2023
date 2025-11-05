import time
from geometry_msgs.msg import Twist

def stepping_forward(twist_pub):
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

    for i in range(10):
        t.linear.x = -0.1
        twist_pub.publish(t)
        time.sleep(0.05)

def stepping_backward(twist_pub):
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

    for i in range(10):
        t.linear.x = 0.1
        twist_pub.publish(t)
