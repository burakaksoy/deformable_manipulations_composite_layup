#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

from pynput import keyboard

def main():
    rospy.init_node('teleop_twist_keyboard_linear', anonymous=True)

    # Parameters
    rate_value = rospy.get_param('~rate', 100)       # Publishing rate (Hz)
    topic_name = rospy.get_param('~topic', '/spacenav2/twist')
    speed = rospy.get_param('~speed', 0.1)          # Linear speed (m/s)
    key_timeout = rospy.get_param('~key_timeout', 0.5)

    pub = rospy.Publisher(topic_name, Twist, queue_size=1)
    rate = rospy.Rate(rate_value)

    current_twist = Twist()
    # Initialize all linear components to 0; no angular velocities
    current_twist.linear.x = 0.0
    current_twist.linear.y = 0.0
    current_twist.linear.z = 0.0
    current_twist.angular.x = 0.0
    current_twist.angular.y = 0.0
    current_twist.angular.z = 0.0

    # Track time of last pressed key
    last_pressed_time = time.time()

    # Set to store currently pressed keys
    pressed_keys = set()

    def update_twist():
        """ Update Twist message based on currently pressed keys. """
        nonlocal current_twist, last_pressed_time

        # Reset linear velocity
        current_twist.linear.x = 0.0
        current_twist.linear.y = 0.0
        current_twist.linear.z = 0.0

        # Check active keys and sum velocities accordingly
        if 'w' in pressed_keys:
            current_twist.linear.x += speed
        if 's' in pressed_keys:
            current_twist.linear.x -= speed
        if 'a' in pressed_keys:
            current_twist.linear.y += speed
        if 'd' in pressed_keys:
            current_twist.linear.y -= speed
        if 'q' in pressed_keys:
            current_twist.linear.z -= speed
        if 'e' in pressed_keys:
            current_twist.linear.z += speed

        # If any key is pressed, update last_pressed_time
        if pressed_keys:
            last_pressed_time = time.time()

    def on_press(key):
        """ Callback when a key is pressed. """
        try:
            if key.char in ['w', 's', 'a', 'd', 'q', 'e']:
                pressed_keys.add(key.char)
                update_twist()
        except AttributeError:
            pass  # Ignore special keys (Shift, Ctrl, etc.)

    def on_release(key):
        """ Callback when a key is released. """
        try:
            if key.char in pressed_keys:
                pressed_keys.remove(key.char)
                update_twist()
        except AttributeError:
            pass

    # Start listening for keyboard input
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rospy.loginfo("teleop_twist_keyboard_linear node started.")
    rospy.loginfo("Press 'w/s' for +/- X, 'a/d' for +/- Y, 'q/e' for +/- Z.\n"
                  "Speed: %.2f m/s\n"
                  "Publishing on topic: %s\n"
                  "Timeout: %.2f seconds",
                  speed, topic_name, key_timeout)

    while not rospy.is_shutdown():
        # If no key is pressed for `key_timeout` seconds, reset velocities
        if (time.time() - last_pressed_time) > key_timeout:
            current_twist.linear.x = 0.0
            current_twist.linear.y = 0.0
            current_twist.linear.z = 0.0

        # Publish the current velocity
        pub.publish(current_twist)
        rate.sleep()

if __name__ == '__main__':
    main()