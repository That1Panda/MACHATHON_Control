#!/usr/bin/env python3
"""
Module to control the car using a keyboard
"""
import rospy
from std_msgs.msg import Float32

from sshkeyboard import listen_keyboard


class KeyboardControl:
    """
    Class to control the car using a keyboard

    Parameters
    ----------
    throttle_pub: rospy.Publisher
        Publisher to publish the throttle value
    steering_pub: rospy.Publisher
        Publisher to publish the steering value
    """

    def __init__(self, throttle_pub: rospy.Publisher, steering_pub: rospy.Publisher):
        self.throttle_pub = throttle_pub
        self.steering_pub = steering_pub
        self.throttle = 0
        self.steering = 0

    def key_pressed(self, key: str) -> None:
        """
        Callback for when a key is pressed

        Parameters
        ----------
        key : str
            The key that was pressed
        """
        if key == "w":
            self.throttle = 100
        if key == "s":
            self.throttle = -100
        if key == "a":
            self.steering = -20
        if key == "d":
            self.steering = 20
        if key == "q":
            self.throttle = 0
            self.steering = 0
        self.publish()

    def publish(self):
        """
        Publish the throttle and steering values
        """
        self.throttle_pub.publish(self.throttle)
        self.steering_pub.publish(self.steering)


def main():
    """
    Main function
    """
    rospy.init_node("keyboard_control")

    throttle_pub = rospy.Publisher("/throttle", Float32, queue_size=1)
    steering_pub = rospy.Publisher("/steering", Float32, queue_size=1)

    keyboard_control = KeyboardControl(throttle_pub, steering_pub)

    listen_keyboard(on_press=keyboard_control.key_pressed)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
