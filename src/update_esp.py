#!/usr/bin/env python3
"""
This node is used to update the esp settings over wifi
"""
import socket
import rospy


def main():
    """
    Main function to update the esp settings
    """
    rospy.init_node("update_esp", anonymous=True)
    ip_address = rospy.get_param("/ESP/IP")
    port = rospy.get_param("/ESP/port")
    server_address_port = (ip_address, port)
    udp_client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    to_send = [1]  # Command to update esp settings
    bt_name = rospy.get_param("/ESP/bluetooth_name")
    to_send.append(len(bt_name))
    for char in bt_name:
        to_send.append(ord(char))

    encoded_to_send = ""
    for command in to_send:
        encoded_to_send += chr(command)

    udp_client_socket.sendto(encoded_to_send.encode(), server_address_port)
    msg_from_server = str(udp_client_socket.recvfrom(1024)[0])[2:-1]
    print(msg_from_server)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
