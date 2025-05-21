#!/usr/bin/env python3

# misty_control.py
# 
# Kyle Vickstrom
#
# This node handles connecting to the misty robot and sending animations / movements

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from mistyPy.Robot import Robot

from sami_ttt_msgs.msg import GameLog, GameState
from sami_ttt_msgs.srv import MistyConnect


# TODO: Add dependecy to README
# TODO: Add connection service
# TODO: Add animation / moving action server (call one animation)
# TODO: Add animation / movement bank based on emote type

class MistyControl(Node):
    def __init__(self):
        super().__init__('misty_control')
        self.misty = None
        self.connect_srv = self.create_service(MistyConnect, 'misty_connect', self.connectMisty)


    def connectMisty(self, request, response):
        """
        Service Server to connect to misty robot
        """
        self.misty = Robot(request.ip)
        self.log("connected to misty?")
        return response

    def log(self, msg):
            """
            log to topic and to curses terminal window
            """
            if self.logging:
                newmsg = GameLog()
                newmsg.stamp = self.get_clock().now().to_msg()
                newmsg.node_name = self.get_name()
                newmsg.content = msg
                self.pubLog.publish(newmsg)
            self.get_logger().info(msg)

def createMisty(args=None):
    rclpy.init(args=args)
    mistyrobot = MistyControl()
    rclpy.spin(mistyrobot)
    rclpy.shutdown()

if __name__ == "__main__":
    createMisty()

"""
from mistyPy.Robot import Robot

def start_skill():
    current_response = misty.MoveArms(50, 150)
    print(current_response)
    print(current_response.status_code)
    print(current_response.json())
    print(current_response.json()["result"])
    current_response = misty.MoveHead(-40, 30 ,0 ,100)
    print(current_response)
    print(current_response.status_code)
    print(current_response.json())
    print(current_response.json()["result"])
    
def main():
    #misty.MoveArms(50, 150)
    misty.MoveArms(leftArmPosition= -29, rightArmPosition= 90)
    misty.MoveArms(leftArmPosition= 90, rightArmPosition= -29)
    misty.MoveArms(leftArmPosition= -29, rightArmPosition= 90)
    misty.MoveHead(26, 30 ,0 ,100)

def drive():
    misty.DriveTime(linearVelocity= 80, angularVelocity= 0, timeMs= 500)
    
    misty.DriveTime(linearVelocity= -80, angularVelocity= 0, timeMs= 500)
    
    
if __name__ == "__main__":
    ipAddress = "192.168.0.174"
    misty = Robot(ipAddress)
    drive()
    misty.stop()
"""