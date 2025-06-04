#!/usr/bin/env python3

# misty_control.py
# 
# Kyle Vickstrom
#
# This node handles connecting to the misty robot and sending animations / movements

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.time import Time

from mistyPy.Robot import Robot

from sami_ttt_msgs.msg import GameLog, GameState
from sami_ttt_msgs.srv import MistyConnect
from sami_ttt_msgs.action import MistyMovement

# These are needed if we're going to enable action cancelation.
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time

# TODO: Finish connection service
# TODO: confirm animation / moving action server (call one animation) works
# TODO: Add animation / movement bank based on emote type

class MistyControl(Node):
    def __init__(self):
        super().__init__('misty_control')
        # self.misty = None
        self.misty = Robot("192.168.0.174")
        self.connected = True
        #self.connected = False
        self.connect_srv = self.create_service(MistyConnect, 'misty_connect', self.connectMisty)
        
        #action server for running misty movement behaviors
        self.server = ActionServer(self, MistyMovement, 'mistymovement', self.behavior_callback, 
            callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_callback)

        # This is the callback the services the action request.
    def behavior_callback(self, goal):

        # Grab the logger and send a message to it.
        self.get_logger().info(f'Got {goal.request.behavior}')

        # Build a result to send the sequence to.  We can do this through the base
        # action type.  Once we build it, we set the return data, in sequence, to an
        # empty list.
        result = MistyMovement.Result()
        result.completion = 'in_progress'
        filename = 'install/sami_ttt/share/sami_ttt/behaviorbank/' + str(goal.request.behavior) + '.py'
        lines = []
        with open(filename,'r') as behaviorscript:
            lines = behaviorscript.readlines()


        self.misty.Speak(goal.request.words)
        while lines:
            # Check to see if we have a cancellation request.  If we do, set the goal
            # status to canceled and return an empty result.
            
            if goal.is_cancel_requested:
                goal.canceled()
                self.get_logger().info('And, the goal is canceled.')
                result.status = 'canceled'
                return MistyMovement.Result()

            # Because of the way the exec() works here, loops in the files do not work, theyll break code
            exec(lines.pop(0))
            
            result.status = 'running'
            goal.publish_feedback(MistyMovement.Feedback(progress=result.status))


        # Let the action server know that we've succeeded in the action.  It it doesn't
        # succeed, you can set other values here.
        goal.succeed()
        self.get_logger().info(f'Result: {result.completion}')

        # Return the result to the action server.
        return result

    # This callback fires when a cancellation request comes in.
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal')
        return CancelResponse.ACCEPT

    def connectMisty(self, request, response):
        """
        Service Server to connect to misty robot
        """
        if not self.connected:
            self.misty = Robot(request.ip)
            self.log("connected to misty?")
            self.connected = True
        else: 
            self.log("already connected to Misty!")
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
    rclpy.spin(mistyrobot, MultiThreadedExecutor())
    rclpy.shutdown()

#dummy display version of the command sending functionality in case Misty breaks 
def dummymovetest1():    
    goalis_cancel_requested = False
    goalrequestbehavior = 'dummytest1'
    # Grab the logger and send a message to it.
    print(f'Got {goalrequestbehavior}')
    # Build a result to send the sequence to.  We can do this through the base
    # action type.  Once we build it, we set the return data, in sequence, to an
    # empty list.
    filename = 'install/sami_ttt/share/sami_ttt/behaviorbank/'+str(goalrequestbehavior) + '.py'
    lines = []
    with open(filename,'r') as behaviorscript:
        lines = behaviorscript.readlines()
    while lines:
        # Check to see if we have a cancellation request.  If we do, set the goal
        # status to canceled and return an empty result.
        if goalis_cancel_requested:
            goalis_cancel_requested()
            print('And, the goal is canceled.')
            resultstatus = 'canceled'
        # Because of the way the exec() works here, loops in the files do not work, theyll break code
        exec(lines.pop(0))
        resultstatus = 'running'
        print(resultstatus)
    print('successful run')



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
