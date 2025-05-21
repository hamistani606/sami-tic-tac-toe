#!/usr/bin/env python3

# ttt_Game.py
#
# Kyle Vickstrom
#
# This is the game node SAMI's tic tac toe game.
# This publishes a GameState msg whenever something changes.
# It waits for user inputs using a service call, which will then update the GameState msg.
# It will trigger a relevent service / action call to move the robot / play audio as needed.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
import random

from sami_ttt_msgs.msg import GameLog, GameState
from sami_ttt_msgs.srv import NewGame

class TicTacGame(Node):
    def __init__(self):
        super().__init__('ttt_game')
        self.logging = True
        self.GameState = None
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.pubGame = self.create_publisher(GameState, 'game_state', 10)
        self.newGame_srv = self.create_service(NewGame, 'new_game', self.newGame)
        # TODO: service calls: newGame (reset), player turn
        # TODO: how to check for win?
        # TODO: how to implement computer's turn

    def newGame(self, request, response):
        """
        This clears the game state and resets.
        TODO: this should be a service call
        """
        newGame = GameState()
        # randomly choose who goes first
        newGame.turn = random.randint(0, 1)
        response.turn = newGame.turn
        newGame.num_turns = 0
        
        # set score
        newGame.score = request.score

        # empty board
        newGame.board = [-1, -1, -1,
                        -1, -1, -1,
                        -1, -1, -1]

        self.GameState = newGame

        self.pubGame.publish(newGame)

        if newGame.turn == 0:
            self.log("New game created. My turn to go first!")
        else:
            self.log("New game created. Your turn to go first!")

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

def createGame(args=None):
    rclpy.init(args=args)
    game = TicTacGame()
    rclpy.spin(game)
    rclpy.shutdown()

if __name__ == "__main__":
    createGame()