#!/usr/bin/env python3

# ttt_Game.py
#
# John Von Worley
#
# This is the GUI node for monitor
# This publishes a GameState msg when the user input is recieved
# Also publishes the gamestate when the robot makes a move

import tkinter as tk
from tkinter import font
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from typing import NamedTuple
from itertools import cycle

# Importing information from .srv files
from sami_ttt_msgs.msg import GameLog, GameState
from sami_ttt_msgs.srv import NewGame, PlayerTurn

# Board GUI, inludes all buttons which are triggered by cursor
class TicTacToeBoard(tk.Tk, Node):
    # Startup functions
    def __init__(self):
        self.tk = tk.Tk()
        Node.__init__(self, 'ttt_gui')
        self.title("Tic-Tac-Toe Game")
        self._cells = {}
        self._create_board_GUI()
        self.create_grid()
        self._create_board_restart()
        self.newGame_client = self.create_client(NewGame, 'new_game')
        self.newTurn_client = self.create_client(PlayerTurn, 'player_turn')
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.subGame = self.create_subscription(GameState, 'game_state', self.update_game, 10)
        self.Game_Info = None
        self.run()

    def update_game(self, Gameinfo):
        """
        This is the loop that updates the GUI with information recieved from other Ros nodes.
        """
        # Adds new game info to instance variable
        self.Game_Info = Gameinfo
        self.Game_Info.turn
        self.draw_symbol()
        self.display_wins()

    # Creates master frame (window) that all other items are added to
    def _create_board_GUI(self):
        display_frame = tk.Frame(master=self.tk)
        display_frame.pack(fill=tk.X)
        # Frame
        self.display = tk.Label(
            master=display_frame,
            text='Player Ready?',
            font=font.Font(size=28, weight= 'bold'),
        )
        # Setting size and placing
        self.geometry("1920x1080")
        self.display.pack()

    def create_grid(self):
        # Adding grid to master frame and centering in middle
        grid_frame = tk.Frame(master=self.tk)
        grid_frame.pack()
        self.button_identities = []
        for row in range(3):
            # Adding specification for row sizes
            self.rowconfigure(row, weight=1, minsize=100)
            self.columnconfigure(row, weight=1, minsize=100)
            # Stores button ID for later use
            self.buttons_pressed = []
            # Iterating through loop and creating all buttons
            for col in range(3):
                num = row * 3 + col
                button = tk.Button(
                    master=grid_frame,
                    text='',
                    font=font.Font(size=125, weight='bold'),
                    fg='black',
                    width=3,
                    height=1,
                    highlightbackground='lightblue',
                    command=lambda n=num: record(n, self.buttons_pressed, self.button_identities, self)
                )
                # Placing buttons on actual grid
                self._cells[button] = (row, col)
                button.grid(
                    row=row,
                    column=col,
                    padx=5,
                    pady=5,
                    sticky='nsew'
                )
                # List containing button IDs
                self.button_identities.append(button)


    # Button to reset game
    def _create_board_restart(self):
        # Adding button to master frame
        restart_button = tk.Frame(master=self.tk)
        # Creating button Object
        restart_button = tk.Button(self.tk, text='Restart Game',
        font = font.Font(size=30, weight='bold')
        )
        # Places button in center of frame
        restart_button.pack(padx=20,pady=20)
        
    def display_wins():
        if self.Game_Info == None:
            return
        # Draw couter
        sami_wins = self.Game_Info.score[0]
        player_wins = self.Game_Info.score[1]

    # Running startup of all tasks within class
    def run(self):
        while rclpy.ok:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.tk.update_idletasks()
            self.tk.update()

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

# Defining GUI func
def main(args=None):
    rclpy.init(args=args)
    board = TicTacToeBoard()

def record(button,buttons_pressed,button_identities, self):
    # Check if it is player turn
    if self.Game_Info == 0 or not self.newTurn_client.wait_for_service(timeout_sec=1):
        return
    if button not in buttons_pressed:
        buttons_pressed.append(button)
        print(button_identities,button)
        print(f"stored: {buttons_pressed}")
        button_identities[button].config(text='X')
    # Button index translated to [0-8]
    location = button
    newTurn = PlayerTurn.Request()
    newTurn.location = location
    # Make request on server for Kyle code to change .msg file
    self.newTurn_client.call_async(newTurn)
    self.log(f"Player turn on {location}")


# Starting process
if __name__ == '__main__':
    main()
