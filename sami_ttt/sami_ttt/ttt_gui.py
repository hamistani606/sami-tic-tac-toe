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
        for row in range(3):
            # Adding specification for row sizes
            self.rowconfigure(row, weight=1, minsize=100)
            self.columnconfigure(row, weight=1, minsize=100)
            # Stores button ID for later use
            button_identities = []
            # Iterating through loop and creating all buttons
            for col in range(3):
                button = tk.Button(
                    master=grid_frame,
                    text='',
                    font=font.Font(size=125, weight='bold'),
                    fg='black',
                    width=3,
                    height=1,
                    highlightbackground='lightblue',
                    # Command which buttons run when pressed
                    command=add_symbol,
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
                button_identities.append(button)
            #print(button_identities)

    # Button to reset game
    def _create_board_restart(self):
        # Adding button to master frame
        restart_button = tk.Frame(master=self.tk)
        # Creating button Object
        restart_button = tk.Button(self.tk, text='Restart Game',
        font = font.Font(size=30, weight='bold'),
        command=send_Gamestate,
        )
        # Places button in center of frame
        restart_button.pack(padx=20,pady=20)
    # Running startup of all tasks within class
    def run(self):
        while rclpy.ok:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.tk.update_idletasks()
            self.tk.update()

# Restart fuction that sends gamestate update to .srv file
def send_Gamestate():
    print("The game has been restarted.")

# Adds symbol based on computer input
# Pulls from GameState.msg, uses button indices to place computer's symbol (may keep constant for first product)
def add_symbol():
    print("Worked")


# Defining GUI func
def main(args=None):
    rclpy.init(args=args)
    board = TicTacToeBoard()

# Starting process
if __name__ == '__main__':
    main()
