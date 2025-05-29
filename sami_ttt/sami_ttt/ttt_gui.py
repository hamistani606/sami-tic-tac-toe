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
        self.sami_moves = []
        self.logging = True
        self.controls_frame = tk.Frame(master=self.tk)
        self.controls_frame.pack(side=tk.BOTTOM, pady=20)
        self._create_difficulty_menu()
        self._create_board_restart()
        self.newGame_client = self.create_client(NewGame, 'new_game')
        self.newTurn_client = self.create_client(PlayerTurn, 'player_turn')
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.subGame = self.create_subscription(GameState, 'game_state', self.update_game, 10)
        self.Game_Info = None

        while not self.newGame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for new_game service...')
        request = NewGame.Request()
        request.difficulty = self.difficulty_var.get()
        self.newGame_client.call_async(request)
        self.log("Requested new game on GUI startup.")

        self.run()

    def update_game(self, Gameinfo):
        """
        This is the loop that updates the GUI with information recieved from other Ros nodes.
        """
        # Adds new game info to instance variable
        self.Game_Info = Gameinfo
        # Loop over game board for unplayed moves, check unrecognized moves
        for idx, mv in enumerate(self.Game_Info.board):
            if mv == 0 and idx not in self.sami_moves:
                    self.sami_moves.append(idx)
                    self.buttons_pressed.append(idx)
                    self.button_identities[idx].config(text='O')

        if self.Game_Info.turn is None:
            return
        # Draw counter
        sami_wins = self.Game_Info.score[0]
        player_wins = self.Game_Info.score[1]
        # Win counter label
        if not hasattr(self, 'win_counter'):
            self.win_counter = tk.Label(self.controls_frame, font=font.Font(size=16))
            self.win_counter.pack(side=tk.RIGHT, padx=(20, 0))
        self.win_counter.config(text=f"Sami Wins: {sami_wins}, Player Wins: {player_wins}")

    
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
        self.buttons_pressed = []
        for row in range(3):
            # Adding specification for row sizes
            self.rowconfigure(row, weight=1, minsize=100)
            self.columnconfigure(row, weight=1, minsize=100)
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
                button.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')
                # List containing button IDs
                self.button_identities.append(button)

    # Button to reset game
    def _create_board_restart(self):
        # Creating button Object
        restart_button = tk.Button(self.controls_frame, text='Restart',
        font = font.Font(size=16, weight='bold'), command=self.restart_game
        )
        # Places button in center of frame
        restart_button.pack(side=tk.LEFT, padx=(20, 0))

    # Difficulty menu
    def _create_difficulty_menu(self):
        # Difficulty label and menu at bottom center
        self.difficulty_var = tk.StringVar(value="medium")
        difficulty_label = tk.Label(
            master=self.controls_frame,
            text="Difficulty:",
            font=font.Font(size=16)
        )
        difficulty_label.pack(side=tk.LEFT, padx=(0, 10))
        difficulty_menu = tk.OptionMenu(self.controls_frame, self.difficulty_var, "easy", "medium", "hard")
        difficulty_menu.config(font=font.Font(size=14))
        difficulty_menu.pack(side=tk.LEFT)

        
    # Running startup of all tasks within class
    def run(self):
        while rclpy.ok():
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

    def restart_game(self):
        if not self.newGame_client.wait_for_service(timeout_sec=1.0):
            self.log("NewGame service not available.")
            return
        # New game request
        request = NewGame.Request()
        request.difficulty = self.difficulty_var.get()
        self.newGame_client.call_async(request)
        self.log(f"Restart button pressed – new game requested with difficulty: {request.difficulty}")
        # Resetting board
        self.Game_Info = None
        self.buttons_pressed.clear()
        self.sami_moves.clear()
        for button in self.button_identities:
            button.config(text='')


# Defining GUI func
def main(args=None):
    rclpy.init(args=args)
    board = TicTacToeBoard()

def record(button, buttons_pressed, button_identities, self):
    # Check if it is player turn
    if self.Game_Info is None:
        self.log("Game state not initialized yet.")
        return
    
    if self.Game_Info.turn != 1 or not self.newTurn_client.wait_for_service(timeout_sec=1):
        self.log("Not player's turn or turn service not available.")
        return
    
    if button not in buttons_pressed:
        buttons_pressed.append(button)
        button_identities[button].config(text='X')
    # Button index translated to [0-8]
    location = button
    newTurn = PlayerTurn.Request()
    newTurn.player_id = 1
    newTurn.location = location
    # Make request on server for Kyle code to change .msg file
    self.newTurn_client.call_async(newTurn)
    self.log(f"Player turn on {location}")

# Starting process
if __name__ == '__main__':
    main()