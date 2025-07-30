#!/usr/bin/env python3
import tkinter as tk
from tkinter import font
from tkinter import messagebox
import random

class TicTacToeBoard(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self._cells = {}
        self._create_board_GUI()
        self.grid_frame = tk.Frame(master=self, bg='white')
        self.grid_frame.pack(fill='both', expand=True)
        self.create_grid()
        self.sami_moves = []
        self.controls_frame = tk.Frame(master=self, height=70)
        self.controls_frame.pack(side=tk.BOTTOM, pady=5, fill=tk.X)
        self.current_turn = "Player"
        self.board_state = [None] * 9
        self.sami_turn_count = 0
        self.game_over = False
        self.config(bg="white")
        self.buttons_pressed = []
        self.cheat_mode = False
        self.cheated_this_game = False

    def _create_board_GUI(self):
        display_frame = tk.Frame(master=self, bg='white')
        display_frame.pack(fill=tk.X)
        self.display = tk.Label(
            master=display_frame,
            text='Player Ready?',
            font=font.Font(size=36, weight='bold'),
            fg='#0076bf',
            bg='white',
            bd=5,
            relief=tk.RIDGE
        )
        self.display.pack(fill=tk.X, pady=10)

    def create_grid(self):
        grid_inner_frame = tk.Frame(master=self.grid_frame, bg='white')
        grid_inner_frame.pack(expand=True)

        self.button_identities = []
        for row in range(3):
            grid_inner_frame.columnconfigure(row, weight=1)
            for col in range(3):
                grid_inner_frame.rowconfigure(col, weight=1)
                num = row * 3 + col
                button = tk.Button(
                    master=grid_inner_frame,
                    text='',
                    font=font.Font(size=60, weight='bold'),
                    fg='#ff0',
                    disabledforeground='black',
                    bg='white',
                    width=5,
                    height=2,
                    activebackground='white',
                    highlightbackground='black',
                    relief='flat',
                    borderwidth=2,
                    command=lambda n=num: self.record(n)
                )
                button.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')
                self.button_identities.append(button)

    def restart_game(self):
        self.buttons_pressed.clear()
        self.sami_moves.clear()
        self.current_turn = "Player"
        self.board_state = [None] * 9
        self.display.config(text='Player Ready?')
        self.game_over = False
        self.sami_turn_count = 0
        for button in self.button_identities:
            button.config(text='', state='normal')

    def run(self):
        self.mainloop()

    def record(self, button):
        pass  

    def disable_board(self):
        for button in self.button_identities:
            button.config(state='disabled')

    def check_winner(self, symbol):
        winning_combos = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],
            [0, 3, 6], [1, 4, 7], [2, 5, 8],
            [0, 4, 8], [2, 4, 6]
        ]
        for combo in winning_combos:
            if all(self.board_state[i] == symbol for i in combo):
                return True
        return False
