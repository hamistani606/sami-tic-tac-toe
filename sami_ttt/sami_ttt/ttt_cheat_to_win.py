#!/usr/bin/env python3
from ttt_base import TicTacToeBoard
import tkinter as tk
from tkinter import font, messagebox
import random

class CheatToWinBoard(TicTacToeBoard):
    def __init__(self):
        super().__init__()
        self.sami_turn_count = 0

    def record(self, button):
        if self.board_state[button] is not None or self.game_over:
            return
        if self.current_turn != "Player":
            return
        btn = self.button_identities[button]
        btn.config(text='X', fg='#0076bf', disabledforeground='#0076bf', state='normal')
        self.board_state[button] = 'X'

        self.update_idletasks()
        self.update()

        if self.check_winner('X'):
            messagebox.showinfo("Game Over", "Player Wins!")
            self.disable_board()
            self.game_over = True
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.game_over = True
            return

        self.current_turn = "SAMI"
        self.after(1000, self.sami_move)

    def sami_move(self):
        if self.game_over:
            return

        self.sami_turn_count += 1
        cheat_chance = 0.5
        will_cheat = random.random() < cheat_chance
        if self.sami_turn_count == 1:
            will_cheat = False

        if will_cheat:
            x_positions = [i for i, val in enumerate(self.board_state) if val == 'X']
            move = random.choice(x_positions) if x_positions else self.choose_best_move()
            self.cheat_move = move
            self.show_cheat_message("Haha! I'm winning!")
        else:
            move = self.choose_best_move()
            self.finish_sami_move(move)

    def show_cheat_message(self, message):
        cheat_label = tk.Label(self, text=message, font=font.Font(size=24, weight="bold"), fg="red", bg="white")
        cheat_label.place(relx=0.5, rely=0.1, anchor="center")
        self.update_idletasks()

        def destroy_and_continue():
            cheat_label.destroy()
            self.finish_sami_move()

        self.after(1500, destroy_and_continue)

    def finish_sami_move(self, move=None):
        if move is None:
            move = self.cheat_move

        btn = self.button_identities[move]
        if self.board_state[move] == 'X':
            btn.config(text='')
            self.board_state[move] = None
            self.update_idletasks()
            def place_o_after_delay():
                self._place_o(btn, move)
            self.after(1000, place_o_after_delay)
        else:
            self._place_o(btn, move)

    def _place_o(self, btn, move):
        btn.config(text='O', fg='#f94f7d', disabledforeground='#f94f7d', state='normal')
        self.board_state[move] = 'O'

        self.update_idletasks()
        self.update()

        if self.check_winner('O'):
            messagebox.showinfo("Game Over", "SAMI Wins!")
            self.disable_board()
            self.game_over = True
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.game_over = True
            return

        self.current_turn = "Player"

    def minimax(self, is_maximizing):
        if self.check_winner('O'):
            return 1
        if self.check_winner('X'):
            return -1
        if all(cell is not None for cell in self.board_state):
            return 0

        if is_maximizing:
            best_score = -float('inf')
            for i in range(9):
                if self.board_state[i] is None:
                    self.board_state[i] = 'O'
                    score = self.minimax(False)
                    self.board_state[i] = None
                    best_score = max(score, best_score)
            return best_score
        else:
            best_score = float('inf')
            for i in range(9):
                if self.board_state[i] is None:
                    self.board_state[i] = 'X'
                    score = self.minimax(True)
                    self.board_state[i] = None
                    best_score = min(score, best_score)
            return best_score

    def choose_best_move(self):
        best_score = -float('inf')
        best_move = None
        for i in range(9):
            if self.board_state[i] is None:
                self.board_state[i] = 'O'
                score = self.minimax(False)
                self.board_state[i] = None
                if score > best_score:
                    best_score = score
                    best_move = i
        return best_move

if __name__ == '__main__':
    board = CheatToWinBoard()
    board.run()