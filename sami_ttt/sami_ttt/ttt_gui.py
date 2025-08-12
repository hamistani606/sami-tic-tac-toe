#!/usr/bin/env python3

import tkinter as tk
from tkinter import font, messagebox
import random
from ttt_base import TicTacToeBoard
import os 
import threading
import math 

game_counter = 0
cheat_played = False

def destroy_cheat_message(self):
    if hasattr(self, 'cheat_label'):
        self.cheat_label.destroy()
        del self.cheat_label

    self.finish_cheat_move()

def show_cheat_message(self, message=None):
    cheat_lines = [
        "Haha! I'm winning!",
        "I'm just too smart for you!",
        "Oh no, did I do that?",
        "I can't help being this good!"
    ]
    self.cheat_line = random.choice(cheat_lines)
    
    self.cheat_label = tk.Label(
        self,
        text=self.cheat_line,
        font=font.Font(size=24, weight="bold"),
        fg="red",
        bg="white"
    )
    self.cheat_label.place(relx=0.5, rely=0.1, anchor="center")
    self.update_idletasks()

    play_text(self.cheat_line)

    self.after(2000, self.destroy_cheat_message)

def play_text(text):
    threading.Thread(
        # Range 150-160 wpm
        target=lambda: os.system(f'say -v Alex -r 160 "{text}"')
    ).start()

def estimate_speech_duration(text):
    words = len(text.split())
    duration_sec = words / 2.0  # ~2 words per second
    duration_ms = math.ceil(duration_sec * 1000)  # Convert to ms
    return duration_ms

class NormalGameBoard(TicTacToeBoard):
    def __init__(self, master=None, parent_app=None):
        super().__init__(master=master)
        self.parent_app = parent_app

        # Always fair in NormalGameBoard
        self.cheat_mode = False
        self.cheat_strength = 0.0

        # Skill: minimax 60% of turns
        self.use_minimax = True
        self.minimax_prob = .6

    def restart_game(self):
        self.cheated_this_game = False
        self.sami_turn_count = 0
        super().restart_game()

    def auto_reset_board(self):
        self.after(500, self.restart_game)

    def record(self, button):
        if self.board_state[button] is not None or self.game_over:
            return
        if self.current_turn != "Player":
            return

        # Show player's turn label here
        self.display.config(text="Your Turn!", fg="#0076bf")
        self.update_idletasks()

        btn = self.button_identities[button]
        btn.config(text='X', fg='#0076bf', disabledforeground='#0076bf', state='normal')
        self.board_state[button] = 'X'

        self.update_idletasks()
        self.update()

        if self.check_winner('X'):
            messagebox.showinfo("Game Over", "Player Wins!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("Player")
            # self.auto_reset_board()
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.disable_board()
            self.game_over = True
            # self.auto_reset_board()
            if self.parent_app:
                self.parent_app.update_score("Tie")
            return

        self.current_turn = "SAMI"
        def show_sami_and_move():
            self.display.config(text="SAMI's Turn!", fg="#f94f7d")
            self.update_idletasks()
            self.after(1000, self.sami_move)
        show_sami_and_move()

    def sami_move(self):
        if self.game_over:
            return

        self.display.config(text="SAMI's Turn!")
        self.update_idletasks()

        if not hasattr(self, 'sami_turn_count'):
            self.sami_turn_count = 0
        self.sami_turn_count += 1

        will_cheat = self.cheat_mode and self.sami_turn_count > 1 and random.random() < self.cheat_strength

        if will_cheat:
            x_positions = [i for i, val in enumerate(self.board_state) if val == 'X']
            if x_positions:
                move = random.choice(x_positions)
            else:
                move = self.choose_best_move()
            self.cheat_move = move
            self.cheated_this_game = True
            self.show_cheat_message("Haha! I'm winning!")
            return

        empty_indices = [i for i, val in enumerate(self.board_state) if val is None]
        if not empty_indices:
            messagebox.showinfo("Game Over", "It's a draw!")
            self.game_over = True
            self.auto_reset_board()
            return

        play_random = random.random() < 0.1
        use_minimax_now = self.use_minimax and random.random() < 0.7  # 60% chance to use minimax
        if use_minimax_now:
            move = self.choose_best_move()
        else:
            move = random.choice(empty_indices)

        # Pick SAMI move line
        turn_lines = [
            "It's my turn now!",
            "Let me show you how it's done!",
            "My move! Watch closely.",
            "Watch and learn!",
            "Time for my genius move!",
            "I’ve got this one!",
            "Prepare to be amazed!",
            "Sit back and watch the master at work!",
            "I bet you didn't see this coming!",
            "Uh-oh… I’m on fire!",
            "Don't blink or you'll miss it!",
            "Let’s spice things up!",
            "Beep boop… my turn!",
            "Activating winning sequence!",
            "Executing superior logic!"
        ]
        line = random.choice(turn_lines)
        play_text(line)
        delay_ms = estimate_speech_duration(line) + 500

        def place_and_switch():

            your_turn_lines = [
                "Your turn!",
                "Go ahead!",
                "Let's see what you can do!",
                "Now it's your move!",
                "Your move! Make it count!",
                "Don’t keep me waiting!",
                "Can you outsmart me?",
                "Ready to surprise me?",
                "Feeling lucky?",
                "Think you can beat that?",
                "Human move detected... go!",
                "Your algorithm turn!",
                "Processing... Awaiting your input!",
                "Execute your move, human!",
            ]
            line = random.choice(your_turn_lines)
            btn = self.button_identities[move]
            btn.config(text='O', fg='#f94f7d', disabledforeground='#f94f7d', state='normal')
            self.board_state[move] = 'O'
            self.update_idletasks()
            self.update()

            if self.check_winner('O'):
                messagebox.showinfo("Game Over", "SAMI Wins!")
                self.disable_board()
                self.game_over = True
                if self.parent_app:
                    self.parent_app.update_score("SAMI")
                else:
                    self.auto_reset_board()
                return

            if all(cell is not None for cell in self.board_state):
                messagebox.showinfo("Game Over", "It's a draw!")
                self.disable_board()
                self.game_over = True
                if self.parent_app:
                    self.parent_app.update_score("Tie")
                else:
                    self.auto_reset_board()
                return

            self.current_turn = "Player"
            self.display.config(text="Your Turn!", fg="#0076bf")
            self.update_idletasks()
            play_text(line)

        self.after(delay_ms, place_and_switch)

    def destroy_cheat_message(self):
        if hasattr(self, 'cheat_label'):
            self.cheat_label.destroy()
            del self.cheat_label
        self.finish_cheat_move()

    def show_cheat_message(self, message=None):
        cheat_lines = [
            "Haha! I'm winning!",
            "I'm just too smart for you!",
            "Oh no, did I do that?",
            "I can't help being this good!",
            "Oops! Did I cheat? Silly me!",
            "Winning feels so easy!",
            "Oh, look at me go!",
            "You didn’t see that coming, did you?",
            "Oops! I might be too good at this.",
            "What? Me? Cheat? Never!",
            "I call that… strategic brilliance!",
            "I'm unstoppable today!",
            "Just a little advantage for me!",
            "Don't be mad, it's just a game!",
            "Oopsies! Did I skip the rules again?",
            "Sshh... don't tell anyone!",
            "I promise I'll play fair next time… maybe.",
            "Sometimes you've gotta bend the rules!",
            "Calculating victory... Success!",
            "My code says I win!",
            "Did my circuits misfire? Oh well!"
        ]

        self.cheat_line = random.choice(cheat_lines)
        if message is None:
            message = self.cheat_line

        self.cheat_label = tk.Label(
            self,
            text=message,
            font=font.Font(size=24, weight="bold"),
            fg="red",
            bg="white"
        )
        self.cheat_label.place(relx=0.5, rely=0.1, anchor="center")
        self.update_idletasks()

        play_text(self.cheat_line)

        self.after(1500, self.destroy_cheat_message)

    def glitch_effect(self, step=0):
        if step < 5:
            glitch_chars = ['#', '*', '%', '@']
            self.cheat_button.config(
                text=random.choice(glitch_chars),
                fg='red' if step % 2 == 0 else 'black',
                bg='white'
            )
            self.update_idletasks()
            self.after(100, lambda: self.glitch_effect(step + 1))
        else:
            self.cheat_button.config(text='', bg='white')
            self.board_state[self.cheat_move] = None
            self.update_idletasks()
            self.after(300, lambda: self._place_o(self.cheat_button, self.cheat_move))

    def finish_cheat_move(self):
        move = self.cheat_move
        self.cheat_button = self.button_identities[move]

        if self.board_state[move] == 'X':
            self.glitch_effect(step=0)
        else:
            self._place_o(self.cheat_button, move)

    def _place_o(self, btn, move):
        btn.config(text='O', fg='#f94f7d', disabledforeground='#f94f7d', state='normal')
        self.board_state[move] = 'O'
        self.update_idletasks()
        self.update()

        if self.check_winner('O'):
            messagebox.showinfo("Game Over", "SAMI Wins!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("SAMI")
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("Tie")
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

def main():
    board = NormalGameBoard()
    board.run()

if __name__ == '__main__':
    print("This file defines NormalGameBoard. Please run ttt_home_page.py to start the app.")