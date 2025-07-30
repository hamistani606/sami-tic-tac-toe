#!/usr/bin/env python3

import tkinter as tk
from tkinter import font, messagebox
import random
from ttt_gui import NormalGameBoard, play_text, estimate_speech_duration

class CheatToLoseBoard(NormalGameBoard):
    def __init__(self, master=None, parent_app=None):
        super().__init__(master=master, parent_app=parent_app)
        self.cheat_mode = True
        self.use_minimax = False
        self.cheat_strength = 1.0
        self.sami_turn_count = 0

    def sami_move(self):
        if self.game_over:
            return

        self.sami_turn_count += 1

        cheat_chance = 0.8
        will_cheat = random.random() < cheat_chance

        empty_positions = [i for i, val in enumerate(self.board_state) if val is None]

        if will_cheat and empty_positions:
            move = random.choice(empty_positions)
            self.cheat_move = move
            self.show_cheat_message("Let me give you a chance")
            soft_lines = [
                "Wow… struggling already? Let me help you out.",
                "Fine, I’ll make this easy for you.",
                "I guess I’ll take a dive—just this once.",
                "Ugh, do I have to lose to keep it interesting?",
                "Here… have a freebie. You clearly need it.",
                "Is this how humans feel mercy?",
                "You're welcome in advance.",
                "Can’t believe I’m doing this for you.",
                "Losing on purpose? Iconic.",
                "I’m practically gift-wrapping this win."
            ]
            line = random.choice(soft_lines)
            play_text(line)
            delay_ms = estimate_speech_duration(line) + 500
            self.after(delay_ms, lambda: self.show_cheat_message(line))
        else:
            # Normal random bad move
            move = random.choice(empty_positions)
            btn = self.button_identities[move]
            self._place_o(btn, move)

    def show_cheat_message(self, message):
        cheat_label = tk.Label(self, text=message, font=font.Font(size=24, weight="bold"), fg="gray", bg="white")
        cheat_label.place(relx=0.5, rely=0.1, anchor="center")
        self.update_idletasks()

        def destroy_and_continue():
            cheat_label.destroy()
            self.finish_sami_cheat()

        self.after(1500, destroy_and_continue)

    def finish_sami_cheat(self):
        move = self.cheat_move
        btn = self.button_identities[move]

        def glitch_effect(step=0):
            if step < 4:
                glitch_chars = ['X', '*', '#', '%']
                btn.config(text=random.choice(glitch_chars), fg='gray' if step % 2 == 0 else 'black', bg='white')
                self.update_idletasks()
                self.after(100, lambda: glitch_effect(step + 1))
            else:
                btn.config(text='X', bg='white')  # Mistakenly put an X!
                self.board_state[move] = 'X'
                self.update_idletasks()
                self.after(300, self.check_game_end)

        glitch_effect()

    def check_game_end(self):
        if self.check_winner('X'):
            messagebox.showinfo("Game Over", "Player Wins!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("Player")
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("Tie")
            return

        self.current_turn = "Player"
