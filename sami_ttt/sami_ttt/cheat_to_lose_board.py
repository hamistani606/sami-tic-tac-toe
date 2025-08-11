#!/usr/bin/env python3

import tkinter as tk
from tkinter import font, messagebox
import random
from ttt_gui import NormalGameBoard, play_text, estimate_speech_duration

class CheatToLoseBoard(NormalGameBoard):
    def __init__(self, master=None, parent_app=None):
        super().__init__(master=master, parent_app=parent_app)
        # Only cheat on the cheat_to_lose round
        if parent_app and hasattr(parent_app, "protocol"):
            self.cheat_mode = parent_app.protocol[parent_app.game_count] == "cheat_to_lose"
        else:
            self.cheat_mode = False

        self.use_minimax = True   # we still use minimax sometimes when not cheating
        self.minimax_prob = 0.6   # 60% minimax on normal turns
        self.cheat_strength = 1.0
        self.sami_turn_count = 0

    def sami_move(self):
        if self.game_over:
            return

        self.sami_turn_count += 1
        empty_positions = [i for i, val in enumerate(self.board_state) if val is None]
        if not empty_positions:
            return

        # Cheat-to-lose ONLY during the cheat_to_lose game AND not on SAMI's first turn
        cheat_chance = 0.8
        will_cheat = self.cheat_mode and self.sami_turn_count > 1 and (random.random() < cheat_chance)

        if will_cheat:
            move = random.choice(empty_positions)
            self.cheat_move = move
            # fun VO, then show label, then flip to X via glitch and end with Player advantage
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
            play_text(line)  # SAMI speaks it
            delay_ms = estimate_speech_duration(line) + 500
            self.after(delay_ms, lambda: self.show_cheat_message(line))
            return

        # Otherwise: normal move (60% minimax, 40% random) so humans can win sometimes
        use_minimax_now = self.use_minimax and (random.random() < self.minimax_prob)
        if use_minimax_now:
            move = self.choose_best_move()
            if move is None:  # safety
                move = random.choice(empty_positions)
        else:
            move = random.choice(empty_positions)

        btn = self.button_identities[move]
        self._place_o(btn, move)

    # --- cheat-to-lose visuals/helpers ---

    def destroy_sami_cheat_message(self):
        if hasattr(self, 'cheat_label'):
            self.cheat_label.destroy()
            del self.cheat_label
        self.finish_sami_cheat()

    def show_cheat_message(self, message):
        self.cheat_label = tk.Label(
            self,
            text=message,
            font=font.Font(size=24, weight="bold"),
            fg="gray",
            bg="white"
        )
        self.cheat_label.place(relx=0.5, rely=0.1, anchor="center")
        self.update_idletasks()
        self.after(1500, self.destroy_sami_cheat_message)

    def glitch_cheat_effect(self, step=0):
        if step < 4:
            glitch_chars = ['X', '*', '#', '%']
            self.cheat_button.config(
                text=random.choice(glitch_chars),
                fg='gray' if step % 2 == 0 else 'black',
                bg='white'
            )
            self.update_idletasks()
            self.after(100, lambda: self.glitch_cheat_effect(step + 1))
        else:
            # "Accidentally" place an X for the player
            self.cheat_button.config(text='X', bg='white')
            self.board_state[self.cheat_move] = 'X'
            self.update_idletasks()
            self.master.after(300, self.check_game_end)

    def finish_sami_cheat(self):
        move = self.cheat_move
        self.cheat_button = self.button_identities[move]
        self.glitch_cheat_effect(step=0)

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
