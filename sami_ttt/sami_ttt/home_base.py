# home_base.py

import tkinter as tk
from tkinter import font, messagebox
from ttt_gui import NormalGameBoard

class HomePage:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Tic-Tac-Toe XOXO SAMI")
        self.player_score = 0
        self.sami_score = 0
        self.game_count = 0

        self.home_frame = tk.Frame(self.root)
        self.game_frame = tk.Frame(self.root)

        self.build_home_screen()

        self.protocol = []  # override in child class

    def build_home_screen(self):
        self.home_frame.pack(fill='both', expand=True)
        label = tk.Label(
            self.home_frame,
            text="Let's play Tic-Tac-Toe\nXOXO SAMI!",
            font=("Helvetica", 28, "bold"),
            fg="#f94f7d")
        label.pack(pady=50)

        start_btn = tk.Button(
            self.home_frame,
            text="Start Game",
            font=("Helvetica", 20),
            command=self.start_game)

        start_btn.pack(pady=20)

    def start_game(self):
        self.home_frame.pack_forget()
        self.build_game_screen()

    def build_game_screen(self):
        self.game_frame.pack(fill='both', expand=True)

        if hasattr(self, "board") and self.board is not None:
            self.board.destroy()

        board_class = self.get_board_class()
        self.board = board_class(master=self.game_frame, parent_app=self)
        self.board.pack(fill='both', expand=True, padx=20, pady=20)

    def get_board_class(self):
        return NormalGameBoard  # default, override in subclasses

    def start_next_game(self):
        if hasattr(self, "board") and self.board is not None:
            self.board.destroy()

        board_class = self.get_board_class()
        self.board = board_class(master=self.game_frame, parent_app=self)
        self.board.pack(fill='both', expand=True, padx=20, pady=20)

        self.board.display.config(
            text=f"Game {self.game_count + 1} | You: {self.player_score} - SAMI: {self.sami_score}"
        )

    def update_score(self, winner):
        if winner == "Player":
            self.player_score += 1
        elif winner == "SAMI":
            self.sami_score += 1

        print(f"[DEBUG] game_count={self.game_count}, protocol length={len(self.protocol)}")
        self.game_count += 1
        print(f"DEBUG after inc -> game_count={self.game_count}")

        if self.game_count == len(self.protocol):
            self.end_match()
        else:
            self.root.after(1000, self.start_next_game)

    def end_match(self):
        if self.player_score > self.sami_score:
            final_msg = f"You won the match! Final Score: You {self.player_score} - SAMI {self.sami_score}"
        elif self.sami_score > self.player_score:
            final_msg = f"SAMI won the match! Final Score: SAMI {self.sami_score} - You {self.player_score}"
        else:
            final_msg = f"It's a tie! Final Score: You {self.player_score} - SAMI {self.sami_score}"

        messagebox.showinfo("Match Over", final_msg)

        self.player_score = 0
        self.sami_score = 0
        self.game_count = 0
        self.game_frame.pack_forget()
        self.home_frame.pack()

    def run(self):
        self.root.mainloop()