#!/usr/bin/env python3

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

    def build_home_screen(self):
        self.home_frame.pack(fill='both', expand = True) 
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

    # def build_game_screen(self):
    #     self.game_frame.pack(fill='both', expand=True)
    #     self.score_label = tk.Label(
    #         self.game_frame, 
    #         # text=self.get_score_text(),
    #         font=("Helvetica", 18))
        
    #     self.score_label.pack(side="top", pady=10, anchor="n")
    #     self.board = NormalGameBoard()
    #     self.board.pack(fill='both', expand=True, padx=20, pady=20)

    def build_game_screen(self):
        self.game_frame.pack(fill='both', expand=True)

        # Remove any old label creation logic (you don't need score_label for just auto-restart)
        # If you want to show score, you can re-add it later

        if hasattr(self, "board") and self.board is not None:
            self.board.destroy()

        self.board = NormalGameBoard(master=self.game_frame, parent_app=self)
        self.board.pack(fill='both', expand=True, padx=20, pady=20)

    # def get_score_text(self):
    #     return f"Best of 3 - You: {self.player_score} | SAMI: {self.sami_score}"
    
    def update_score(self, winner):
        if winner == "Player":
            self.player_score += 1
        elif winner == "SAMI":
            self.sami_score += 1
        self.game_count += 1

        if self.player_score == 2 or self.sami_score == 2:
            self.end_match()
        else:
            self.board.destroy()
            self.board = NormalGameBoard(master=self.game_frame, parent_app=self)
            self.board.pack(fill='both', expand=True, padx=20, pady=20)
            # self.score_label.config(text=self.get_score_text())

    def end_match(self):
        if self.player_score > self.sami_score:
            final_msg = "You won the match!"
        elif self.sami_score > self.player_score:
            final_msg = "SAMI won the match!"
        else:
            final_msg = "It's a tie match!"

        messagebox.showinfo("Match Over", final_msg)

        # Reset for new match
        self.player_score = 0
        self.sami_score = 0
        self.game_count = 0
        self.game_frame.pack_forget()
        self.home_frame.pack()

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    app = HomePage()
    app.run()