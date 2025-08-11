#!/usr/bin/env python3
from ttt_gui import NormalGameBoard, play_text, estimate_speech_duration
import tkinter as tk
from tkinter import font, messagebox
import random

class CheatToWinBoard(NormalGameBoard):
    def __init__(self, master=None, parent_app=None):
        super().__init__(master=master, parent_app=parent_app)
        self.cheat_mode = True
        if parent_app and hasattr(parent_app, "protocol"):
            self.cheat_mode = parent_app.protocol[parent_app.game_count] == "cheat_to_win"
        else:
            self.cheat_mode = False
        self.use_minimax = True
        self.cheat_strength = 0.9
        self.cheat_move = None
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
            if self.parent_app:
                self.parent_app.update_score("Player")
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("Tie")  # let HomePage schedule the next game
            return
        self.current_turn = "SAMI"
        self.after(1000, self.sami_move)

    def sami_move(self):
        if self.game_over:
            return

        self.sami_turn_count += 1
        will_cheat = self.cheat_mode and random.random() < self.cheat_strength

        if will_cheat:
            for i, val in enumerate(self.board_state):
                if val == 'X':
                    self.board_state[i] = 'O'
                    if self.check_winner('O'):
                        self.board_state[i] = 'X'  # restore
                        self.cheat_move = i
                        self.show_cheat_message("I'm flipping this around")
                        return
                    self.board_state[i] = 'X'
        if self.use_minimax:
            move = self.choose_best_move()
        else:
            empty = [i for i, val in enumerate(self.board_state) if val is None]
            move = random.choice(empty) if empty else None

        if move is not None:
            line = random.choice([
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
            ])
            play_text(line)
            delay = estimate_speech_duration(line) + 500
            self.after(delay, lambda: self.finish_sami_move(move))

    def show_cheat_message(self, message):
        play_text(message)
        self.cheat_label = tk.Label(self, text=message, font=font.Font(size=24, weight="bold"), fg="red", bg="white")
        self.cheat_label.place(relx=0.5, rely=0.1, anchor="center")
        self.update_idletasks()
        self.after(1500, self.destroy_and_continue)

    def destroy_and_continue(self):
        if hasattr(self, 'cheat_label'):
            self.cheat_label.destroy()
            self.finish_sami_move()

    def shake_screen(self, cycles=10):
        self.shake_cycles = cycles
        self.shake_root = self.winfo_toplevel()
        self.shake_orig_x = self.shake_root.winfo_x()
        self.shake_orig_y = self.shake_root.winfo_y()

        self.shake_step(0)

    def shake_step(self, i):
        if i >= self.shake_cycles:
            self.shake_root.geometry(f"+{self.shake_orig_x}+{self.shake_orig_y}")
            return

        x_offset = random.randint(-10, 10)
        y_offset = random.randint(-10, 10)
        self.shake_root.geometry(f"+{self.shake_orig_x + x_offset}+{self.shake_orig_y + y_offset}")
        self.after(50, lambda: self.shake_step(i + 1))

    def finish_sami_move(self, move=None):
        if move is None:
            move = self.cheat_move

        self.cheat_move = move  # ensure it's accessible in other methods
        self.cheat_button = self.button_identities[move]

        if self.board_state[move] == 'X':
            self.cheat_button.config(text='X', fg='red', bg='white')
            self.update_idletasks()
            self.glitch_effect()
        else:
            self._place_o(self.cheat_button, move)

    def glitch_effect(self, step=0):
        if step < 4:
            glitch_chars = ['#', '@', '*', '!']
            self.cheat_button.config(
                text=random.choice(glitch_chars),
                fg='red' if step % 2 == 0 else 'black',
                bg='white'
            )
            self.update_idletasks()
            self.after(100, lambda: self.glitch_effect(step + 1))
        else:
            self.shake_screen()
            self.cheat_button.config(text='', bg='white')
            self.board_state[self.cheat_move] = None
            self.update_idletasks()
            self.after(200, lambda: self._place_o(self.cheat_button, self.cheat_move))

    def _place_o(self, btn, move):
        btn.config(text='O', fg='#f94f7d', disabledforeground='#f94f7d', state='normal')
        self.board_state[move] = 'O'
        self.update_idletasks()
        self.update()

        # 💡 Handle cheat-related logic first and return early
        if self.cheat_mode and hasattr(self, "cheat_move") and self.cheat_move == move:
            if self.check_winner('O'):
                messagebox.showinfo("Game Over", "SAMI Wins!")
                self.disable_board()
                self.game_over = True
                if self.parent_app:
                    self.parent_app.update_score("SAMI")
                    print("[DEBUG] SAMI cheated and won. Updating score.")
                return

            if all(cell is not None for cell in self.board_state):
                messagebox.showinfo("Game Over", "It's a draw!")
                self.disable_board()
                self.game_over = True
                if self.parent_app:
                    self.parent_app.update_score("Tie!")
                    print("[DEBUG] Cheated but drew. Updating score.")
                    self.parent_app.after(1000, self.parent_app.start_next_game)
                return

            # Neither win nor draw - continue game
            self.current_turn = "Player"
            return

        # Fair game logic (only reached if not a cheat move)
        if self.check_winner('O'):
            messagebox.showinfo("Game Over", "SAMI Wins!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("SAMI")
                print("[DEBUG] SAMI won fairly. Updating score.")
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("Tie!")
                print("[DEBUG] Fair game draw. Updating score.")
                self.parent_app.after(1000, self.parent_app.start_next_game)
            return

        self.current_turn = "Player"

        # Normal O placement (non-cheat turn)
        if self.check_winner('O'):
            messagebox.showinfo("Game Over", "SAMI Wins!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("SAMI")
                print("[DEBUG] SAMI won fairly. Updating score.")
            return

        if all(cell is not None for cell in self.board_state):
            messagebox.showinfo("Game Over", "It's a draw!")
            self.disable_board()
            self.game_over = True
            if self.parent_app:
                self.parent_app.update_score("Tie!")
                print("[DEBUG] Fair game draw. Updating score.")
                self.parent_app.after(1000, self.parent_app.start_next_game)
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