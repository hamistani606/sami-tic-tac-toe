#!/usr/bin/env python3

from ttt_gui import NormalGameBoard
from cheat_to_win_board import CheatToWinBoard
from home_base import HomePage

class CheatToWinApp(HomePage):
    def __init__(self):
        super().__init__()
        self.protocol = [
            "fair", "fair", "fair",
            "cheat_to_win",
            "fair", "fair", "fair"
        ]

    def get_board_class(self):
        condition = self.protocol[self.game_count]
        if condition == "cheat_to_win":
            return CheatToWinBoard
        return NormalGameBoard

if __name__ == '__main__':
    app = CheatToWinApp()
    app.run()
