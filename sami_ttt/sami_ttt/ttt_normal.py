#!/usr/bin/env python3
#!/usr/bin/env python3

from ttt_gui import NormalGameBoard
from home_base import HomePage

class NormalApp(HomePage):
    def __init__(self):
        super().__init__()
        self.protocol = ["fair"] * 7

    def get_board_class(self):
        return NormalGameBoard

if __name__ == "__main__":
    app = NormalApp()
    app.run()
