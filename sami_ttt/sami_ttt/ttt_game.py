#!/usr/bin/env python3

# ttt_game.py
#
# Kyle Vickstrom
#
# This is the game node SAMI's tic tac toe game.
# This publishes a GameState msg whenever something changes.
# It waits for user inputs using a service call, which will then update the GameState msg.
# It will trigger a relevent service / action call to move the robot / play audio as needed.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
import random

from sami_ttt_msgs.msg import GameLog, GameState
from sami_ttt_msgs.srv import NewGame, PlayerTurn

class TicTacGame(Node):
    def __init__(self):
        super().__init__('ttt_game')
        self.logging = True
        self.GameState = None
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.pubGame = self.create_publisher(GameState, 'game_state', 10)
        self.newGame_srv = self.create_service(NewGame, 'new_game', self.newGame)
        self.playerTurn_srv = self.create_service(PlayerTurn, 'player_turn', self.playerTurn)
        self.newGame_client = self.create_client(NewGame, 'new_game')
        self.difficulty = "medium"  # change this to easy, medium, or hard

        self.auto_start_game()

    def auto_start_game(self):
        req = NewGame.Request()
        while not self.newGame_client.wait_for_service(timeout_sec=1.0):
            self.log("Waiting for /new_game service...")
        future = self.newGame_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.log("New game initialized on startup.")
        else:
            self.log("Failed to initialize new game.")

    def newGame(self, request, response):
        """
        This clears the game state and resets.
        Service Server.
        """
        newGame = GameState()
        # randomly choose who goes first
        newGame.turn = random.randint(0, 1)
        # 0 for sami win, 1 for player win, 99 for currently playing
        newGame.win = 99
        response.turn = newGame.turn
        newGame.num_turns = 0
        
        # set score
        if self.GameState is None:
            newGame.score = [0, 0]
        else:
            newGame.score = self.GameState.score

        # empty board
        newGame.board = [-1, -1, -1,
                        -1, -1, -1,
                        -1, -1, -1]

        self.log(f"Score is {newGame.score[0]} robot, {newGame.score[1]} you.")
        if newGame.turn == 0:
            self.log("New game created. My turn to go first!")
        else:
            self.log("New game created. Your turn to go first!")

        self.GameState = newGame

        self.pubGame.publish(newGame)

        if newGame.turn == 0:
            self.make_misty_move()

        return response

    def newGame_request(self):
        """
        Service request to game node to create a new game.
        """
        if not self.newGame_client.wait_for_service(timeout_sec=1):
            self.log("Game node not active.")
            return
        request = NewGame.Request()
        self.NewGame_response = self.newGame_client.call_async(request)
        self.log("Requested new game")


    def playerTurn(self, request, response):
        """
        Service call for player to mark a new location
        Respondes true / false for valid move or not
        publishes updated GameState msg
        """
        # check for started game
        if self.GameState is None:
            self.log("No game started yet!")
            response.valid = False
            return response
        
        # check for correct id
        if request.player_id != self.GameState.turn:
            self.log("Wrong player!")
            response.valid = False
            return response

        if request.location >= 9:
            self.log("Invalid location!")
            response.valid = False
            return response

        # check for valid move
        if self.GameState.board[request.location] != -1:
            # space is not blank
            self.log("Space already occupied!")
            response.valid = False
            return response

        self.GameState.board[request.location] = self.GameState.turn
        self.GameState.num_turns += 1
        self.log(f"Placed on {request.location}.")


        # Check for win condition here
        if self.GameState.num_turns >= 5:
            self.checkForWin()

        if self.GameState.win != 99:
            # TODO: someone has won so SAMI text / interaction here
            # request new game upon win or have user indicate they want another game?
            #self.newGame_request()
            pass
        else:
            # advance turn
            if self.GameState.turn == 0:
                self.GameState.turn = 1
                # TODO: Trigger animation / speech here?
                self.log("Your turn!")
            else:
                # TODO: Also animation here
                self.GameState.turn = 0
                self.log("My turn!")
                self.make_misty_move()

        # publish updated game state
        self.pubGame.publish(self.GameState)

        response.valid = True

        return response

        # set move

    def make_misty_move(self):
        if self.GameState is None:
            return
        
        available = [i for i, x in enumerate(self.GameState.board) if x == -1]
        if not available:
            return
        
        move = None
        if self.difficulty == "easy":
            move = random.choice(available)
        elif self.difficulty == "medium":
            move = self.find_best_move() if random.random() < 0.7 else random.choice(available)
        elif self.difficulty == "hard":
            move = self.find_best_move() if random.random() < 0.99 else random.choice(available)
        
        self.GameState.board[move] = 0
        self.GameState.num_turns += 1
        self.log(f"Misty places at {move}")

        if self.GameState.num_turns >=5:
            self.checkForWin()

        if self.GameState.win == 99:
            self.GameState.turn = 1
            self.log("Your turn!")
        
        self.pubGame.publish(self.GameState)

    def find_best_move(self):
        for move in range(9):
            if self.GameState.board[move] == -1:
                self.GameState.board[move] = 0
                if self.check_win_simulation(0):
                    self.GameState.board[move] = -1
                    return move
                self.GameState.board[move] = -1

        for move in range(9):
            if self.GameState.board[move] == -1:
                self.GameState.board[move] = 1
                if self.check_win_simulation(1):
                    self.GameState.board[move] = -1
                    return move
                self.GameState.board[move] = -1

        for move in [4, 0, 2, 6, 8, 1, 3, 5, 7]:
            if self.GameState.board[move] == -1:
                return move
    
    def check_win_simulation(self, pid):
        b = self.GameState.board
        wins = [
            [b[0], b[1], b[2]], [b[3], b[4], b[5]], [b[6], b[7], b[8]],
            [b[0], b[3], b[6]], [b[1], b[4], b[7]], [b[2], b[5], b[8]],
            [b[0], b[4], b[8]], [b[2], b[4], b[6]]
        ]
        return any(all(cell == pid for cell in line) for line in wins)

    def checkForWin(self):
        """
        Checks if there a player has won and modifies the score accordingly
        """
        # get rows
        rows = [[self.GameState.board[0], self.GameState.board[1],self.GameState.board[2]],
                [self.GameState.board[3], self.GameState.board[4],self.GameState.board[5]],
                [self.GameState.board[6], self.GameState.board[7],self.GameState.board[8]]]
        # get cols
        cols = [[self.GameState.board[0], self.GameState.board[3],self.GameState.board[6]],
                [self.GameState.board[1], self.GameState.board[4],self.GameState.board[7]],
                [self.GameState.board[2], self.GameState.board[5],self.GameState.board[8]]]
        # get diags
        diags = [[self.GameState.board[0],self.GameState.board[4],self.GameState.board[8]],
                [self.GameState.board[2],self.GameState.board[4], self.GameState.board[6]]]

        # check for win by just totalling each of these lists^
        # total will be 0 if sami wins, 3 if player wins, if any are -1 then break

        # check rows
        #self.log(f"rows: {rows}")
        #self.log(f"cols: {cols}")
        #self.log(f"diags: {diags}")

        # check rows, cols, diags for user win.
        if self.checkAllCombos(rows):
            return
        elif self.checkAllCombos(cols):
            return
        elif self.checkAllCombos(diags):
            return

        self.log("No win yet")

    def checkAllCombos(self, combos):
        """
        loops over combos to find the total
        """
        oneFree = False
        for combo in combos:
            one = combo[0]
            two = combo[1]
            three = combo[2]
            result = set(one, two, three)
            #self.log(f"Row result: {result}")
            if len(result)==1:
                # combo is all the same player (or unused)
                if one != -1:
                    # one of the players won
                    self.GameState.win = self.GameState.turn
                    self.GameState.score[self.GameState.turn] += 1
                    self.log(f"Player id {self.GameState.turn} wins!")
                    return result
            else:
                if one == -1:
                    oneFree = True
                elif two == -1:
                    oneFree = True
                elif three == -1:
                    oneFree = True
                
        
        if not oneFree:
            # tie
            self.GameState.win = 2
            self.log(f"It's a tie!")


        return False


        


    def log(self, msg):
        """
        log to topic and to curses terminal window
        """
        if self.logging:
            newmsg = GameLog()
            newmsg.stamp = self.get_clock().now().to_msg()
            newmsg.node_name = self.get_name()
            newmsg.content = msg
            self.pubLog.publish(newmsg)
        self.get_logger().info(msg)

def createGame(args=None):
    rclpy.init(args=args)
    game = TicTacGame()
    rclpy.spin(game)
    rclpy.shutdown()
    
def main():
    createGame()

if __name__ == "__main__":
    createGame()