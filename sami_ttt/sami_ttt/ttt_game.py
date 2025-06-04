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
from sami_ttt_msgs.action import Speak, MistyMovement

class TicTacGame(Node):
    def __init__(self):
        super().__init__('ttt_game')
        self.logging = True
        self.GameState = None
        self.speaking = False
        self.moving = False
        self.started = False
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.pubGame = self.create_publisher(GameState, 'game_state', 10)
        self.newGame_srv = self.create_service(NewGame, 'new_game', self.newGame)
        self.playerTurn_srv = self.create_service(PlayerTurn, 'player_turn', self.playerTurn)
        self.newGame_client = self.create_client(NewGame, 'new_game')

        self.moveMisty_action = ActionClient(self, MistyMovement, 'mistymovement')

        # self.speakClient = ActionClient(self, Speak, 'speak')
        self.difficulty = "medium"  # change this to easy, medium, or hard

        self.auto_start_game()

    def moveMisty(self, behavior:str, words:str):
        """
        Action server request to move misty.
        who refers to the text misty will speak
        ex 0 for I win, 1 for you win
        """
        '''
        if self.moving:
            return
        else:
            self.moving = True
        '''
        goal = MistyMovement.Goal()
        goal.behavior = behavior
        goal.words = words
        self.moveMisty_action.wait_for_server()
        # spin until future complete whenever you call the move misty
        self.moveResult = self.moveMisty_action.send_goal_async(goal)
        #rclpy.spin_until_future_complete(self, self.moveResult)

        #self.moveResult = self.moveMisty_action.send_goal_async(goal, feedback_callback=self.move_fb)

        #self.moveResult.add_done_callback(self.moveResponse)

    def move_fb(self, fb_msg):
        """
        feedback callback for misty moving action
        """
        self.log(fb_msg.progress)

    def moveResponse(self, future):
        """
        when misty move action is accepted or rejected
        """
        self.move_handle = future.Result()
        if not self.move_handle.accepted:
            self.log("goal rejected")
            return

        self.move_result_handle = self.move_handle.get_result_async()
        self.move_result_handle.add_done_callback(self.process_move_result)
        
    def process_move_result(self, future):
        """
        when move action is finished
        """
        result = future.Result().completion
        self.moving = False
        self.log(result)



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
        if not self.started:
            # self.speak("Welcome to tic tac toe!")
            self.moveMisty('test1', "Welcome to tic tac toe!")
            self.started = True
        else:
            if newGame.turn == 0:
                # self.speak(f"Score is {self.GameState.score[0]} me, {self.GameState.score[1]} you.")\pass
                self.moveMisty('test2', f"Score is {self.GameState.score[0]} me, {self.GameState.score[1]} you.")
            else:
                # self.speak(f"Score is {self.GameState.score[0]} me, {self.GameState.score[1]} you. YOUR TURN!!")
                self.moveMisty('test2', f"Score is {self.GameState.score[0]} me, {self.GameState.score[1]} you. YOUR TURN!!")
                
        # 0 for sami win, 1 for player win, 99 for currently playing
        newGame.win = 99
        response.turn = newGame.turn
        newGame.num_turns = 0

        # apply difficulty from request
        if request.difficulty:
            self.difficulty = request.difficulty
            self.log(f"Difficulty set to {self.difficulty}")

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
                # self.speak("YOUR TURN!")
                self.log("Your turn!")
                self.moveMisty("playerturn", "Your turn!")
                #rclpy.spin_until_future_complete(self, self.moveResult)
            else:
                # TODO: Also animation here
                self.GameState.turn = 0
                #self.speak("MY TURN")
                # self.log("My turn!")
                self.moveMisty("thinking", "My turn! hmmm...")
                #rclpy.spin_until_future_complete(self, self.moveResult)
                self.make_misty_move()

        # publish updated game state
        self.pubGame.publish(self.GameState)

        response.valid = True

        return response

        # set move

    def make_misty_move(self):
        """
        Assigns a move for misty based off the difficulty settings which are also created and defined in this function
        """
        if self.GameState is None:
            return

        available = [i for i, x in enumerate(self.GameState.board) if x == -1]
        if not available:
            return

        move = None
        if self.difficulty == "easy":
            move = random.choice(available)
        elif self.difficulty == "medium":
            move = self.find_best_move() if random.random() < 0.5 else random.choice(available)
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
        """
        Algorithm for finding the best possible move in a tic-tac-toe game
        """
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
        if self.checkAllCombos():
            return

        if all(x != -1 for x in self.GameState.board):
            self.GameState.win = 2
            # self.speak("It's a tie!")
            self.moveMisty('win2', "It's a tie!")
            self.log("It's a tie!")

    def checkAllCombos(self):
        """
        loops over combos to find the total
        """
        b = self.GameState.board
        win_lines = [
            [b[0], b[1], b[2]], [b[3], b[4], b[5]], [b[6], b[7], b[8]],
            [b[0], b[3], b[6]], [b[1], b[4], b[7]], [b[2], b[5], b[8]], 
            [b[0], b[4], b[8]], [b[2], b[4], b[6]]                       
        ]

        for line in win_lines:
            if len(set(line)) == 1 and line[0] != -1:
                self.GameState.win = self.GameState.turn
                self.GameState.score[self.GameState.turn] += 1
                self.log(f"Player id {self.GameState.turn} wins!")
                if self.GameState.turn == 0:
                    # self.speak("I WIN!")
                    self.moveMisty('win1', "I WIN!")
                    pass
                else:
                    # self.speak("YOU WIN!")
                    self.moveMisty('loss1', "YOU WIN!")
                    pass
                return True
        return False

    def speak(self, msg: str):
        """
        Call the action server to speak with the given string
        """
        self.speakClient.wait_for_server()
        words = Speak.Goal()
        words.words = msg
        self.speakResponse = self.speakClient.send_goal_async(words)
        self.speakResponse.add_done_callback(self.speakresponse)

    def speakresponse(self, future):
        """
        called when action is accepted or rejected
        just registers callback to toggle off speaking bool
        """
        self.speakGoalHandle = future.result()
        if not self.speakGoalHandle.accepted:
            self.log("Speaking rejected")
        if not self.speaking:
            self.speakResultHandle = self.speakGoalHandle.get_result_async()
            self.speaking = True
            self.speakResultHandle.add_done_callback(self.doneSpeaking)

    def doneSpeaking(self, future):
        self.speaking = False

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