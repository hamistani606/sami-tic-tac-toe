# ROB 421 Tic Tac Toe Game
   
<img src="sami_ttt/assets/nodes_sketch.jpg"  width="50%"/>  
  
### Dependencies:
put your dependecies here and any weird install steps  
    - curses (no install needed on linux)  
    - Misty-SDK  
    - gtts  
    - SpeechRecognition  
    - playsound  
        - may need `pip install playsound==1.2.2`
    - PyAudio  
        - `sudo apt-get install portaudio19.dev` before installing PyAudio 
  
### Game Tools:
`ros2 launch sami_ttt gameTools.py`  
This starts all the nodes except the board GUI and console UI.  
NOT IMPLEMENTED YET  
  
### Board GUI:
This is John's GUI using the LCD screen. Displays the board and relays the user's input with the service call to `ttt_game`.
  
### UI Controller:
`ros2 run sami_ttt ttt_console`  
This is a shell like UI built with curses.  
This can manually run the game and can test each node with appropriate service / action calls.  
  
The UI shows the running game log. For a node's log message to show up on the main UI:  
publish type `GameLog` msg to topic `game_log`  
This will show the timestamp, node of origin, and log message in the console window.  
I like to create a class method `log(self, msg:str)` where this string is published to `self.get_logger().info(msg)` and topic `game_log`.  

## Game Tools Nodes:  
* `ttt_game`: This is the game manager. Publishes the GameState and receives player input via service calls.  
                    Calls for sounds, robot animations via action calls when appropriate.  
* `cpu`: Not Implemented Yet. This node calls the player turn service on the `ttt_game` node and the relevent action calls for audio and robot motion.
                    The 'next move' algorithm can be written here or in an adjacent node that this node calls.  
* `misty_control`: This is the `move_robot` node. Should be able to be swapped with a `move_sami` equivalent.  
* `audio`: Not Implemented Yet. This node receives a string via action call and uses a TTS package to play the string on speakers.  