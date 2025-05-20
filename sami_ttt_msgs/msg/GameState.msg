# turn = 0 for computer's turn, 1 for player's turn
int64 turn
# counts number of turns in game
int64 num_turns
# idx 0 is num wins of computer, idx 1 is num wins of player
int64[2] score
# 9 positions on board, idx 0 is top left, 2 is top right...
int64[9] board