misty.move_head(0, 30, 0)
misty.move_arm("left", 70) # choosing Misty's arm and its position
misty.move_arm("right", -50)
time.sleep(0.4)
misty.move_arm("right", 0)
time.sleep(0.4)
misty.move_arm("right", -50)
time.sleep(0.4)
misty.move_arm("right", 0)
