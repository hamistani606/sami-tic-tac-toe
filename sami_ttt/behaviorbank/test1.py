self.misty.MoveHead(0, 30, 0)
self.misty.MoveArm("left", 70) # choosing Misty's arm and its position
self.misty.MoveArm("right", -50)
time.sleep(0.4)
self.misty.MoveArm("right", 0)
time.sleep(0.4)
self.misty.MoveArm("right", -50)
time.sleep(0.4)
self.misty.MoveArm("right", 0)
