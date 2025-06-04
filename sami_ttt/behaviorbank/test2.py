self.misty.MoveHead(0, 30, 0)
self.misty.MoveArm("right", 70) # choosing Misty's arm and its position
self.misty.MoveArm("left", -50)
time.sleep(0.4)
self.misty.MoveArm("left", 0)
time.sleep(0.4)
self.misty.MoveArm("left", -50)
time.sleep(0.4)
self.misty.MoveArm("left", 0)
