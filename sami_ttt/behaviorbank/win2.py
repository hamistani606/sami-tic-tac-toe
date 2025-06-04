# Movement meant to mimic hand shaking
self.misty.MoveHead(0, 0, 0) # Centers head
self.misty.DriveTime(10, 0, 500, 0) # Slowly moves forward
time.sleep(0.25)
# Shakes arm up and down to mimic hadn
self.misty.MoveArm('right', 0, 40)
self.misty.MoveArm('right', 30, 50)
self.misty.MoveArm('right', 0, 50)
self.misty.MoveArm('right', 30, 50)
self.misty.MoveArm('right', 0, 50)