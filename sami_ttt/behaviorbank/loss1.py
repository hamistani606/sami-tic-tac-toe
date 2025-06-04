self.misty.MoveHead(26,0,0, None, 1.5) # Head tilts down (in shame)
self.misty.MoveArms(90,90) # Arms go to sides
time.sleep(0.5)
self.misty.drive(-5,0,1000) # Slowly drives back
time.sleep(0.5)
# All below slowly bring robot back to same position
self.misty.MoveHead(0,0,0, None, 1.5)
self.misty.MoveArms(0,0)
time.sleep(0.75)
self.misty.drive(5,0,1000)
time.sleep(0.5)