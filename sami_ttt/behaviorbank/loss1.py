misty.MoveHead(26,0,0, None, 1.5) # Head tilts down (in shame)
misty.MoveArms(90,90) # Arms go to sides
time.sleep(0.5)
misty.drive(-5,0,1000) # Slowly drives back
time.sleep(0.5)
# All below slowly bring robot back to same position
misty.MoveHead(0,0,0, None, 1.5)
misty.MoveArms(0,0)
time.sleep(0.75)
misty.drive(5,0,1000)
time.sleep(0.5)