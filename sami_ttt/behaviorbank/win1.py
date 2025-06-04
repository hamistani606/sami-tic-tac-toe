self.misty.MoveHead(-30, 0, 0, Duration = 200) # Tilts head back in excitement
time.sleep(0.225)
self.misty.MoveArms(-29, 0) # Waves hands in air, back and forth to up
time.sleep(0.125)
self.misty.DriveTime(0, -40, 300) # Turns left, then right slightly. Intended to mimic excitement
time.sleep(0.325)
self.misty.MoveArms(0, -29)
time.sleep(0.125)
self.misty.DriveTime(0, 40, 300)
time.sleep(0.325)
self.misty.MoveArms(-29, 0)
time.sleep(0.125)
self.misty.DriveTime(0, -40, 300)
time.sleep(0.325)
self.misty.MoveArms(0, -29)
time.sleep(0.125)
self.misty.DriveTime(0, 40, 300)
time.sleep(0.325)
self.misty.MoveArms(0, 0, Duration = 200) # Zero entries move arms and head back to central position
time.sleep(0.225)
self.misty.MoveHead(0, 0, 0, Duration = 200)
time.sleep(0.225)