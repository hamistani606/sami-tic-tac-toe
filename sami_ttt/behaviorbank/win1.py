self.misty.MoveArms(-29, 0) # Waves hands in air, back and forth to up
time.sleep(0.125)
self.misty.drive(0, -20) # Turns left, then right slightly. Intended to mimic excitement
self.misty.MoveArms(0, -29)
time.sleep(0.125)
self.misty.drive(0, 20)
self.misty.MoveArms(-29, 0)
time.sleep(0.125)
self.misty.drive(0, -20)
self.misty.MoveArms(0, -29)
time.sleep(0.125)
self.misty.drive(0, 20)
self.misty.MoveArms(0, 0)