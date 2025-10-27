from machine import Pin

class Motor:
    def __init__(self, pins, invertDirection=False, currentPosition=0):
        self.pins = []
        self.powerPattern = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [1, 0, 0, 1]
        ]
        for pin in pins:
            self.pins.append(Pin(pin, Pin.OUT))
        self.position = currentPosition
        self.direction = 1
        self.pins[4].low()
        self.invertDirection = invertDirection

    def step(self): # step the motor
        self.position += self.direction
        patternPosition = int((-1 * self.position + 5) % len(self.powerPattern))
        pattern = self.powerPattern[patternPosition]
        for i in range(len(pattern)):
            if pattern[i] == 0:
                self.pins[i].low()
            elif pattern[i] == 1:
                self.pins[i].high()
               
       
       
    def enable(self): # enable h-bridge driver to start sending power to motor. Motor should only be enabled while moving to prevent overheating
        self.pins[4].high()
       
    def disable(self): # disable h-bridge driver
        self.pins[4].low()
       
    def setDirection(self, direction): # set the direction of motion for subsequent steps
        self.direction = direction
        if self.invertDirection:
            self.direction = -1 * direction
