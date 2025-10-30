from machine import Pin

class Motor:
    def __init__(self, pins, invertDirection=False, currentPosition=0):
        self.pins = []
        self.powerPattern = (
            (1, 0, 1, 0),
            (0, 1, 1, 0),
            (0, 1, 0, 1),
            (1, 0, 0, 1)
        )
        for pin in pins:
            self.pins.append(Pin(pin, Pin.OUT))
        self.position = currentPosition
        self.direction = 1
        self.pins[4].low()
        self.invertDirection = invertDirection

    def step(self): # step the motor
        self.position += self.direction
        patternPosition = (-self.position + 5) % 4
        pattern = self.powerPattern[patternPosition]
        pins = self.pins
        if pattern[0]:
            pins[0].high()
        else:
            pins[0].low()
        if pattern[1]:
            pins[1].high()
        else:
            pins[1].low()
        if pattern[2]:
            pins[2].high()
        else:
            pins[2].low()
        if pattern[3]:
            pins[3].high()
        else:
            pins[3].low()
               
       
       
    def enable(self): # enable h-bridge driver to start sending power to motor. Motor should only be enabled while moving to prevent overheating
        self.pins[4].high()
        
       
    def disable(self): # disable h-bridge driver
        self.pins[4].low()
       
    def setDirection(self, direction): # set the direction of motion for subsequent steps
        self.direction = direction
        if self.invertDirection:
            self.direction = -1 * direction
