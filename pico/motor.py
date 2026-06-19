from machine import Pin, PWM
import micropython

class Motor:
    def __init__(self, pins, invertDirection=False, currentPosition=0, pwmDuty=65535, pwmFreq=1000):
        self.pins = []
        self.powerPattern = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [1, 0, 0, 1]
        ]
        for i in range(4):
            self.pins.append(Pin(pins[i], Pin.OUT))
            
        self.enable_pin = PWM(Pin(pins[4]))
        self.enable_pin.freq(pwmFreq)
        self.enable_pin.duty_u16(0)
        self.pins.append(self.enable_pin)
        
        self.pwm_duty = pwmDuty
        self.position = currentPosition
        self.direction = 1
        self.invertDirection = invertDirection

    @micropython.native
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
        self.enable_pin.duty_u16(self.pwm_duty)
        
       
    def disable(self): # disable h-bridge driver
        self.enable_pin.duty_u16(0)
        
    def setPower(self, duty_u16):
        """Set the PWM duty cycle (0-65535) for motor power."""
        self.pwm_duty = duty_u16
        
    def setPowerPercent(self, percent):
        """Set the PWM duty cycle as a percentage (0-100)."""
        self.pwm_duty = int((percent / 100.0) * 65535)
       
    def setDirection(self, direction): # set the direction of motion for subsequent steps
        self.direction = direction
        if self.invertDirection:
            self.direction = -1 * direction

    def __del__(self): # Destructor to disable the motor when the object is destroyed or program exits
        self.disable()

