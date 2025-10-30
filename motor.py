from machine import Pin
from micropython import const, mem32

# SIO registers for direct GPIO manipulation on RP2040
SIO_BASE = const(0xd0000000)
GPIO_OUT_SET_REG = const(SIO_BASE + 0x14)
GPIO_OUT_CLR_REG = const(SIO_BASE + 0x18)

class Motor:
    def __init__(self, pins, invertDirection=False, currentPosition=0):
        self.pin_numbers = pins[:4]
        self.pins = [Pin(p, Pin.OUT) for p in pins]
        
        self.powerPattern = (
            (1, 0, 1, 0),
            (0, 1, 1, 0),
            (0, 1, 0, 1),
            (1, 0, 0, 1)
        )

        # Pre-calculate bitmasks for setting and clearing pins for each step pattern.
        # This allows setting all 4 motor pins in just two memory writes.
        self.set_masks = []
        self.clr_masks = []
        for pattern in self.powerPattern:
            set_mask = 0
            clr_mask = 0
            for i in range(4):
                pin_num = self.pin_numbers[i]
                if pattern[i]:
                    set_mask |= (1 << pin_num)
                else:
                    clr_mask |= (1 << pin_num)
            self.set_masks.append(set_mask)
            self.clr_masks.append(clr_mask)

        self.position = currentPosition
        self.direction = 1
        self.pins[4].low() # Disable H-bridge initially
        self.invertDirection = invertDirection

    @micropython.native
    def step(self): # step the motor
        self.position += self.direction
        patternPosition = (-self.position + 5) & 3 # Use bitwise AND for modulo 4
        
        # Set/clear all 4 pins in two atomic memory writes. This is much faster
        # than individual pin calls.
        mem32[GPIO_OUT_SET_REG] = self.set_masks[patternPosition]
        mem32[GPIO_OUT_CLR_REG] = self.clr_masks[patternPosition]
               
    def enable(self): # enable h-bridge driver to start sending power to motor. Motor should only be enabled while moving to prevent overheating
        self.pins[4].high()
        
    def disable(self): # disable h-bridge driver
        self.pins[4].low()
       
    def setDirection(self, direction): # set the direction of motion for subsequent steps
        self.direction = direction
        if self.invertDirection:
            self.direction = -1 * direction
