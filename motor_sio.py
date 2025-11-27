from machine import Pin, mem32
import micropython

_POWER_PATTERN = const((
    (1, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 1),
    (1, 0, 0, 1),
))

# SIO registers base address
SIO_BASE = const(0xd0000000)
GPIO_OUT_SET = const(SIO_BASE + 0x14)
GPIO_OUT_CLR = const(SIO_BASE + 0x18)

class Motor:
    def __init__(self, pins, invertDirection=False, currentPosition=0):
        self.pins = []
        for pin in pins:
            self.pins.append(Pin(pin, Pin.OUT))
        self.position = currentPosition
        self.direction = 1
        self.pins[4].low()
        self.invertDirection = invertDirection

        self.pin_mask = 0
        for pin_num in pins[:4]:
            self.pin_mask |= 1 << pin_num

        self.pin_ids = pins[:4]
        self.enable_pin_mask = 1 << pins[4]

        self.step_masks = []
        for pattern in _POWER_PATTERN:
            set_mask = 0
            for i, val in enumerate(pattern):
                if val == 1:
                    set_mask |= 1 << pins[i] # Use original pin numbers
            self.step_masks.append(set_mask)

    @micropython.native
    def step(self):
        position = int(self.position)
        direction = int(self.direction)
        pin_mask = int(self.pin_mask)

        position += direction
        self.position = position

        patternPosition = int((-1 * position + 5) % 4)
        set_mask = self.step_masks[patternPosition]

        mem32[GPIO_OUT_SET] = set_mask
        mem32[GPIO_OUT_CLR] = pin_mask & ~set_mask

    def enable(self): # enable h-bridge driver
        mem32[GPIO_OUT_SET] = self.enable_pin_mask

    def disable(self): # disable h-bridge driver
        mem32[GPIO_OUT_CLR] = self.enable_pin_mask

    def setDirection(self, direction: int):
        self.direction = direction
        if self.invertDirection:
            self.direction = -1 * direction