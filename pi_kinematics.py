import math

_A = 0.0113142
_B = 16.9572
_C = -663.986

class PrecalculatedMove:
    def __init__(self, x1, x2, y1, y2, initial_positions, tickTimeUs=2000):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.tickTimeUs = tickTimeUs
        self.initial_positions = initial_positions
        self.scalingFactor = self.getTimeScalingFactor(x1, x2, y1, y2)
        print(f'scalingFactor = {self.scalingFactor}')
        self.moves = bytearray()
        self.precalculate()

    def precalculate(self):
        print("Pre-calculating moves...")
        self.temporalPosition = 0
        simulated_positions = list(self.initial_positions)
        
        self.complete = False

        while not self.complete:
            steps = self.moveFunction(self.temporalPosition)
            if self.complete:
                break
            
            move_commands = []
            for i in range(4):
                gap = steps[i] - simulated_positions[i]
                command = 0
                if abs(gap) > 0.5:
                    command = int(math.copysign(1, gap))
                    simulated_positions[i] += command
                move_commands.append(command)
            
            encoded_byte = 0
            for i, command in enumerate(move_commands):
                value = 0
                if command == 1:
                    value = 1
                elif command == -1:
                    value = 2
                encoded_byte |= (value << (i * 2))
            
            self.moves.append(encoded_byte)
            self.temporalPosition += self.tickTimeUs
        
        print("Move pre-calculation complete.")

    def moveFunction(self, microSec):
        if microSec > self.scalingFactor * 1000000:
            self.complete = True
            return None
        return self.parametricLine(microSec, self.x1, self.x2, self.y1, self.y2, self.scalingFactor)
        
    def parametricLine(self, microSec, x1, x2, y1, y2, scalingFactor):
        timeSec = microSec / 1000000
        targetX = x1 + (x2 - x1) * (timeSec / scalingFactor)
        targetY = y1 + (y2 - y1) * (timeSec / scalingFactor)
        steps = self.getAllSteps(targetX, targetY)
        return steps

    def getAllSteps(self, xBoard, yBoard):
        def mapFromBoard(n):
            if n > 8 or n < 1:
                print(f'Requested board position {n} is out of range')
                return
            return 10 + (float(n - 1) * 28.71428)

        x = mapFromBoard(xBoard)
        y = mapFromBoard(yBoard)

        a1 = x + 17
        b1 = y + 17
        a2 = 238 - x
        b3 = 238 - y

        a1Sq = (x + 17) ** 2
        b1Sq = (y + 17) ** 2
        a2Sq = (238 - x) ** 2
        b3Sq = (238 - y) ** 2

        s1 = round(_A * (a1Sq + b1Sq) + (_B * math.sqrt(a1Sq + b1Sq)) + _C)
        s2 = round(_A * (a2Sq + b1Sq) + (_B * math.sqrt(a2Sq + b1Sq)) + _C)
        s3 = round(_A * (a1Sq + b3Sq) + (_B * math.sqrt(a1Sq + b3Sq)) + _C)
        s4 = round(_A * (a2Sq + b3Sq) + (_B * math.sqrt(a2Sq + b3Sq)) + _C)

        return s1, s2, s3, s4

    def getTimeScalingFactor(self, x1, x2, y1, y2):
        maxMotorSpeed = 500
        rotatedCoordinates = [
            [x1, x2, y1, y2],
            [y1, y2, 8 - x1, 8 - x2],
            [8 - x1, 8 - x2, 8 - y1, 8 - y2],
            [8 - y1, 8 - y2, x1, x2]
        ]
        maxSpeeds = []
        for rotation in rotatedCoordinates:
            d0 = self.ds_dt(0, rotation[0], rotation[1], rotation[2], rotation[3])
            d1 = self.ds_dt(-1, rotation[0], rotation[1], rotation[2], rotation[3])
            maxSpeeds.append(max(abs(d0), abs(d1)))
        unscaledMaxSpeed = max(maxSpeeds)
        timeScalingFactor = unscaledMaxSpeed / maxMotorSpeed
        return timeScalingFactor + 0.2
    
    def ds_dt(self, t, x1, x2, y1, y2):
        a = 0.0113142
        b = 16.9572
        dx = x2 - x1
        dy = y2 - y1
        const = 0.0597013054131951

        term_x = t * dx - x1 + const
        term_y = t * dy - y1 + const

        sqrtTerm = math.sqrt(term_x**2 + term_y**2)

        if sqrtTerm == 0:
            return 0
        
        numerator1 = 1649.0197518368 * a * (-dx * term_x - dy * term_y) * sqrtTerm
        numerator2 = 14.35714 * b * (2 * -dx * term_x + 2 * -dy * term_y)
        numerator = numerator1 + numerator2
        denominator = sqrtTerm

        return numerator / denominator
