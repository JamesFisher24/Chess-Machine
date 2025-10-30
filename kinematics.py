import micropython
import math
import utime

class Move:
    def __init__(self, motors, tickTimeUs=2000):
        self.times = []
        for motor in motors:
            motor.enable()
        self.motors = motors
        self.tickTimeUs = tickTimeUs
        self.temporalPosition = 0 # t value for the move. Should be the same as the time since the begining of the move if the move was not paused
        self.complete = False
        print(self.temporalPosition)

    def updateMotors(self):
        self.temporalPosition += self.tickTimeUs
        self.times.append(f'before IK: {utime.ticks_us()}')
        steps = self.moveFunction(self.temporalPosition)
        self.times.append(f'after IK: {utime.ticks_us()}')
        if not steps:
            return
        i = 0
        for motor in self.motors:
            currentPosition = motor.position
            gap = steps[i] - currentPosition
            i += 1
            if gap != 0: #move motor if that would bring the position closer to the target
                if gap > 0:
                    motor.setDirection(1)
                else:
                    motor.setDirection(-1)
                self.times.append(f'before step: {utime.ticks_us()}')
                motor.step()
                self.times.append(f'after step: {utime.ticks_us()}')

    @micropython.native
    def getAllSteps(self, xBoard: float, yBoard: float) -> tuple: # return all lengths given the board coordinates
        # parabolic constants for string spool diameter compensation
        a = 0.0113142
        b = 16.9572
        c = -663.986

        # Convert board coordinates to millimeters
        x = 10.0 + (xBoard - 1.0) * 28.71428
        y = 10.0 + (yBoard - 1.0) * 28.71428

        a1 = x + 17.0
        b1 = y + 17.0
        a2 = 238.0 - x
        b3 = 238.0 - y

        a1_sq = a1 * a1
        b1_sq = b1 * b1
        a2_sq = a2 * a2
        b3_sq = b3 * b3

        sum_sq_1 = a1_sq + b1_sq
        sum_sq_2 = a2_sq + b1_sq
        sum_sq_3 = a1_sq + b3_sq
        sum_sq_4 = a2_sq + b3_sq

        # Using math.sqrt from the math module
        s1 = int(a * sum_sq_1 + b * math.sqrt(sum_sq_1) + c)
        s2 = int(a * sum_sq_2 + b * math.sqrt(sum_sq_2) + c)
        s3 = int(a * sum_sq_3 + b * math.sqrt(sum_sq_3) + c)
        s4 = int(a * sum_sq_4 + b * math.sqrt(sum_sq_4) + c)

        return s1, s2, s3, s4


class ParametricLineMove(Move):
    def __init__(self, x1, x2, y1, y2, motors, tickTimeUs=2000):
        super().__init__(motors, tickTimeUs=tickTimeUs)
        self.motors = motors
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.scalingFactor = self.getTimeScalingFactor(x1, x2, y1, y2)
        print(f'scalingFactor = {self.scalingFactor}')

    def moveFunction(self, microSec):
        if self.temporalPosition > self.scalingFactor * 1000000:
            print(f"Move Complete at time {self.temporalPosition}")
            self.complete = True
            return
        return self.parametricLine(microSec, self.x1, self.x2, self.y1, self.y2, self.scalingFactor)
        
    @micropython.native
    def parametricLine(self, microSec: int, x1: float, x2: float, y1: float, y2: float, scalingFactor: float) -> tuple: # Calculate where each motor should be at a specific micro second time during a move
        timeSec = microSec / 1000000.0
        targetX = x1 + (x2 - x1) * (timeSec / scalingFactor)
        targetY = y1 + (y2 - y1) * (timeSec / scalingFactor)
        steps = self.getAllSteps(targetX, targetY)
        return steps

    def getTimeScalingFactor(self, x1, x2, y1, y2): # Calculates the time scaling factor for the move

        # max motor speed in steps per second
        maxMotorSpeed = 500

        # rotate coordinates to each motor's refrence frame to be able to use the same derivative function
        rotatedCoordinates = [
            [x1, x2, y1, y2],
            [y1, y2, 8 - x1, 8 - x2],
            [8 - x1, 8 - x2, 8 - y1, 8 - y2],
            [8 - y1, 8 - y2, x1, x2]
        ]
        print(rotatedCoordinates)
        maxSpeeds = []
        for rotation in rotatedCoordinates: # loop through each refrence frame and record the highest speed for that motor
            d0 = self.ds_dt(0, rotation[0], rotation[1], rotation[2], rotation[3])
            d1 = self.ds_dt(-1, rotation[0], rotation[1], rotation[2], rotation[3])
            print(d0, d1)
            maxSpeeds.append(max(abs(d0), abs(d1)))
        print(maxSpeeds)
        unscaledMaxSpeed = max(maxSpeeds)
        print(f'unscaledMaxSpeed={unscaledMaxSpeed}')
        timeScalingFactor = unscaledMaxSpeed / maxMotorSpeed
        return timeScalingFactor + 0.2
    
    @micropython.native
    def ds_dt(self, t: float, x1: float, x2: float, y1: float, y2: float) -> float: # Calculate the speed of any motor at a given time position on its path
        a = 0.0113142
        b = 16.9572
        dx = x2 - x1
        dy = y2 - y1
        const = 0.0597013054131951

        term_x = t * dx - x1 + const
        term_y = t * dy - y1 + const

        sqrtTerm = math.sqrt(term_x*term_x + term_y*term_y)

        # Check for division by zero
        if sqrtTerm == 0:
            print("derivative error. Division by zero")
            return 0.0
        
        numerator1 = 1649.0197518368 * a * (-dx * term_x - dy * term_y) * sqrtTerm
        numerator2 = 14.35714 * b * (2 * -dx * term_x + 2 * -dy * term_y)
        numerator = numerator1 + numerator2
        denominator = sqrtTerm

        return numerator / denominator

'''
 ___________________
| Initial Positions |
 -------------------
(0, 3826, 3826, 5980)

 _________________
| Motor Positions |
 -----------------

3------4
|      |
|      |
1------2
'''