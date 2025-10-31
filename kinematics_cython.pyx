# cython: language_level=3
import math
import utime

cdef class Move:
    def __init__(self, motors, tickTimeUs=2000):
        self.times = []
        for motor in motors:
            motor.enable()
        self.motors = motors
        self.tickTimeUs = tickTimeUs
        self.temporalPosition = 0
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
            if gap != 0:
                if gap > 0:
                    motor.setDirection(1)
                else:
                    motor.setDirection(-1)
                self.times.append(f'before step: {utime.ticks_us()}')
                motor.step()
                self.times.append(f'after step: {utime.ticks_us()}')

    cpdef tuple getAllSteps(self, float xBoard, float yBoard):
        cdef float a = 0.0113142
        cdef float b = 16.9572
        cdef float c = -663.986

        cdef float x = 10.0 + (xBoard - 1.0) * 28.71428
        cdef float y = 10.0 + (yBoard - 1.0) * 28.71428

        cdef float a1 = x + 17.0
        cdef float b1 = y + 17.0
        cdef float a2 = 238.0 - x
        cdef float b3 = 238.0 - y

        cdef float a1_sq = a1 * a1
        cdef float b1_sq = b1 * b1
        cdef float a2_sq = a2 * a2
        cdef float b3_sq = b3 * b3

        cdef float sum_sq_1 = a1_sq + b1_sq
        cdef float sum_sq_2 = a2_sq + b1_sq
        cdef float sum_sq_3 = a1_sq + b3_sq
        cdef float sum_sq_4 = a2_sq + b3_sq

        s1 = int(a * sum_sq_1 + b * math.sqrt(sum_sq_1) + c)
        s2 = int(a * sum_sq_2 + b * math.sqrt(sum_sq_2) + c)
        s3 = int(a * sum_sq_3 + b * math.sqrt(sum_sq_3) + c)
        s4 = int(a * sum_sq_4 + b * math.sqrt(sum_sq_4) + c)

        return s1, s2, s3, s4


cdef class ParametricLineMove(Move):
    def __init__(self, float x1, float x2, float y1, float y2, motors, tickTimeUs=2000):
        super().__init__(motors, tickTimeUs=tickTimeUs)
        self.motors = motors
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.scalingFactor = self.getTimeScalingFactor(x1, x2, y1, y2)
        print(f'scalingFactor = {self.scalingFactor}')

    def moveFunction(self, int microSec):
        if self.temporalPosition > self.scalingFactor * 1000000:
            print(f"Move Complete at time {self.temporalPosition}")
            self.complete = True
            return
        return self.parametricLine(microSec, self.x1, self.x2, self.y1, self.y2, self.scalingFactor)

    cpdef tuple parametricLine(self, int microSec, float x1, float x2, float y1, float y2, float scalingFactor):
        cdef float timeSec = microSec / 1000000.0
        cdef float targetX = x1 + (x2 - x1) * (timeSec / scalingFactor)
        cdef float targetY = y1 + (y2 - y1) * (timeSec / scalingFactor)
        steps = self.getAllSteps(targetX, targetY)
        return steps

    def getTimeScalingFactor(self, float x1, float x2, float y1, float y2):
        cdef int maxMotorSpeed = 500

        rotatedCoordinates = [
            [x1, x2, y1, y2],
            [y1, y2, 8 - x1, 8 - x2],
            [8 - x1, 8 - x2, 8 - y1, 8 - y2],
            [8 - y1, 8 - y2, x1, x2]
        ]
        print(rotatedCoordinates)
        maxSpeeds = []
        for rotation in rotatedCoordinates:
            d0 = self.ds_dt(0, rotation[0], rotation[1], rotation[2], rotation[3])
            d1 = self.ds_dt(-1, rotation[0], rotation[1], rotation[2], rotation[3])
            print(d0, d1)
            maxSpeeds.append(max(abs(d0), abs(d1)))
        print(maxSpeeds)
        unscaledMaxSpeed = max(maxSpeeds)
        print(f'unscaledMaxSpeed={unscaledMaxSpeed}')
        timeScalingFactor = unscaledMaxSpeed / maxMotorSpeed
        return timeScalingFactor + 0.2

    cpdef float ds_dt(self, float t, float x1, float x2, float y1, float y2):
        cdef float a = 0.0113142
        cdef float b = 16.9572
        cdef float dx = x2 - x1
        cdef float dy = y2 - y1
        cdef float const = 0.0597013054131951

        cdef float term_x = t * dx - x1 + const
        cdef float term_y = t * dy - y1 + const

        cdef float sqrtTerm = math.sqrt(term_x*term_x + term_y*term_y)

        if sqrtTerm == 0:
            print("derivative error. Division by zero")
            return 0.0
        
        cdef float numerator1 = 1649.0197518368 * a * (-dx * term_x - dy * term_y) * sqrtTerm
        cdef float numerator2 = 14.35714 * b * (2 * -dx * term_x + 2 * -dy * term_y)
        cdef float numerator = numerator1 + numerator2
        cdef float denominator = sqrtTerm

        return numerator / denominator
