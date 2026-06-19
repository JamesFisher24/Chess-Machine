import kinematics
import utime

class Board:
    def __init__(self, x, y, motors):
        self.motors = motors
        self.x = x
        self.y = y
    
    def calculateMove(self, x, y):
        self.currentMove = kinematics.PrecalculatedMove(self.x, x, self.y, y, self.motors)
    
    def calculateMultiMove(self, waypoints):
        if len(waypoints) == 1:
            x, y = waypoints[0]
            self.currentMove = kinematics.PrecalculatedMove(self.x, x, self.y, y, self.motors)
        else:
            lines = []
            cur_x, cur_y = self.x, self.y
            for x, y in waypoints:
                lines.append((cur_x, cur_y, x, y))
                cur_x, cur_y = x, y
            self.currentMove = kinematics.MultiLineMove(lines, self.motors)

    
    def enable(self):
        for motor in self.motors:
            motor.enable()
        
    def disable(self):
        for motor in self.motors:
            motor.disable()
            
    def executeMove(self):
        self.enable()
        while not self.currentMove.complete:
            self.currentMove.updateMotors()
            utime.sleep_us(self.currentMove.tickTimeUs)
        self.disable()
        self.x = self.currentMove.x2
        self.y = self.currentMove.y2
    
