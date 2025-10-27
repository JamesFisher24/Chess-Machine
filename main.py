import utime
import motor
import kinematics
from debug import stepFromREPL

motors = [
    motor.Motor(pins=[20,17,18,19,16], invertDirection=False, currentPosition=6503),
    motor.Motor(pins=[10,7,9,21,8], invertDirection=False, currentPosition=3864),
    motor.Motor(pins=[11,13,12,14,15]),
    motor.Motor(pins=[2,3,4,6,5], invertDirection=True)
]

currentMove = kinematics.ParametricLineMove(1, 8, 1, 8, motors, tickTimeUs=2000)

while not currentMove.complete:
    currentMove.updateMotors()
    utime.sleep_us(currentMove.tickTimeUs)
    print(currentMove.complete)
    input()

# stepFromREPL(motors[3])
