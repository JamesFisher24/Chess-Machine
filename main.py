import utime
import motor
import kinematics
from debug import stepFromREPL

motors = [
    motor.Motor(pins=[20,17,18,19,16], invertDirection=True, currentPosition=6503),
    motor.Motor(pins=[11,13,12,14,15]),
    motor.Motor(pins=[10,7,9,21,8], invertDirection=False, currentPosition=3826),
    motor.Motor(pins=[2,3,4,6,5], invertDirection=False, currentPosition=5980)
] # motor pin mapping in order of port number

portMapping = [1, 3, 2, 4] # port number in order of board position



currentMove = kinematics.ParametricLineMove(8, 8, 8, 1, motors, tickTimeUs=2000)
print(currentMove.scalingFactor)
while not currentMove.complete:
    print(f'before update call: {utime.ticks_us()}')
    currentMove.updateMotors()
    print(f'after update call: {utime.ticks_us()}')
    utime.sleep_us(currentMove.tickTimeUs)
