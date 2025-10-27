import math
import utime

def stepFromREPL(motor, timePerStepUs=2000):
    counter = 0
    while True:
        print(f'Number of steps traveled: {counter}\n')
        steps = 100
        input()
        if steps == 'reset':
            counter = 0
            steps = 0
        else:
            steps = int(steps)
            counter = counter + steps
           
        motor.setDirection(math.copysign(1, steps))
        motor.enable()
        for i in range(abs(steps)):
            motor.step()
            utime.sleep_us(timePerStepUs)
        motor.disable()