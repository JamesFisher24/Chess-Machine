"""
Kinematics Engine for a 4-Axis Cable-Driven Robot (Chess Robot)
This file handles the complex mathematics required to move the electromagnet in straight lines
across an 8x8 chessboard using 4 stepper motors located at the corners.
"""
import math
import utime
import micropython
import gc

# --- Spool Winding Constants ---
# As string wraps around a spool, the effective radius increases.
# This means a stepper motor pulls more string per step when there is already a lot of string on the spool.
# These constants (_A, _B, _C) are the coefficients of a quadratic polynomial aproximation function used to convert 
# physical string length to actual stepper motor steps.
# Equation: Steps = (_A * length^2) + (_B * length) + _C
_A = 0.0113142
_B = 16.9572
_C = -663.986

class Move:
    """
    Base class for executing any type of movement.
    Handles the common logic of tracking time, maintaining target positions, 
    and sending step pulses to the motors.
    """
    def __init__(self, motors, tickTimeUs=2000):
        # tickTimeUs defines the time resolution of the movement loop (default: 2ms)
        self.tickTimeUs = tickTimeUs
        self.temporalPosition = 0  # Tracks elapsed time in microseconds during the move
        self.complete = False      # Flag indicating if the move has finished
        print(self.temporalPosition)

        # To avoid slow dynamic memory allocation during real-time motor control,
        # we pre-allocate all variables used in the heavy math function (getAllSteps).
        self.x = 0.0
        self.y = 0.0
        self.a1 = 0.0
        self.b1 = 0.0
        self.a2 = 0.0
        self.b3 = 0.0
        self.a1Sq = 0.0
        self.b1Sq = 0.0
        self.a2Sq = 0.0
        self.b3Sq = 0.0

    def updateMotors(self):
        """
        Called continuously in a loop to advance the movement over time.
        """
        # Disable Garbage Collection during calculation to prevent unexpected 
        # pauses which would cause stepper motor stuttering.
        gc.disable()
        # Advance the virtual time clock
        self.temporalPosition += self.tickTimeUs
        # Ask the specific move implementation (subclass) where the motors should be at this current time
        steps = self.moveFunction(self.temporalPosition)
        # Re-enable Garbage Collection
        gc.enable()

        if not steps:
            return
            
        # Iterate through all 4 motors to check if they need to step to catch up to the target
        i = 0
        for motor in self.motors:
            currentPosition = motor.position
            gap = steps[i] - currentPosition
            i += 1
            # If the calculated target position is at least half a step away from our current position, take a step.
            if abs(gap) > 0.5:
                # Set direction (1 or -1) based on the sign of the gap
                motor.setDirection(int(math.copysign(1, gap)))
                motor.step()

    @micropython.native # Use MicroPython's native code emitter for faster execution
    def getAllSteps(self, xBoard, yBoard):
        """
        Converts 2D board coordinates (1-8 like a chessboard) into the absolute number 
        of stepper motor steps required for all 4 motors to reach that point.
        """
        def mapFromBoard(n):
            # Validates and maps a chessboard coordinate (1-8) to physical units (e.g. millimeters)
            if n > 8 or n < 1:
                print(f'Requested board position {n} is out of range')
                return
            # Example conversion: Each square seems to be ~28.71 units wide, with a 10 unit offset
            return 10 + (float(n - 1) * 28.71428)
        print(mapFromBoard(1))
        self.x = mapFromBoard(xBoard)
        self.y = mapFromBoard(yBoard)

        # The robot has a motor at each corner. We calculate the horizontal (a) and vertical (b) 
        # distance from the end effector to each motor.
        # Motor 1 is at (-17, -17). Motor 2 is at (238+17, -17).
        # The total board space seems to be around 238 units across.
        self.a1 = self.x + 17
        self.b1 = self.y + 17
        self.a2 = 238 - self.x
        self.b3 = 238 - self.y

        # Square the distances for Pythagorean theorem (a^2 + b^2 = c^2)
        self.a1Sq = (self.x + 17) ** 2
        self.b1Sq = (self.y + 17) ** 2
        self.a2Sq = (238 - self.x) ** 2
        self.b3Sq = (238 - self.y) ** 2

        # Calculate the physical string length to each corner (c = sqrt(a^2 + b^2))
        # Then, apply the quadratic polynomial to convert string length into motor steps.
        # Motor 1 (Bottom Left)
        s1 = round(_A * (self.a1Sq + self.b1Sq) + (_B * math.sqrt(self.a1Sq + self.b1Sq)) + _C)
        # Motor 2 (Bottom Right)
        s2 = round(_A * (self.a2Sq + self.b1Sq) + (_B * math.sqrt(self.a2Sq + self.b1Sq)) + _C)
        # Motor 3 (Top Left)
        s3 = round(_A * (self.a1Sq + self.b3Sq) + (_B * math.sqrt(self.a1Sq + self.b3Sq)) + _C)
        # Motor 4 (Top Right)
        s4 = round(_A * (self.a2Sq + self.b3Sq) + (_B * math.sqrt(self.a2Sq + self.b3Sq)) + _C)

        return s1, s2, s3, s4


class ParametricLineMove(Move):
    """
    Subclass that represents a straight line movement between two points on the board.
    Moving in a straight line in a Cartesian plane requires complex, non-linear coordinate 
    movements from all 4 string lengths.
    """
    def __init__(self, x1, x2, y1, y2, motors, tickTimeUs=2000):
        super().__init__(motors, tickTimeUs=tickTimeUs)
        self.motors = motors
        # Start and end coordinates in board units (1-8)
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        
        # Calculate how long the move needs to take so that the fastest moving motor 
        # does not exceed its maximum speed limitation.
        self.scalingFactor = self.getTimeScalingFactor(x1, x2, y1, y2)
        print(f'scalingFactor = {self.scalingFactor}')

    def moveFunction(self, microSec):
        """
        Implementation of the required moveFunction. 
        Given an elapsed time, where should the end effector be?
        """
        # If we have passed the calculated end time for this move, mark it complete.
        if self.temporalPosition > self.scalingFactor * 1000000:
            print(f"Move Complete at time {self.temporalPosition}")
            self.complete = True
            return
        # Otherwise, calculate the coordinates on the line based on the time
        return self.parametricLine(microSec, self.x1, self.x2, self.y1, self.y2, self.scalingFactor)
        
    def parametricLine(self, microSec, x1, x2, y1, y2, scalingFactor): 
        """
        Interpolates the X and Y coordinates along a straight line based on the elapsed time.
        """
        timeSec = microSec / 1000000
        # Linear interpolation (Lerp): start + (difference) * progress
        targetX = x1 + (x2 - x1) * (timeSec / scalingFactor)
        targetY = y1 + (y2 - y1) * (timeSec / scalingFactor)
        
        # Calculate where the 4 motors need to be to reach this specific X, Y point
        steps = self.getAllSteps(targetX, targetY)
        return steps

    def getTimeScalingFactor(self, x1, x2, y1, y2): 
        """
        Analyzes the planned movement to find the theoretical maximum speed any motor will experience.
        Returns a time duration multiplier to guarantee no motor runs too fast.
        """
        maxMotorSpeed = 500 # Maximum allowable steps per second for the steppers

        # To simplify calculus, we rotate the move vector into the reference frame of each 
        # of the 4 motors. This allows us to use a single derivative function for all 4 corners.
        rotatedCoordinates = [
            [x1, x2, y1, y2],                 # Motor 1 Frame
            [y1, y2, 8 - x1, 8 - x2],         # Motor 2 Frame
            [8 - x1, 8 - x2, 8 - y1, 8 - y2], # Motor 4 Frame
            [8 - y1, 8 - y2, x1, x2]          # Motor 3 Frame
        ]
        print(rotatedCoordinates)
        
        maxSpeeds = []
        # Calculate the maximum instantaneous speed for each motor during the move
        for rotation in rotatedCoordinates: 
            # Check the speed at the very beginning of the move (t=0)
            d0 = self.ds_dt(0, rotation[0], rotation[1], rotation[2], rotation[3])
            # Check the speed at the very end of the move (t=-1 could represent completion, though the math here assumes a specific normalization)
            d1 = self.ds_dt(-1, rotation[0], rotation[1], rotation[2], rotation[3])
            print(d0, d1)
            # For a linear move the maximum speed will occur at the start or the end of the move.
            maxSpeeds.append(max(abs(d0), abs(d1)))
            
        print(maxSpeeds)
        # Find the absolute fastest moving motor
        unscaledMaxSpeed = max(maxSpeeds)
        print(f'unscaledMaxSpeed={unscaledMaxSpeed}')
        
        # How much do we need to stretch time to keep unscaledMaxSpeed under maxMotorSpeed?
        timeScalingFactor = unscaledMaxSpeed / maxMotorSpeed
        return timeScalingFactor + 0.2 # Add a 0.2 second buffer to ensure smooth acceleration profiles
    
    def ds_dt(self, t, x1, x2, y1, y2): 
        """
        The analytical derivative of the string length equation with respect to time.
        This provides the instantaneous velocity (steps per second) of a motor at any time 't'.
        Instead of simulating the whole move, calculus tells us exactly how fast a motor is pulling.
        """
        print(x1, y1, x2, y2)
        a = 0.0113142
        b = 16.9572
        dx = x2 - x1
        dy = y2 - y1
        const = 0.0597013054131951 # Mathematical constant derived during the derivative formulation

        # Calculate intermediate terms representing position at time 't'
        term_x = t * dx - x1 + const
        term_y = t * dy - y1 + const

        # Pythagorean distance calculation
        sqrtTerm = math.sqrt(term_x**2 + term_y**2)

        if sqrtTerm == 0:
            print("derivative error. Division by zero")
            return 0
        
        # The complex chain-rule derivative of the Steps polynomial function
        numerator1 = 1649.0197518368 * a * (-dx * term_x - dy * term_y) * sqrtTerm
        numerator2 = 14.35714 * b * (2 * -dx * term_x + 2 * -dy * term_y)
        print(numerator1, numerator2)
        numerator = numerator1 + numerator2
        denominator = sqrtTerm

        return numerator / denominator

'''
Motor Layout on the Board:
3------4
|      |
|      |
1------2
'''

class PrecalculatedMove(ParametricLineMove):
    """
    Because the square roots and math in ParametricLineMove are heavy for a microcontroller,
    this class pre-computes an entire movement and compresses it into a lightweight bytearray.
    This ensures the robot can move smoothly without processor stutters.
    """
    def __init__(self, x1, x2, y1, y2, motors, tickTimeUs=2000):
        super().__init__(x1, x2, y1, y2, motors, tickTimeUs)
        self.moves = bytearray() # Stores the pre-calculated step sequences
        self.move_index = 0      # Tracks which byte we are currently executing
        self.precalculate()

    @micropython.native
    def precalculate(self):
        """
        Runs the entire move in a fast-forward simulation.
        Instead of moving motors, it records the required steps into a memory buffer.
        """
        print("Pre-calculating moves...")
        
        # --- OPTIMIZATIONS FOR MICROPYTHON SPEED ---
        # 1. Pre-calculate the starting and ending board coordinates ONCE
        start_x = 10 + (float(self.x1 - 1) * 28.71428)
        start_y = 10 + (float(self.y1 - 1) * 28.71428)
        end_x = 10 + (float(self.x2 - 1) * 28.71428)
        end_y = 10 + (float(self.y2 - 1) * 28.71428)
        
        dx = end_x - start_x
        dy = end_y - start_y
        
        total_time_us = self.scalingFactor * 1000000
        tick_us = self.tickTimeUs
        
        # Pre-allocate the bytearray to prevent memory fragmentation!
        num_ticks = int(total_time_us / tick_us) + 1
        self.moves = bytearray(num_ticks)
        
        # 2. Localize variables for much faster lookup in the while loop
        A, B, C = _A, _B, _C
        sqrt = math.sqrt
        round_func = round
        
        # Create a virtual snapshot of where the motors are currently located
        simulated_pos_1 = self.motors[0].position
        simulated_pos_2 = self.motors[1].position
        simulated_pos_3 = self.motors[2].position
        simulated_pos_4 = self.motors[3].position
        
        t = 0
        idx = 0
        while t <= total_time_us:
            # Inline parametricLine mapping (avoids function calls)
            progress = t / total_time_us
            target_x = start_x + dx * progress
            target_y = start_y + dy * progress
            
            # Inline getAllSteps (avoids function calls and redundant mapFromBoard calculations)
            a1 = target_x + 17
            b1 = target_y + 17
            a2 = 238 - target_x
            b3 = 238 - target_y
            
            a1Sq = a1 * a1
            b1Sq = b1 * b1
            a2Sq = a2 * a2
            b3Sq = b3 * b3
            
            d1Sq = a1Sq + b1Sq
            d2Sq = a2Sq + b1Sq
            d3Sq = a1Sq + b3Sq
            d4Sq = a2Sq + b3Sq
            
            # math.sqrt and round are fast when localized
            s1 = round_func(A * d1Sq + B * sqrt(d1Sq) + C)
            s2 = round_func(A * d2Sq + B * sqrt(d2Sq) + C)
            s3 = round_func(A * d3Sq + B * sqrt(d3Sq) + C)
            s4 = round_func(A * d4Sq + B * sqrt(d4Sq) + C)
            
            # Inline motor logic without creating Lists/Arrays
            gap1 = s1 - simulated_pos_1
            gap2 = s2 - simulated_pos_2
            gap3 = s3 - simulated_pos_3
            gap4 = s4 - simulated_pos_4
            
            c1 = c2 = c3 = c4 = 0
            
            if gap1 > 0.5: c1 = 1; simulated_pos_1 += 1
            elif gap1 < -0.5: c1 = 2; simulated_pos_1 -= 1
                
            if gap2 > 0.5: c2 = 1; simulated_pos_2 += 1
            elif gap2 < -0.5: c2 = 2; simulated_pos_2 -= 1
                
            if gap3 > 0.5: c3 = 1; simulated_pos_3 += 1
            elif gap3 < -0.5: c3 = 2; simulated_pos_3 -= 1
                
            if gap4 > 0.5: c4 = 1; simulated_pos_4 += 1
            elif gap4 < -0.5: c4 = 2; simulated_pos_4 -= 1
            
            # Bitwise pack the 4 commands into one byte directly
            encoded_byte = c1 | (c2 << 2) | (c3 << 4) | (c4 << 6)
            
            if idx < num_ticks:
                self.moves[idx] = encoded_byte
            else:
                self.moves.append(encoded_byte)
            
            t += tick_us
            idx += 1
            
            # Periodically collect garbage from temporary float objects created in the loop
            if idx % 100 == 0:
                gc.collect()
            
        self.temporalPosition = 0
        self.complete = False
        print("Move pre-calculation complete.")

    @micropython.native
    def updateMotors(self):
        """
        The fast, real-time playback function.
        It simply reads bytes from the array and applies the step signals, skipping all heavy math.
        """
        if self.move_index >= len(self.moves):
            self.complete = True
            return

        # Fetch the instruction byte for this specific tick
        encoded_byte = self.moves[self.move_index]
        
        # Decode the 4 motor instructions
        for i in range(4):
            # Extract the 2 bits for motor i by shifting and applying a bitmask (0b11)
            value = (encoded_byte >> (i * 2)) & 0b11
            command = 0
            if value == 1:
                command = 1
            elif value == 2:
                command = -1
            
            # Execute the step physically
            if command != 0:
                self.motors[i].setDirection(command)
                self.motors[i].step()
        
        # Advance the playback head
        self.move_index += 1


class MultiLineMove(PrecalculatedMove):
    """
    Executes movement along multiple sequential lines.
    To avoid memory exhaustion on the Raspberry Pi Pico, the moves are split
    into segments if the precalculated bytearray exceeds the specified memory limit.
    Segments are precalculated sequentially (the next segment is precalculated
    on-the-fly when the current one finishes).
    """
    def __init__(self, lines, motors, tickTimeUs=2000, max_mem_bytes=10000):
        # Initialize grandparent class Move directly to bypass single line initialization
        Move.__init__(self, motors, tickTimeUs=tickTimeUs)
        self.motors = motors
        self.lines = lines
        self.max_mem_bytes = max_mem_bytes
        
        if not lines:
            raise ValueError("Lines array cannot be empty")
            
        # Start and end coordinates of the overall multi-line movement
        self.x1 = lines[0][0]
        self.y1 = lines[0][1]
        self.x2 = lines[-1][2]
        self.y2 = lines[-1][3]
        
        self.segments = []
        self.moves = bytearray()
        self.move_index = 0
        self.current_segment_index = 0
        
        # Track simulated positions internally to avoid reading inverted physical positions
        self.simulated_positions = [
            motors[0].position,
            motors[1].position,
            motors[2].position,
            motors[3].position
        ]
        
        # Calculate scaling factor as sum of all lines' scaling factors
        self.scalingFactor = 0.0
        
        self.plan_segments()
        
        # Precalculate the first segment
        if self.segments:
            self.precalculate_segment(0)
        else:
            self.complete = True

    def get_line_size(self, line):
        x1, y1, x2, y2 = line
        scaling_factor = self.getTimeScalingFactor(x1, x2, y1, y2)
        total_time_us = scaling_factor * 1000000
        num_ticks = int(total_time_us / self.tickTimeUs) + 1
        return num_ticks, scaling_factor

    def plan_segments(self):
        current_segment = []
        current_segment_size = 0
        
        for line in self.lines:
            line_size, scaling_factor = self.get_line_size(line)
            self.scalingFactor += scaling_factor
            
            # If adding this line exceeds the memory limit, and the current segment has lines,
            # we finalize the current segment and start a new one.
            if current_segment_size + line_size > self.max_mem_bytes and current_segment:
                self.segments.append(current_segment)
                current_segment = [line]
                current_segment_size = line_size
            else:
                current_segment.append(line)
                current_segment_size += line_size
                
        if current_segment:
            self.segments.append(current_segment)

    @micropython.native
    def precalculate_segment(self, segment_index):
        """
        Precalculates all lines in the specified segment into self.moves bytearray.
        """
        print(f"Pre-calculating segment {segment_index + 1}/{len(self.segments)}...")
        lines = self.segments[segment_index]
        
        # Compute total ticks and details for all lines in this segment
        line_info = []
        total_ticks = 0
        for line in lines:
            x1, y1, x2, y2 = line
            scaling_factor = self.getTimeScalingFactor(x1, x2, y1, y2)
            total_time_us = scaling_factor * 1000000
            num_ticks = int(total_time_us / self.tickTimeUs) + 1
            line_info.append((line, scaling_factor, total_time_us, num_ticks))
            total_ticks += num_ticks
            
        # Pre-allocate bytearray
        self.moves = bytearray(total_ticks)
        
        # Localize optimizations for speed
        A, B, C = _A, _B, _C
        sqrt = math.sqrt
        round_func = round
        tick_us = self.tickTimeUs
        
        # Start simulation from the stored virtual/simulated positions
        simulated_pos_1 = self.simulated_positions[0]
        simulated_pos_2 = self.simulated_positions[1]
        simulated_pos_3 = self.simulated_positions[2]
        simulated_pos_4 = self.simulated_positions[3]
        
        idx = 0
        for line, scaling_factor, total_time_us, num_ticks in line_info:
            x1, y1, x2, y2 = line
            start_x = 10 + (float(x1 - 1) * 28.71428)
            start_y = 10 + (float(y1 - 1) * 28.71428)
            end_x = 10 + (float(x2 - 1) * 28.71428)
            end_y = 10 + (float(y2 - 1) * 28.71428)
            
            dx = end_x - start_x
            dy = end_y - start_y
            
            t = 0
            while t <= total_time_us:
                progress = t / total_time_us
                target_x = start_x + dx * progress
                target_y = start_y + dy * progress
                
                a1 = target_x + 17
                b1 = target_y + 17
                a2 = 238 - target_x
                b3 = 238 - target_y
                
                a1Sq = a1 * a1
                b1Sq = b1 * b1
                a2Sq = a2 * a2
                b3Sq = b3 * b3
                
                d1Sq = a1Sq + b1Sq
                d2Sq = a2Sq + b1Sq
                d3Sq = a1Sq + b3Sq
                d4Sq = a2Sq + b3Sq
                
                s1 = round_func(A * d1Sq + B * sqrt(d1Sq) + C)
                s2 = round_func(A * d2Sq + B * sqrt(d2Sq) + C)
                s3 = round_func(A * d3Sq + B * sqrt(d3Sq) + C)
                s4 = round_func(A * d4Sq + B * sqrt(d4Sq) + C)
                
                gap1 = s1 - simulated_pos_1
                gap2 = s2 - simulated_pos_2
                gap3 = s3 - simulated_pos_3
                gap4 = s4 - simulated_pos_4
                
                c1 = c2 = c3 = c4 = 0
                
                if gap1 > 0.5: c1 = 1; simulated_pos_1 += 1
                elif gap1 < -0.5: c1 = 2; simulated_pos_1 -= 1
                    
                if gap2 > 0.5: c2 = 1; simulated_pos_2 += 1
                elif gap2 < -0.5: c2 = 2; simulated_pos_2 -= 1
                    
                if gap3 > 0.5: c3 = 1; simulated_pos_3 += 1
                elif gap3 < -0.5: c3 = 2; simulated_pos_3 -= 1
                    
                if gap4 > 0.5: c4 = 1; simulated_pos_4 += 1
                elif gap4 < -0.5: c4 = 2; simulated_pos_4 -= 1
                
                encoded_byte = c1 | (c2 << 2) | (c3 << 4) | (c4 << 6)
                
                if idx < total_ticks:
                    self.moves[idx] = encoded_byte
                else:
                    self.moves.append(encoded_byte)
                
                t += tick_us
                idx += 1
                
                if idx % 100 == 0:
                    gc.collect()
                    
        self.simulated_positions = [
            simulated_pos_1,
            simulated_pos_2,
            simulated_pos_3,
            simulated_pos_4
        ]
        self.move_index = 0
        self.temporalPosition = 0
        self.complete = False
        print(f"Segment {segment_index + 1} pre-calculation complete.")

    @micropython.native
    def updateMotors(self):
        """
        Executes playback. If the current segment buffer finishes, precalculates
        the next segment on the fly.
        """
        if self.move_index >= len(self.moves):
            if self.current_segment_index + 1 < len(self.segments):
                self.current_segment_index += 1
                self.precalculate_segment(self.current_segment_index)
                if self.move_index >= len(self.moves):
                    self.complete = True
                    return
            else:
                self.complete = True
                return

        encoded_byte = self.moves[self.move_index]
        for i in range(4):
            value = (encoded_byte >> (i * 2)) & 0b11
            command = 0
            if value == 1:
                command = 1
            elif value == 2:
                command = -1
            if command != 0:
                self.motors[i].setDirection(command)
                self.motors[i].step()
        self.move_index += 1


