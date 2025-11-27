import utime
import motor
from machine import UART, Timer

# --- Motor Setup ---
motors = [
    motor.Motor(pins=[20,17,18,19,16], invertDirection=True, currentPosition=6503),
    motor.Motor(pins=[11,13,12,14,15]),
    motor.Motor(pins=[10,7,9,21,8], invertDirection=False, currentPosition=3826),
    motor.Motor(pins=[2,3,4,6,5], invertDirection=False, currentPosition=5980)
]

# --- UART Setup ---
uart = UART(0, baudrate=115200)

# --- Globals for Timer ---
current_motor_positions = [m.position for m in motors] # Initialize with actual motor positions

def decode_and_step(encoded_byte):
    global current_motor_positions
    for i in range(4):
        value = (encoded_byte >> (i * 2)) & 0b11
        command = 0
        if value == 1:
            command = 1
        elif value == 2:
            command = -1
        
        if command != 0:
            motors[i].setDirection(command)
            motors[i].step()
            current_motor_positions[i] = motors[i].position # Update tracked position

def timer_callback(t):
    global move_index
    if move_index < len(command_buffer):
        encoded_byte = command_buffer[move_index]
        decode_and_step(encoded_byte)
        move_index += 1
    else:
        # Move complete
        t.deinit()
        print("Move complete.")

def main_loop():
    global command_buffer, move_index
    while True:
        # Wait for a move sequence or position request
        if uart.any():
            byte = uart.read(1)
            if byte == b'S':
                # Start of move sequence, receive all commands
                command_buffer = bytearray()
                while True:
                    if uart.any():
                        byte = uart.read(1)
                        if byte == b'E':
                            break
                        command_buffer.extend(byte)
                
                # All commands received, start the timed execution
                print(f"Received {len(command_buffer)} move commands. Starting move.")
                move_index = 0
                timer = Timer()
                timer.init(freq=500, mode=Timer.PERIODIC, callback=timer_callback)
            elif byte == b'R': # New command for reporting position
                pos_str = ",".join(map(str, current_motor_positions)) + "\n"
                uart.write(pos_str.encode())
                print(f"Pico reported position: {pos_str.strip()}")

# --- Main Execution ---
print("Pico motor controller ready.")
for m in motors:
    m.enable()

main_loop()