import board
from machine import UART, Pin
import motor as motor
from debug import stepFromREPL
import utime

motors = [
    motor.Motor(pins=[10,7,21,9,8], invertDirection=True),
    motor.Motor(pins=[2,3,6,4,5], invertDirection=False, currentPosition=3826),
    motor.Motor(pins=[20,17,18,19,16], invertDirection=False, currentPosition=3826),
    motor.Motor(pins=[12,11,13,14,15], currentPosition=5980)
]
'''
Motor Layout
2------3
|      |
|      |
0------1
'''

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1)) 

# Renamed to board_obj to avoid shadowing the 'import board' module
board_obj = board.Board(1, 1, motors) 
board_obj.disable() 

print("Pico UART Receiver Ready...")

while True:
    if uart.any():  
        # Using readline() is safer for command protocols to ensure you get the full string. 
        # (Make sure whatever is sending the serial commands terminates them with a newline '\n')
        raw_message = uart.readline()  
        
        if raw_message:  
            try:
                # Decode, remove whitespace/newlines, and capitalize for consistency
                clean_message = raw_message.decode('utf-8').strip().upper()
                print(f"Received: {clean_message}")
                
                # --- COMMAND: POS ---
                if clean_message == "POS":
                    # Send back current coordinates
                    uart.write(f"POS({board_obj.x},{board_obj.y})\r\n")
                    continue
                
                # --- COMMAND: MOV(x,y,...) ---
                if clean_message.startswith("MOV(") and clean_message.endswith(")"):
                    # Extract everything inside the parentheses
                    params = clean_message[4:-1]
                    parts = params.split(',')
                    
                    if len(parts) >= 2 and len(parts) % 2 == 0:
                        waypoints = []
                        valid = True
                        for i in range(0, len(parts), 2):
                            x_str = parts[i].strip()
                            y_str = parts[i+1].strip()
                            
                            if x_str.isdigit() and y_str.isdigit():
                                x = int(x_str)
                                y = int(y_str)
                                if 1 <= x <= 8 and 1 <= y <= 8:
                                    waypoints.append((x, y))
                                else:
                                    valid = False
                                    break
                            else:
                                valid = False
                                break
                        
                        if valid:
                            param_str = ",".join(f"{wx},{wy}" for wx, wy in waypoints)
                            # 1. Send Acknowledgment
                            uart.write(f"ACK({param_str})\r\n")
                            print(f"Executing Move -> {param_str}")
                            
                            # 2. Execute movement
                            board_obj.calculateMultiMove(waypoints)
                            board_obj.executeMove()
                            
                            # 3. Send completion message
                            uart.write(f"DONE({param_str})\r\n")
                        else:
                            # Valid format, but coordinates out of bounds or not numbers
                            uart.write(f"NAK({params})\r\n")
                    else:
                        # Incorrect number of comma-separated arguments
                        uart.write(f"NAK({params})\r\n")
                else:
                    # Unrecognized command format entirely
                    print(f"Ignored invalid command format: {clean_message}")
                    uart.write("NAK\r\n")
                    
            except Exception as e:
                print(f"Unexpected error parsing UART data: {e}")
                uart.write("NAK\r\n")
                
    utime.sleep(0.05) # Small buffer delay

board_obj.disable()
