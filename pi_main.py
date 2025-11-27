import time
import serial
import pi_kinematics
import os # For checking if file exists

# --- Configuration ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 115200
POSITION_FILE = "robot_position.txt"

# Initial positions of the motors on the Pico
# These should match the initial positions in the Pico's code
INITIAL_MOTOR_POSITIONS = [6503, 0, 3826, 5980]

# --- Global for current position ---
current_robot_position = list(INITIAL_MOTOR_POSITIONS) # Make it a mutable list

def load_position_from_file():
    global current_robot_position
    if os.path.exists(POSITION_FILE):
        try:
            with open(POSITION_FILE, "r") as f:
                data = f.read().strip()
                if data:
                    current_robot_position = list(map(int, data.split(',')))
                    print(f"Loaded position from file: {current_robot_position}")
                    return True
        except Exception as e:
            print(f"Error loading position from file: {e}")
    print(f"No position file found or error loading. Using initial positions: {current_robot_position}")
    return False

def save_position_to_file(position_data):
    try:
        with open(POSITION_FILE, "w") as f:
            f.write(",".join(map(str, position_data)))
        # print(f"Position saved: {position_data}") # Comment out for less console spam
    except Exception as e:
        print(f"Error saving position: {e}")

def main():
    global current_robot_position

    # --- Load position on startup ---
    load_position_from_file()

    # --- Setup Serial Port ---
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return

    last_save_time = time.time()

    # --- Main Loop for Robot Control and Position Saving ---
    # This loop will continuously run, performing robot control and saving position
    while True:
        # --- Robot Control Logic (Example: a move) ---
        # This part would be replaced by your actual robot control logic
        # For demonstration, let's just do one move and then loop for saving
        if not hasattr(main, 'move_done'): # Only do the move once for demo
            x1, y1 = 8, 8
            x2, y2 = 8, 1
            # Use the loaded position as the starting point for the move calculation
            move = pi_kinematics.PrecalculatedMove(x1, x2, y1, y2, current_robot_position)
            
            print(f"Sending {len(move.moves)} move commands to the Pico...")
            ser.write(b'S') # Send a start byte
            for encoded_byte in move.moves:
                ser.write(bytes([encoded_byte]))
            ser.write(b'E') # Send an end byte
            print("Move commands sent.")
            main.move_done = True # Mark move as done

        # --- Periodic Position Saving ---
        if time.time() - last_save_time >= 0.5:
            ser.write(b'R') # Request position from Pico
            response = ser.readline().decode().strip()
            if response:
                try:
                    # Update current_robot_position with what Pico reported
                    current_robot_position = list(map(int, response.split(',')))
                    save_position_to_file(current_robot_position)
                except ValueError:
                    print(f"Invalid position data received: {response}")
            else:
                print("No position response from Pico (timeout).")
            last_save_time = time.time()
        
        # Add a small sleep to prevent busy-waiting if nothing else is happening
        # This is important for the Pi Zero to not consume 100% CPU
        time.sleep(0.01)
 
    # --- Close the Serial Port ---
    ser.close()

if __name__ == '__main__':
    main()
