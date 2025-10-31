import utime
from motor import Motor
import machine

# This test script is designed to measure the performance of the motor.step()
# function in isolation. It creates a motor instance and calls step()
# thousands of times in a tight loop to get an accurate average execution time.

print("Starting motor step performance test...")

# IMPORTANT: The pin numbers here must match one of your actual motors.
# Using the pinout for the first motor from your main.py
MOTOR_PINS = [20, 17, 18, 19, 16]

try:
    motor_instance = Motor(pins=MOTOR_PINS)
except Exception as e:
    print("Error initializing motor. Please check your pin numbers.")
    print(e)

# Allow time for initialization
utime.sleep_ms(100)

# --- Test Execution ---
NUM_STEPS = 20000
print(f"Performing {NUM_STEPS} steps...")

# Warm-up run (optional, helps ensure caches are warm, etc.)
for _ in range(100):
    motor_instance.step()

# Timed run
local_step = motor_instance.step  # Cache the method to a local variable
start_time = utime.ticks_us()
for _ in range(NUM_STEPS):
    local_step()  # Call the local variable, avoiding attribute lookup in the loop
end_time = utime.ticks_us()

# --- Results ---
total_duration_us = utime.ticks_diff(end_time, start_time)
avg_time_us = total_duration_us / NUM_STEPS

print("\n--- Test Complete ---")
print(f"Total time for {NUM_STEPS} steps: {total_duration_us} us")
print(f"Average time per step: {avg_time_us:.4f} us")

if avg_time_us < 100:
    print("\nConclusion: The motor.step() function is very fast as expected.")
    print("The ~500us delay is likely coming from interactions within the main application loop.")
else:
    print("\nConclusion: The motor.step() function itself is unexpectedly slow.")
    print("The source of the delay is likely within the Motor class or a lower-level issue.")
