from machine import UART

uart = UART(0, baudrate=115200)


while True:
    if uart.any():  # Check if any data is available
        message = uart.read()  # Read the incoming message
        if message:  # If a message is received
            number = int(message)
