import serial
import time

# Define the UART device and file path
uart_device = "/dev/cu.usbmodem14601"
file_path = "capture.csv"

# Open the UART device for reading by line
try:
    ser = serial.Serial(uart_device, baudrate=9600, timeout=2)
except serial.SerialException as e:
    print(f"Error opening UART: {e}")
    exit(1)

# Initialize a list to store the lines
lines = []
samples=0
# Write the header to the file
with open(file_path, "w") as file:
    file.write("aX,aY,aZ,gX,gY,gZ\n")

try:
    start_time = None

    while True:
        line = ser.readline().decode("utf-8").strip()

        # Only append non-empty lines
        if line:
            print(".", end='')
            lines.append(line)

            if start_time is None:
                start_time = time.time()

        # Check if 2 seconds have passed since the first non-empty line was read
        if start_time is not None and time.time() - start_time >= 2:
            if len(lines) == 416:
                # Save the data to the file
                with open(file_path, "a") as file:
                    file.write("\n".join(lines) + "\n")
                    file.write("\n")
                samples += 1
                print(samples, " samples")
            else:
                print("Error: Incorrect number of lines read", len(lines))

            # Clear the lines list and reset the start time
            lines = []
            start_time = None

except KeyboardInterrupt:
    # Close the UART port when the user presses Ctrl+C
    ser.close()
    print("UART port closed.")
except Exception as e:
    # Handle other exceptions
    print(f"Error: {e}")
finally:
    # Close the UART port and save any remaining data if an error occurs
    ser.close()
    # with open(file_path, "a") as file:
    #     file.write("\n".join(lines) + "\n")
    print("UART port closed. Remaining data saved to capture.txt")
