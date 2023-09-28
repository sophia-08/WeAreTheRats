import serial
import time
import sys

# Define the UART device and file path
uart_device = "/dev/cu.usbmodem14601"
file_path = "capture.csv"
file_index = 0
num_samples=150
if len(sys.argv) == 3:
    # print(sys.argv[1],sys.argv[2] )
    file_path = "./data/" + sys.argv[1]
    file_index = int(sys.argv[2])
else:
    print ("wrong args")
    exit(1)


# Open the UART device for reading by line
try:
    ser = serial.Serial(uart_device, baudrate=9600, timeout=2)
except serial.SerialException as e:
    print(f"Error opening UART: {e}")
    exit(1)

# Initialize a list to store the lines
lines = []
samples=0
file = open(file_path + "_" + str(file_index) + ".dat", "w")

try:
    while True:
        line = ser.readline().decode("utf-8").strip()


        # Only append non-empty lines
        if line:
            columns = line.split(',')        
            print(columns[0])
            # lines.append(line)
            if (int(columns[0]) == 0) :
                if file:
                    file.close()
                file = open(file_path + "_" + str(file_index) + ".dat", "w")
                file_index += 1
                file.write("lineno,aX,aY,aZ,gX,gY,gZ\n")
                file.write(line + "\n")
            else:
                file.write(line + "\n")

        # Write the header to the file

        # start_time = None

        # while True:
        #     line = ser.readline().decode("utf-8").strip()

        #     # Only append non-empty lines
        #     if line:
        #         print(line[0:3])
        #         # lines.append(line)
        #         file.write(line + "\n")

            #     if start_time is None:
            #         start_time = time.time()

            # # Check if 2 seconds have passed since the first non-empty line was read
            # if start_time is not None and time.time() - start_time >= 2:
            #     if len(lines) == num_samples:
            #         # Save the data to the file
            #         with open(file_path, "a") as file:
            #             file.write("\n".join(lines) + "\n")
            #             file.write("\n")
            #         samples += 1
            #         print(samples, " samples")
            #     else:
            #         print("Error: Incorrect number of lines read", len(lines))

            #     # Clear the lines list and reset the start time
            #     lines = []
            #     start_time = None

except KeyboardInterrupt:
    # Close the UART port when the user presses Ctrl+C
    ser.close()
    if file:
        file.close()
    print("UART port closed.")
except Exception as e:
    # Handle other exceptions
    print(f"Error: {e}")
finally:
    # Close the UART port and save any remaining data if an error occurs
    ser.close()
    if file:
        file.close()
    # with open(file_path, "a") as file:
    #     file.write("\n".join(lines) + "\n")
    print("UART port closed. Remaining data saved to capture.txt")
