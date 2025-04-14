import serial
import time
import threading

class UWBReader:
    def __init__(self, port="/dev/ttyACM1"):
        self.ser = None
        self.latest_position = (None, None)
        self.running = False
        self.port = port

    def start(self):
        """Initialize the UWB module and start reading data in a separate thread. COnfugured based on the DWM documentation"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # Reduced timeout for faster response
            )
            print("Serial connection established.")
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            return False

        # Reset the UWB module (send a break signal)
        print("Resetting UWB module...")
        self.ser.send_break()
        time.sleep(1)  # Wait for the module to reset

        # Enter shell mode
        if not self.enter_shell_mode():
            print("Failed to enter shell mode. Exiting.")
            self.ser.close()
            return False

        # Start continuous streaming
        print("Starting continuous streaming...")
        self.ser.write(b"lep\n")
        time.sleep(1)  # Wait for the module to start streaming

        # Flush the input buffer to discard any old data
        self.ser.reset_input_buffer()

        # Start a thread to continuously read UWB data
        self.running = True
        self.thread = threading.Thread(target=self.read_uwb_data)
        self.thread.start()

        return True

    def stop(self):
        """Stop the UWB reader thread and close the serial connection."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if self.ser:
            self.ser.close()
        print("UWB reader stopped.")

    def enter_shell_mode(self):
        """Send double Enter and wait for the dwm> prompt."""
        print("Entering shell mode...")
        max_attempts = 5  # Maximum number of attempts to enter shell mode
        attempt = 0

        while attempt < max_attempts:
            self.ser.write(b"\n\n")  # Send double Enter
            time.sleep(0.5)  # Wait for the module to respond

            # Read the response
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode().strip()
                print(f"Response: {data}")  # Debugging: Print the raw response

                # Check if the dwm> prompt is received
                if "dwm>" or "POS" in data:
                    print("Shell mode entered.")
                    return True

            attempt += 1
            print(f"Attempt {attempt} failed. Retrying...")
            time.sleep(0.5)  # Small delay before retrying

        print("Failed to enter shell mode after multiple attempts.")
        return False

    def read_uwb_data(self):
        """Continuously read UWB data in a separate thread."""
        while self.running:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode().strip()
                print(f"Raw UWB data: {data}")  # Debugging: Print raw data

                # Parse the data
                try:
                    parts = data.split(',')
                    if parts[0] == "POS" and len(parts) >= 3:
                        x = float(parts[1])  # Extract x coordinate
                        y = float(parts[2])  # Extract y coordinate
                        self.latest_position = (x, y)
                        print(f"Latest position: ({x}, {y})")  # Debugging: Print latest position
                except (ValueError, IndexError) as e:
                    print(f"Failed to parse UWB data: {data}. Error: {e}")
            time.sleep(0.01)  # Small delay to avoid busy-waiting

    def get_latest_position(self):
        """Return the latest UWB position."""
        return self.latest_position
