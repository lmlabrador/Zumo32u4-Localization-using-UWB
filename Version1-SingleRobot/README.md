This project enables a Zumo robot to localize and navigate using UWB data on a Raspberry Pi.

The repository contains Python scripts to calibrate the robot's movement, process UWB data, and drive the robot to a target position.

Overview:
• Calibration: The calibrate.py script measures encoder counts for accurate distance and turning.
• Navigation: The main.py script reads UWB positions and navigates the robot using helper functions in navigation.py.
• Communication: SerialGateway.py, uwb_reader.py, and Zumo.py handle serial communication with the Zumo robot and UWB module.

Prerequisites:
• Hardware: Raspberry Pi4 B, Zumo robot (running the ZumoSerial sketch), and a UWB module.
• Software: Python 3 and the pyserial library.
To install pyserial, run: pip install pyserial

Setup and Usage:

    Clone the Repository:
        git clone https://github.com/yourusername/Zumo32u4-Localization-using-UWB.git
        cd Zumo32u4-Localization-using-UWB

    (Optional) Calibrate the Robot:
        Run the calibration script to update encoder values if needed: python calibrate.py

    Run the Navigation Script:
        Navigate the robot to a target position by providing the target X and Y coordinates:
            python main.py <target_x> <target_y>
            Example: python main.py 1.0 2.0

Monitor the Output:
The terminal will display UWB data, heading information, and movement commands as the robot navigates.

Notes:
• Ensure the serial port configurations in SerialGateway.py and uwb_reader.py match your Raspberry Pi's setup.
• Adjust calibration constants in calibrate.py if the robot’s behavior deviates from expected performance.
