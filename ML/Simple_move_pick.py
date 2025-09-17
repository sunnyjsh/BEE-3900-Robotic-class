import serial
import time
import math
import threading
import signal
import os

# ====== Constants ======
NAV_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_3433031353135191A2E0-if00"  # Navigation Arduino
ARM_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_34333323832351C031B2-if00"  # Robotic Arm Arduino
BAUD_RATE = 115200
ARM_BAUD = 115200

# Direction Constants
NORTH = 0
SOUTH = 180

# ====== Global Variables ======
pose = [0, 0]
yaw = 0
current_yaw = 0
exit_event = threading.Event()

# ====== Initialize Serial Ports ======
try:
    arduino_nav = serial.Serial(NAV_SERIAL, BAUD_RATE, timeout=1)
    print("Navigation Arduino connected")
except Exception as e:
    print(f"Navigation Arduino connection failed: {str(e)}")
    arduino_nav = None

try:
    arduino_arm = serial.Serial(ARM_SERIAL, ARM_BAUD, timeout=1)
    print("Robotic Arm Arduino connected")
    time.sleep(2)
    arduino_arm.write(b"ZERO\n")
    wait_success = False
    timeout = time.time() + 2.0
    while time.time() < timeout:
        if arduino_arm.in_waiting:
            line = arduino_arm.readline().decode().strip()
            if line == "BASELINE_RESET":
                wait_success = True
                break
    if wait_success:
        print("Ard_Arm connected")
    else:
        print("Ard_Arm is still problem")
except Exception as e:
    print(f"Robotic Arm Arduino connection failed: {str(e)}")
    arduino_arm = None

# ====== Helper Functions ======
def normalize_angle(angle):
    """Normalizes an angle to the -180 to 180 degree range."""
    angle = angle % 360
    if angle > 180: angle -= 360
    return angle

def wait_for(ser, token, timeout=5.0):
    """Waits for a specific string token from a serial port with a timeout."""
    if not ser: return False
    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line == token: return True
    return False

def query_yaw_from_nav():
    """Queries the navigation Arduino for the current IMU yaw angle."""
    global arduino_nav
    if not arduino_nav: return 0
    try:
        arduino_nav.write(b"YAW\n")
        timeout = time.time() + 1.0
        while time.time() < timeout:
            if arduino_nav.in_waiting:
                line = arduino_nav.readline().decode().strip()
                if line.startswith("IMU_YAW="):
                    return normalize_angle(int(line.split('=')[1]))
        print("Warning: Querying IMU angle timed out")
        return 0
    except Exception as e:
        print(f"Warning: Error querying IMU angle: {str(e)}")
        return 0

def try_reconnect_arm():
    """Attempts to reconnect to the robotic arm's serial port."""
    global arduino_arm
    if arduino_arm:
        try: arduino_arm.close()
        except: pass
    arduino_arm = None
    print("Attempting to reconnect Robotic Arm Arduino...")
    try:
        arduino_arm = serial.Serial(ARM_SERIAL, ARM_BAUD, timeout=1)
        print("Robotic Arm Arduino reconnected")
        return True
    except Exception as e:
        print(f"Robotic Arm Arduino reconnection failed: {str(e)}")
        return False

def try_reconnect_serial():
    """Attempts to reconnect to the navigation Arduino's serial port."""
    global arduino_nav
    if arduino_nav:
        try: arduino_nav.close()
        except: pass
    arduino_nav = None
    print("Attempting to reconnect Navigation Arduino...")
    try:
        arduino_nav = serial.Serial(NAV_SERIAL, BAUD_RATE, timeout=1)
        print("Navigation Arduino reconnected")

        time.sleep(2)
        arduino_nav.write(b"ZERO\n")
        wait_success = False
        timeout = time.time() + 2.0
        while time.time() < timeout:
            if arduino_nav.in_waiting:
                line = arduino_nav.readline().decode().strip()
                if line == "BASELINE_RESET":
                    wait_success = True
                    break
        if wait_success:
            print("Ard_Nav connected")
        else:
            print("Ard_Nav is still problem")
    
    except Exception as e:
        print(f"Navigation Arduino reconnection failed: {str(e)}")
        return False

def send_arm(cmd, delay=0.5):
    """Sends a command to the robotic arm and waits for a 'DONE' confirmation."""
    global arduino_arm
    if not arduino_arm:
        print(f"Warning: Robotic Arm Arduino not connected, cannot send command: {cmd}")
        try_reconnect_arm()
        return
    try:
        arduino_arm.write(f"{cmd}\n".encode())
        print(f"Sent to Arm: {cmd}")
        if not wait_for(arduino_arm, "DONE", timeout=10.0):
            print(f"Warning: Timed out waiting for 'DONE' from Arm Arduino: {cmd}")
        time.sleep(delay)
    except Exception as e:
        print(f"Warning: Error sending arm command '{cmd}': {e}")
        try_reconnect_arm()

def send_nav(cmd, delay=0.5):
    """Sends a command to the navigation Arduino, waits for 'DONE', and updates robot pose."""
    global pose, yaw, current_yaw
    if not arduino_nav:
        print(f"Warning: Navigation Arduino not connected, cannot send command: {cmd}")
        try_reconnect_serial()
        return
    try:
        yaw_before = None
        if cmd.startswith(("L", "R", "ML", "MR")) and len(cmd) > 1:
            yaw_before = query_yaw_from_nav()
            print(f"Angle before rotation: {yaw_before:.1f} degrees")
        arduino_nav.write(f"{cmd}\n".encode())
        print(f"Sent to Nav: {cmd}")
        if not wait_for(arduino_nav, "DONE", timeout=10.0):
            print(f"Warning: Timed out waiting for 'DONE' from Nav Arduino: {cmd}")
        time.sleep(delay)
        current_yaw = query_yaw_from_nav()
        if cmd.startswith(("F", "B")) and len(cmd) > 1:
            dist = int(cmd[1:]) * (1 if cmd[0] == 'F' else -1)
            theta = math.radians(yaw)
            pose[0] += dist * math.cos(theta)
            pose[1] += dist * math.sin(theta)
        elif yaw_before is not None:
            yaw_after = current_yaw
            delta = normalize_angle(yaw_after - yaw_before)
            yaw = normalize_angle(yaw + delta)
            print(f"Updated global yaw: {yaw:.1f} degrees")
    except Exception as e:
        print(f"Warning: Error sending nav command '{cmd}': {e}")
        try_reconnect_serial()

def rotate_to_direction(target_angle):
    """Rotates the robot to a specific target angle using IMU feedback."""
    max_attempts = 3
    for attempt in range(max_attempts):
        current_angle = query_yaw_from_nav()
        print(f"Current: {current_angle:.1f} degrees, Target: {target_angle} degrees")
        angle_diff = normalize_angle(target_angle - current_angle)
        if abs(angle_diff) < 8:
            print(f"Rotation complete. Final error: {angle_diff:.1f} degrees")
            return True
        command = f"MR{int(min(angle_diff, 90))}" if angle_diff > 0 else f"ML{int(min(abs(angle_diff), 90))}"
        send_nav(command)
        time.sleep(1.5)
    final_error = normalize_angle(target_angle - query_yaw_from_nav())
    print(f"Rotation finished. Final error: {final_error:.1f} degrees")
    return abs(final_error) < 15
    
def execute_scan():
    """
    Sends 'SCAN' command to the navigation Arduino and parses the simplified response.
    Returns a single integer distance in mm, or None on failure.
    """
    global arduino_nav
    if not arduino_nav:
        print("Cannot execute scan, Navigation Arduino not connected.")
        try_reconnect_serial()
        return None

    try:
        arduino_nav.flushInput()  # Clear any old data from the input buffer
        arduino_nav.write(b"SCAN\n")
        print("Executing SCAN...")

        distance_result = None
        timeout = time.time() + 5.0  # 5-second timeout for the scan

        while time.time() < timeout:
            if arduino_nav.in_waiting:
                line = arduino_nav.readline().decode().strip()
                if not line: continue

                #print(f"  -> Received: {line}") # Debug print

                # Check for the simplified SCAN_RESULT line
                if line.startswith("SCAN_RESULT:"):
                    try:
                        # e.g., "SCAN_RESULT:450" -> value = 450
                        value = int(line.split(':')[1])
                        distance_result = value
                        #print(f"Parsed distance: {value}mm")
                    except (IndexError, ValueError) as e:
                        print(f"Could not parse line: {line} ({e})")
                
                # Check for completion
                elif line == "DONE":
                    #print("SCAN complete.")
                    return distance_result # Return the found distance

        print("SCAN timed out. Did not receive 'DONE'.")
        return distance_result # Return partial data if available

    except Exception as e:
        print(f"An error occurred during SCAN: {str(e)}")
        try_reconnect_serial()
        return None

# ====== Main Task Sequence ======
def execute_pickup_and_return_sequence():
    """
    Rotates the arm-mounted sensor to scan left, right, and center,
    printing the distance at each point.
    """
    try:
#         print("\n--- Starting 3-Point Environment Scan ---")
# 
#         # 1. Scan Left
#         print("\nStep 1: Scanning Left...")
#         send_arm("GL")  # Command arm to rotate to the left position
#         distance_left = execute_scan()
#         if distance_left is not None:
#             print(f"--> Left Distance: {distance_left}mm")
#         else:
#             print("--> Left scan failed.")
# 
#         time.sleep(1) # Pause between actions
# 
#         # 2. Scan Right
#         print("\nStep 2: Scanning Right...")
#         send_arm("GR")  # Command arm to rotate to the right position
#         distance_right = execute_scan()
#         if distance_right is not None:
#             print(f"--> Right Distance: {distance_right}mm")
#         else:
#             print("--> Right scan failed.")
# 
#         time.sleep(1) # Pause
# 
#         # 3. Scan Center and Reset Position
#         print("\nStep 3: Scanning Center...")
#         send_arm("GC")  # Command arm to rotate back to the center
#         distance_center = execute_scan()
#         if distance_center is not None:
#             print(f"--> Center Distance: {distance_center}mm")
#         else:
#             print("--> Center scan failed.")
# 
#         # Initialize robot state
#         yaw = query_yaw_from_nav()
#         print(f"Starting sequence with initial yaw: {yaw:.1f} degrees")
#         
        # EX) Complex motions
        send_nav("F30") 
        send_arm("G1")
        send_nav("B30")
        send_arm("G2")
        send_nav("MR10")
        send_nav("ML30")
        send_arm("G0")

        print("\n Scan sequence complete!")

    except Exception as e:
        print(f"An error occurred during the sequence: {str(e)}")
        import traceback
        traceback.print_exc()

# ====== Main Program Entry Point ======
if __name__ == "__main__":
    def signal_handler(sig, frame):
        """Handles program interruption (Ctrl+C) to safely close serial ports."""
        print("\nProgram interrupted, cleaning up...")
        exit_event.set()
        if arduino_nav:
            try: arduino_nav.close()
            except: pass
        if arduino_arm:
            try: arduino_arm.close()
            except: pass
        print("Program exited.")
        os._exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    
    # Check for Arduino connections
    if not arduino_nav or not arduino_arm:
        print("One or both Arduinos not connected. Please check connections and try again.")
        os._exit(1)
        
    # Run the main task
    execute_pickup_and_return_sequence()
    
    print("Program finished.")
