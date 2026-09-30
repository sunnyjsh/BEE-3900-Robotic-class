import serial
import time
import sys

# ====== Constants ======
NAV_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8513332393535170A1E0-if00"  
BAUD_RATE = 115200

# ====== Global Variables ======
arduino_nav = None

def connect_serial():
    global arduino_nav
    print("Connecting to Arduino...")
    try:
        arduino_nav = serial.Serial(NAV_SERIAL, BAUD_RATE, timeout=1)
        time.sleep(2) 
        print("Connected successfully to Arduino!\n")
    except Exception as e:
        print(f"SERIAL FAILED! {e}")
        time.sleep(3)
        sys.exit(1)

def send_nav(cmd, delay=0.2, wait_for_done=True):
    try:
        arduino_nav.write(f"{cmd}\n".encode())
        print(f"  -> Sent: {cmd}")
        if wait_for_done:
            start_time = time.time()
            while time.time() - start_time < 20.0:
                if arduino_nav.in_waiting:
                    line = arduino_nav.readline().decode('utf-8').strip()
                    if line.startswith("MSG:"):
                        pass 
                    elif line == "DONE":
                        break
                    elif line == "HALTED":
                        print("  [Arduino Acknowledged Halt]")
                        sys.exit(0)
        time.sleep(delay)
    except serial.SerialException:
        print("Serial communication error.")

def run_simple_move():
    connect_serial()
    print("Starting back-and-forth movement. Press Ctrl+C to stop.")
    
    try:
        while True:
            # Move forward
            send_nav("F200")
            time.sleep(1) # Brief pause before reversing
            
            # Move backward
            send_nav("B200")
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[STOP] Keyboard interrupt detected. Shutting down!")
        if arduino_nav:
            arduino_nav.write(b"!\n") # Send emergency halt
            arduino_nav.close()
        sys.exit(0)

if __name__ == "__main__":
    run_simple_move()