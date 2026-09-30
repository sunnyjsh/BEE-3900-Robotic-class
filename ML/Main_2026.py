import serial
import time
import sys
import signal
import cv2
import numpy as np
import os

# Import Edge Impulse Runner
from edge_impulse_linux.image import ImageImpulseRunner

# ====== Constants & Paths ======
NAV_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8513332393535170A1E0-if00"  
BAUD_RATE = 115200

MODEL_FILENAME = 'bee3900_25.eim' 
IMAGE_SAVE_DIR = '/home/pi/Desktop/Image'

# ====== Global Variables ======
arduino_nav = None
runner = None
video = None

# ====== Visual UI Function ======
def update_ui(primary_text, secondary_text="", stand_counts=""):
    img = np.zeros((400, 800, 3), dtype=np.uint8)
    img[:] = (150, 255, 255) # Bright yellow background

    # Main Status Text
    cv2.putText(img, primary_text, (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 4)
    if secondary_text:
        cv2.putText(img, secondary_text, (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 3)
        
    # Stand Count Text (Bottom of screen)
    if stand_counts:
        cv2.putText(img, stand_counts, (30, 350), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 5)

    cv2.imshow("ASABE Main Task Status", img)
    cv2.waitKey(1)

# ====== Emergency Stop ======
def signal_handler(sig, frame):
    print("\n[EMERGENCY STOP] Sending Halt command to Arduino and shutting down!")
    shutdown_routine()
    sys.exit(0)

def shutdown_routine():
    if arduino_nav:
        arduino_nav.write(b"!\n")
        arduino_nav.close()
    if runner:
        runner.stop()
    if video:
        video.release()
    cv2.destroyAllWindows()

def connect_serial():
    global arduino_nav
    update_ui("Connecting...", "Checking Arduino Serial")
    try:
        arduino_nav = serial.Serial(NAV_SERIAL, BAUD_RATE, timeout=1)
        time.sleep(2) 
        print("Connected successfully to Arduino!\n")
    except Exception as e:
        update_ui("SERIAL FAILED!", str(e))
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
                        pass # Ignore standard routing messages to keep terminal clean
                    elif line == "DONE":
                        break
                    elif line == "HALTED":
                        print("  [Arduino Acknowledged Halt]")
                        shutdown_routine()
                        sys.exit(0)
        time.sleep(delay)
    except serial.SerialException:
        pass

# ====== Computer Vision Functions ======
def init_vision():
    global runner, video
    
    if not os.path.exists(IMAGE_SAVE_DIR):
        os.makedirs(IMAGE_SAVE_DIR)
        
    update_ui("Loading AI Model...", "Initializing Edge Impulse")
    print("Initializing Edge Impulse model...")
    
    try:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(dir_path, MODEL_FILENAME)
        runner = ImageImpulseRunner(model_path)
        runner.init()
    except Exception as e:
        update_ui("AI MODEL FAILED!", str(e))
        time.sleep(3)
        sys.exit(1)

    update_ui("Starting Camera...", "Warming up webcam")
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(1) # Let the camera sensor warm up

def identify_plant():
    # 1. Flush the camera buffer to get a fresh, real-time frame
    # (Since the robot was moving, old blurry frames are stuck in the queue)
    for _ in range(5):
        video.grab()
        
    ret, raw_frame = video.read()
    if not ret:
        print("Error: Failed to grab frame from camera.")
        return "EMPTY" # Default to empty if camera fails so we don't break the rules by actuating

    # 2. Prevent Side Cropping (Square Padding)
    h, w = raw_frame.shape[:2]
    diff = w - h
    top, bottom = diff // 2, diff // 2
    square_frame = cv2.copyMakeBorder(raw_frame, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    rgb_frame = cv2.cvtColor(square_frame, cv2.COLOR_BGR2RGB)
    
    # 3. Run Inference
    features, cropped = runner.get_features_from_image(rgb_frame)
    res = runner.classify(features)
    
    # 4. Process Bounding Boxes
    valid_boxes = []
    if "bounding_boxes" in res["result"]:
        valid_boxes = [b for b in res["result"]["bounding_boxes"] if b['value'] > 0.5]

    num_plants = len(valid_boxes)

    # 5. Determine Plant Type based on ASABE Rules
    if num_plants == 0:
        plant_type = "EMPTY"
    elif num_plants == 1:
        plant_type = "SINGLE"
    else: 
        plant_type = "DOUBLE"

    # 6. Save Raw Image for Debugging
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    img_filename = os.path.join(IMAGE_SAVE_DIR, f"plant_{timestamp}_{plant_type}.jpg")
    cv2.imwrite(img_filename, raw_frame)

    # 7. Draw Boxes on Camera View
    cropped_h, cropped_w = cropped.shape[:2]
    ratio_x = square_frame.shape[1] / cropped_w
    ratio_y = square_frame.shape[0] / cropped_h
    display_frame = square_frame.copy()

    for b in valid_boxes[:2]: 
        x = int(b['x'] * ratio_x)
        y = int(b['y'] * ratio_y)
        bw = int(b['width'] * ratio_x)
        bh = int(b['height'] * ratio_y)
        cx = x + bw // 2
        cy = y + bh // 2
        label_text = b['label'] 
        
        box_color = (0, 255, 0) if label_text.lower() == "good" else (255, 0, 255)
        cv2.circle(display_frame, (cx, cy), 5, box_color, -1)
        cv2.rectangle(display_frame, (x, y), (x + bw, y + bh), box_color, 2)
        cv2.putText(display_frame, f"{label_text} ({b['value']:.2f})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

    cv2.imshow("Robot AI Vision", display_frame)
    cv2.waitKey(1)

    return plant_type


# ====== Main Competition Routine ======
def run_main_task():
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize everything before starting the timer
    init_vision()
    connect_serial()

    print("========================================================")
    print("STARTING 2026 ASABE MAIN TASK")
    print("========================================================\n")

    global_start_time = time.time()
    TIME_LIMIT = 600.0 # 10-Minute Trial Limit
    
    turn_sequence = ["SWR", "SWL", "SWR", "SWL"] 
    
    # Stand Count Trackers
    single_plants = 0
    double_plants = 0
    empty_plants = 0

    try:
        for i in range(5):
            print(f"\n--- Driving Row {i + 1} ---")
            stand_str = f"S:{single_plants} D:{double_plants} E:{empty_plants}"
            
            update_ui(f"Row {i + 1}", "Clearing starting border...", stand_str)
            send_nav("TF120")
            
            # Track down the row (5 Plants + 1 Boundary Edge)
            for node in range(6):
                if time.time() - global_start_time >= TIME_LIMIT:
                    print("\n--- 10 MINUTE TIME LIMIT REACHED ---")
                    return

                stand_str = f"S:{single_plants} D:{double_plants} E:{empty_plants}"
                update_ui(f"Row {i + 1}", f"Hunting for Node {node + 1}...", stand_str)
                
                # Dynamic Parameterized Searching
                if node == 0:
                    if i == 0:
                        send_nav("TS250,400") # Row 1 Start to Plant 1
                    else:
                        send_nav("TS150,350") # Rows 2-5 Start to Plant 1
                else:
                    send_nav("TS100,250") # Adjusted for the F75 push
                
                print(f"    Node {node + 1} reached.")

                # === ACTUATION & AI STAND COUNTING ===
                if node < 5: 
                    # Pause for a split second to let the camera stabilize from the sudden stop
                    time.sleep(0.3)
                    
                    # RUN AI VISION MODEL
                    plant_type = identify_plant()
                    
                    if plant_type == "DOUBLE":
                        double_plants += 1
                        stand_str = f"S:{single_plants} D:{double_plants} E:{empty_plants}"
                        update_ui(f"Row {i + 1}", "DOUBLE DETECTED! Actuating...", stand_str)
                        
                        send_nav("RD", wait_for_done=False)  
                        send_nav("LD", wait_for_done=False)  
                        send_nav("F75", wait_for_done=True) # Sweeps the double plant
                        send_nav("RU", wait_for_done=False)  
                        send_nav("LU", wait_for_done=False)
                        
                    elif plant_type == "SINGLE":
                        single_plants += 1
                        stand_str = f"S:{single_plants} D:{double_plants} E:{empty_plants}"
                        update_ui(f"Row {i + 1}", "Single Plant. Coasting...", stand_str)
                        send_nav("F75", wait_for_done=True) # Push past without arms down
                        
                    elif plant_type == "EMPTY":
                        empty_plants += 1
                        stand_str = f"S:{single_plants} D:{double_plants} E:{empty_plants}"
                        update_ui(f"Row {i + 1}", "Empty Node. Coasting...", stand_str)
                        send_nav("F75", wait_for_done=True) # Push past without arms down
                
                # 6th node (Boundary Edge) - Do not analyze, just push off.
                else:
                    send_nav("TF50", wait_for_done=True) 

            # Execute Edge U-Turn
            if i < 4:
                if time.time() - global_start_time >= TIME_LIMIT:
                    break

                print(f"\nExecuting Edge U-Turn Sequence...")
                turn_direction = "RIGHT" if turn_sequence[i] == "SWR" else "LEFT"
                stand_str = f"S:{single_plants} D:{double_plants} E:{empty_plants}"
                
                update_ui(f"EDGE TURN {turn_direction}", "Hunting for Corner...", stand_str)
                send_nav("TS150,350")
                
                send_nav("F90")  
                send_nav(turn_sequence[i]) 
                
                update_ui(f"EDGE TURN {turn_direction}", "Tracking to next row...", stand_str)
                send_nav("TS300,600") 
                
                update_ui(f"EDGE TURN {turn_direction}", "Pivoting into new row...", stand_str)
                send_nav("F90")
                send_nav(turn_sequence[i])
                
        # Final Stand Count Display 
        final_stand_str = f"S: {single_plants} | D: {double_plants} | E: {empty_plants}"
        update_ui("COURSE COMPLETE!", "Final Count:", final_stand_str)
        
        print("\n========================================================")
        print("COURSE COMPLETE")
        print(f"FINAL STAND COUNT -> {final_stand_str}")
        print("========================================================\n")
        
        # Hold UI open for judges to see the count
        time.sleep(15) 
        
    finally:
        shutdown_routine()

if __name__ == "__main__":
    run_main_task()