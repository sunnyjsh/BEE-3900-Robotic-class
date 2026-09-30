import time
import sys
import cv2
import numpy as np
import os

# Import Edge Impulse Runner
from edge_impulse_linux.image import ImageImpulseRunner

# ====== Constants & Paths ======
MODEL_FILENAME = 'bee3900_25.eim' 
IMAGE_SAVE_DIR = '/home/pi/Desktop/Image'

# ====== Global Variables ======
runner = None
video = None

def shutdown_routine():
    if runner:
        runner.stop()
    if video:
        video.release()
    cv2.destroyAllWindows()

def init_vision():
    global runner, video
    
    if not os.path.exists(IMAGE_SAVE_DIR):
        os.makedirs(IMAGE_SAVE_DIR)
        
    print("Initializing Edge Impulse model...")
    try:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(dir_path, MODEL_FILENAME)
        runner = ImageImpulseRunner(model_path)
        runner.init()
    except Exception as e:
        print("AI MODEL FAILED!", str(e))
        sys.exit(1)

    print("Warming up webcam...")
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(1) # Let the camera sensor warm up

def identify_plant():
    # 1. Flush the camera buffer to get a fresh, real-time frame
    for _ in range(5):
        video.grab()
        
    ret, raw_frame = video.read()
    if not ret:
        print("Error: Failed to grab frame from camera.")
        return "EMPTY"

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

    # 5. Determine Plant Type 
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
    return plant_type

def run_vision_test():
    init_vision()
    print("\nStarting Vision Test.")
    print("Press 'q' while clicked on the video window or Ctrl+C in terminal to exit.\n")
    
    try:
        while True:
            # Run the plant identification function
            plant_type = identify_plant()
            print(f"Detected Plant Status: {plant_type}")
            
            # Pause for 1 second between inferences, and check if 'q' was pressed
            if cv2.waitKey(1000) & 0xFF == ord('q'):
                print("User pressed 'q'. Exiting...")
                break
                
    except KeyboardInterrupt:
        print("\n[STOP] Keyboard interrupt detected.")
    finally:
        shutdown_routine()

if __name__ == "__main__":
    run_vision_test()