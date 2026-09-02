import os
import cv2
import time
import numpy as np

# Import Edge Impulse Runner
from edge_impulse_linux.image import ImageImpulseRunner

# ====== Model Parameters ======
MODEL_FILENAME = 'bee3900_25.eim' 
IMAGE_SAVE_DIR = '/home/pi/Desktop/Image'

# ====== Stand Count Variables ======
# Rule format required: S: NumSingle, D: NumDouble, E: NumEmpty
stand_s = 0
stand_d = 0
stand_e = 0

def run_identification():
    global stand_s, stand_d, stand_e
    
    # Ensure the image saving directory exists
    if not os.path.exists(IMAGE_SAVE_DIR):
        os.makedirs(IMAGE_SAVE_DIR)
        print(f"Created directory for saving images: {IMAGE_SAVE_DIR}")
        
    print("Initializing Edge Impulse model for Plant Identification...")
    runner = None
    try:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(dir_path, MODEL_FILENAME)
        runner = ImageImpulseRunner(model_path)
        runner.init()
    except Exception as e:
        print(f"Error initializing Edge Impulse runner: {e}")
        return

    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("==========================================================")
    print("Camera active. Running fully autonomous identification.")
    print("Auto-logging the current plant every 2 seconds.")
    print("Raw images will be saved to the 'Image' folder.")
    print("Task will automatically terminate after exactly 2 minutes.")
    print("Press 'Q' to quit early and display final count.")
    print("==========================================================")
    
    start_time = time.time()
    duration_limit = 120.0  # 2 minutes in seconds
    
    try:
        while True:
            # --- Check 2-Minute Time Limit ---
            current_time = time.time()
            elapsed_time = current_time - start_time
            time_left = duration_limit - elapsed_time
            
            if elapsed_time >= duration_limit:
                print("\n--- 2 Minute Time Limit Reached. Stopping Task. ---")
                break

            # --- Flush the camera buffer to guarantee a real-time frame ---
            for _ in range(4):
                video.grab()
            ret, raw_frame = video.read()
            if not ret: continue
            
            # --- Prevent Side Cropping (Square Padding) ---
            h, w = raw_frame.shape[:2]
            diff = w - h
            top, bottom = diff // 2, diff // 2
            square_frame = cv2.copyMakeBorder(raw_frame, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
            
            rgb_frame = cv2.cvtColor(square_frame, cv2.COLOR_BGR2RGB)
            
            # Run Inference
            features, cropped = runner.get_features_from_image(rgb_frame)
            res = runner.classify(features)
            
            # Mapping ratios for drawing boxes
            cropped_h, cropped_w = cropped.shape[:2]
            ratio_x = square_frame.shape[1] / cropped_w
            ratio_y = square_frame.shape[0] / cropped_h
            
            display_frame = square_frame.copy()
            
            valid_boxes = []
            if "bounding_boxes" in res["result"]:
                # Confidence threshold set to 50%
                valid_boxes = [b for b in res["result"]["bounding_boxes"] if b['value'] > 0.5]

            num_plants = len(valid_boxes)

            # =========================================================
            # OFFICIAL RULE COLORS & TEXT
            # =========================================================
            if num_plants == 0:
                status_color = (0, 0, 255) # Red (Empty Plant)
                status_text = "Empty Plant"
                current_type = "E"
            elif num_plants == 1:
                status_color = (0, 255, 0) # Green (Single Plant)
                status_text = "Single Plant"
                current_type = "S"
            else: 
                status_color = (255, 0, 0) # Blue (Double Plant)
                status_text = "Double Plant"
                current_type = "D"

            # =========================================================
            # AUTO-LOGGING EVERY 2 SECONDS
            # =========================================================
            if current_type == "S":
                stand_s += 1
            elif current_type == "D":
                stand_d += 1
            elif current_type == "E":
                stand_e += 1
                
            stand_count_str = f"S: {stand_s}  ,  D: {stand_d}  ,  E: {stand_e}"
            print(f"[{int(time_left)}s left] Auto-Logged {status_text}! -> {stand_count_str}")

            # =========================================================
            # SAVE RAW IMAGE
            # =========================================================
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            img_filename = os.path.join(IMAGE_SAVE_DIR, f"plant_{timestamp}.jpg")
            cv2.imwrite(img_filename, raw_frame)

            # Create a dedicated 640x300 solid color status window
            status_img = np.zeros((300, 640, 3), dtype=np.uint8)
            status_img[:] = status_color
            
            # Draw Real-Time Status Text
            cv2.putText(status_img, status_text, (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)
            
            # Draw Stand Count formatting required by rules
            cv2.putText(status_img, stand_count_str, (30, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
            
            # Show the Status Window
            cv2.imshow("Plant Status & Stand Count", status_img)

            # =========================================================
            # DRAW BOUNDING BOXES ON CAMERA VIEW
            # =========================================================
            if num_plants > 0:
                for b in valid_boxes[:2]: 
                    x = int(b['x'] * ratio_x)
                    y = int(b['y'] * ratio_y)
                    bw = int(b['width'] * ratio_x)
                    bh = int(b['height'] * ratio_y)
                    cx = x + bw // 2
                    cy = y + bh // 2
                    label_text = b['label'] 
                    
                    if label_text.lower() == "bad":
                        box_color = (255, 0, 255) # Magenta
                    elif label_text.lower() == "good":
                        box_color = (0, 255, 0) # Green
                    else:
                        box_color = (255, 255, 0) # Cyan
                    
                    cv2.circle(display_frame, (cx, cy), 5, box_color, -1)
                    cv2.rectangle(display_frame, (x, y), (x + bw, y + bh), box_color, 2)
                    cv2.putText(display_frame, f"{label_text} ({b['value']:.2f})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

            # Show the Camera View
            cv2.imshow("Camera View", display_frame)
            
            # =========================================================
            # 2-SECOND PAUSE & KEYBOARD QUIT
            # =========================================================
            # waitKey(2000) freezes the program for 2000ms (2 seconds)
            if cv2.waitKey(2000) & 0xFF == ord('q'):
                print("\n--- User Quit Early ---")
                break
            
    except KeyboardInterrupt:
        pass
    finally:
        if runner:
            runner.stop()
        video.release()
        cv2.destroyAllWindows() 
        
        # Rule check: Ensure the final output perfectly matches the required formatting
        print("\n=====================================")
        print("         FINAL STAND COUNT           ")
        print(f"        S: {stand_s} , D: {stand_d} , E: {stand_e}        ")
        print("=====================================\n")

if __name__ == "__main__":
    run_identification()
