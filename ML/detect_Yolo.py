import cv2
from ultralytics import YOLO

# Load the lightweight Nano model (it will auto-download the 6MB file on first run)
model = YOLO("yolov8n.pt")

# Initialize the USB camera (Index 0 is typically the first connected USB webcam)
cap = cv2.VideoCapture(0)

print("Starting YOLO detection. Press 'q' in the video window to exit.")

while cap.isOpened():
    # Capture frame-by-frame
    success, frame = cap.read()
    if not success:
        print("Failed to grab frame from camera. Exiting.")
        break

    # Run inference on the frame
    # Reducing imgsz to 320 significantly speeds up processing on the Raspberry Pi
    results = model(frame, imgsz=320)

    # The plot() function draws the bounding boxes and confidence scores onto the frame
    annotated_frame = results[0].plot()

    # Display the resulting frame
    cv2.imshow("YOLOv8 USB Camera Detection", annotated_frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all graphical windows
cap.release()
cv2.destroyAllWindows()