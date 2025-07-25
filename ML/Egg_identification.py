# The steps implemented in the object detection sample code: 
# 1. for an image of width and height being (w, h) pixels, resize image to (w', h'), where w/h = w'/h' and w' x h' = 262144
# 2. resize network input size to (w', h')
# 3. pass the image to network and do inference
# (4. if inference speed is too slow for you, try to make w' x h' smaller, which is defined with DEFAULT_INPUT_SIZE (in object_detection.py or ObjectDetection.cs))
import sys
import serial
from threading import Timer
import tflite_runtime.interpreter as tf
#from tflite_runtime.interpreter import Interpreter
import numpy as np
from PIL import Image
from object_detection import ObjectDetection
import cv2
import helper
import colorsys
from time import time

# model file name and label file name
MODEL_FILENAME = 'Final.tflite'
LABELS_FILENAME = '500.txt'

class TFLiteObjectDetection(ObjectDetection):
    """Object Detection class for TensorFlow Lite"""
    def __init__(self, model_filename, labels):
        super(TFLiteObjectDetection, self).__init__(labels)	
        self.interpreter = tf.Interpreter(model_path=model_filename)
        self.interpreter.allocate_tensors()
        self.interpreter.invoke()  # Warm-up, which may improve inference speed

        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]['index']
        self.output_index = self.interpreter.get_output_details()[0]['index']

    def predict(self, preprocessed_image):
        inputs = np.array(preprocessed_image, dtype=np.float32)[np.newaxis, :, :, (0, 1, 2)]  # RGB -> BGR and add 1 dimension.

        # Resize input tensor and re-allocate the tensors.
        self.interpreter.resize_tensor_input(self.input_index, inputs.shape)
        self.interpreter.allocate_tensors()
        
        self.interpreter.set_tensor(self.input_index, inputs)
        self.interpreter.invoke()
        return self.interpreter.get_tensor(self.output_index)[0]
    
def main(image_filename=''):
    # Load labels
    with open(LABELS_FILENAME, 'r') as f:
        labels = [l.strip() for l in f.readlines()]

    od_model = TFLiteObjectDetection(MODEL_FILENAME, labels)
    
    video_object = cv2.VideoCapture(0)
    
    video_object.set(cv2.CAP_PROP_FPS, 30)  # Frame rate 30 FPS
    video_object.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Image width
    video_object.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    
    previous = time()
    delta = 0
    found = False
    start = True
    t_0 = 0
    # the time interval between detections (s)
    t_wait = 1	
    # the time image shows on the screen (ms)
    t_show = 500
    
    
    #try:
    while(start):
        
        if found and time() - t_0 > t_show/1000:
            found = False
            previous = time()
        while(not found):
            # Get the current time, increase delta and update the previous variable
            current = time()
            delta += current - previous
            previous = current
            
            ret,frame = video_object.read()
            
            # Update orientation based on EXIF tags, if the file has orientation info.
            image = helper.update_orientation(frame)

            # Convert to OpenCV format
            #image = helper.convert_to_opencv(image)
                
            # If the image has either w or h greater than 1600 we resize it down respecting
            # aspect ratio such that the largest dimension is 1600
            image = helper.resize_down_to_1600_max_dim(image)
            
            # We next get the largest center square
            h, w = image.shape[:2]
            min_dim = min(w,h)
            max_square_image = helper.crop_center(image, min_dim, min_dim)
            
            # Resize that square down to 512x512
            augmented_image = cv2.resize(max_square_image, (512, 512 ))  # 降低输入分辨率

            # Check if t_wait (or some other value) seconds passed
            if delta > t_wait:
                # Operations on image
                #image = Image.open(image_filename)
                
                # Predict image using PIL Image 
                predictions = od_model.predict_image(Image.fromarray(augmented_image))
                #(predictions)
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                
                # Looping through number of predictions
                for pred in predictions:
                    if pred['probability'] >= 0.65:
                        found = True
                        # Draw rectangle for each bounding box based on left, top pixel + width and height
                        topleft = (int(pred['boundingBox']['left'] * augmented_image.shape[0]), int(pred['boundingBox']['top'] * augmented_image.shape[1]))
                        bottomright = (int(topleft[0] + pred['boundingBox']['width'] * augmented_image.shape[0]), int(topleft[1] + pred['boundingBox']['height'] * augmented_image.shape[0]))
                        print(topleft)
                        print(bottomright)
                        
                        # Compute the center of boxes
                        x = (topleft[0] + bottomright[0]) // 2
                        y = (topleft[1] + bottomright[1]) // 2
                        
                        # text to put
                        text=f"{pred['tagName']} | {round(pred['probability'] * 100, 2)}%" 
                        
                        # draw rectangle and text on img
                        cv2.rectangle(augmented_image, topleft, bottomright, (255, 0 ,0), 2)
                        cv2.putText(augmented_image, text, topleft, font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                       
                        # -- Compute mean color value -- #

                        # Crop the image to the desired region (bounding box shrunk by 7 pixels to crop out hay along the borders)
                        cropped_egg = augmented_image[(topleft[1]):(bottomright[1]), (topleft[0]):(bottomright[0])]
                        
                        # Get the pixel values of the cropped region
                        pixels = cropped_egg.reshape(-1, 3)  # Flatten into list of pixel
                        
                        # Convert the list of pixels to a NumPy array for easier manipulation
                        pixel_array = np.array(pixels)
                        
                        # Calculate the average of the R, G, B values for all pixels in the region
                        average_color = np.mean(pixel_array, axis=0)
                        
                        r, g, b = average_color[::-1]  # BGR -> RGB
                        
                        avgHSV = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)  # Normalize to 0–1
                        brightness = avgHSV[2]
                        # In HSV (Hue, Saturation, Value) color space, a lower value indicates a darker color
                        print(f"Brightness= {avgHSV[2]}")

                if found :
                    cv2.imshow('Frames',augmented_image)
                    cv2.waitKey(t_show)
                    t_0 = time()
                    
                # Reset the time counter
                delta = 0                
            
            # show the frame on the newly created image window
            if not found:
                cv2.imshow('Frames',augmented_image)
                cv2.waitKey(1)	
    
if __name__ == '__main__':
    main()
