import cv2
# importing os module
import os

# Image path ; r' means the raw string
image_path = r'/Users/sj737/Library/Mobile Documents/com~apple~CloudDocs/New/temp.jpg'
# If you are using a PC, then the path would be like 'C:\Users\sj737\Desktop\New\temp.jpg'

# Image directory
directory = r'/Users/sj737/Library/Mobile Documents/com~apple~CloudDocs/New/'

# Using cv2.imread() method to read the image
img = cv2.imread(image_path)

# Change the current directory to specified directory
os.chdir(directory)

# List files and directories in the path above
print("Before saving image:")
print(os.listdir(directory))

# Filename
filename = 'savedImage.jpg'

# Using cv2.imwrite() method of saving the image
cv2.imwrite(filename, img)

# List files and directories in the path above
print("After saving image:")
print(os.listdir(directory))

print('Successfully saved')
