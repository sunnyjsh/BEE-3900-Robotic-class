import cv2
import time


TIMER = int(5)
k=0

cap = cv2.VideoCapture(0)
capture=int(1)

while True:
    ret, img= cap.read()

    cv2.imshow('frame', img)
# press "t" to save an image.
    if cv2.waitKey(1) & 0xFF == ord('t'):
        cv2.imwrite('taken' + str(capture) + '.jpg', img)
        capture=capture+1

        break
    
cap.release()
cv2.destroyAllWindows()
