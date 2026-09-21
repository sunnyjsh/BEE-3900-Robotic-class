import cv2

video= cv2.VideoCapture(0)

width= int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
height= int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

writer= cv2.VideoWriter('basicvideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))


while True:
    ret,frame= video.read()

    writer.write(frame)

    cv2.imshow('frame', frame)
# ord('q') returns the Unicode code point of q; click "q" while recording it.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
writer.release()
cv2.destroyAllWindows()
