#!/home/sagar/opencv_ws/envs/bin/python

import cv2 as cv
# url = "http://192.168.227.107:8080/video"
cap = cv.VideoCapture("shadows.mp4")
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    frame_resized = cv.resize(frame, None, fx=0.1, fy=0.1, interpolation=cv.INTER_AREA)
    # Our operations on the frame come here
    gray = cv.cvtColor(frame_resized, cv.COLOR_BGR2GRAY)
    # Display the resulting frame
    cv.imshow('frame', gray)
    if cv.waitKey(1) == ord('q'):
        break
        

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
