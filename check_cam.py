# ls /dev/video*

import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Camera', frame)
        cv2.waitKey(1)
    else:
        print('NO CAMERA')
