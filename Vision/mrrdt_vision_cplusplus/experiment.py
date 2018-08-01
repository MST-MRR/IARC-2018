import dlib
import cv2
import iris
import numpy as np
from time import sleep

cap = cv2.VideoCapture('grid.ogv')

while(cap.isOpened()):
    ret, frame = cap.read()
    if not ret: break
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(grayscale, 5)
    ret, thr = cv2.threshold(blurred, .95*255, 255, cv2.THRESH_BINARY)
    skeleton = dlib.skeleton(thr)
    lines = cv2.HoughLines(skeleton, 1, np.pi/180, 250)
    if lines is not None:
        for rho,theta in lines[:, 0, :]:
            
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 10000*(-b))
            y1 = int(y0 + 10000*(a))
            x2 = int(x0 - 10000*(-b))
            y2 = int(y0 - 10000*(a))

            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)

    # end_of_grid_present = not np.count_nonzero(thr[:20, :])
    # cv2.putText(skeleton, 'grid border detected' if end_of_grid_present else 'no grid border detected', (50, 75), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow('detections', frame)
    cv2.imshow('grid', skeleton)
    key = cv2.waitKey(1)
    
    if (chr(key & 0xFF) == 'q'):
        break