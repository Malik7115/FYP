import cv2
import numpy as np

def nothing(x):
    pass 

cap = cv2.VideoCapture(1)
cv2.namedWindow('sliders')

cv2.createTrackbar('LH','sliders',0,255,nothing)
cv2.createTrackbar('UH','sliders',0,255,nothing)
cv2.createTrackbar('LS','sliders',0,255,nothing)
cv2.createTrackbar('US','sliders',0,255,nothing)
cv2.createTrackbar('LV','sliders',0,255,nothing)
cv2.createTrackbar('UV','sliders',0,255,nothing)
#################print RGB ranges



while(1):
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    LH = cv2.getTrackbarPos('LH','sliders')
    UH = cv2.getTrackbarPos('UH','sliders')
    LS = cv2.getTrackbarPos('LS','sliders')
    US = cv2.getTrackbarPos('US','sliders')
    LV = cv2.getTrackbarPos('LV','sliders')
    UV = cv2.getTrackbarPos('UV','sliders')
    
    lower_red = np.array([LH,LS,LV])
    upper_red = np.array([UH,US,UV])
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
