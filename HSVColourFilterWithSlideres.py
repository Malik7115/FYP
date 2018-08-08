import cv2
import numpy as np

def nothing(x):
    pass 

cap = cv2.VideoCapture(1)
cv2.namedWindow('sliders')
balls = cv2.imread('balls2.jpeg',1)

cv2.createTrackbar('LH','sliders',0,255,nothing)
cv2.createTrackbar('UH','sliders',0,255,nothing)
cv2.createTrackbar('LS','sliders',0,255,nothing)
cv2.createTrackbar('US','sliders',0,255,nothing)
cv2.createTrackbar('LV','sliders',0,255,nothing)
cv2.createTrackbar('UV','sliders',0,255,nothing)
#################print RGB ranges
kernel = np.ones((5,5),np.uint8)


while(1):
    _, frame = cap.read()
    
    
    
    hsv = cv2.cvtColor(balls, cv2.COLOR_BGR2HSV)
#     
    h,s,v = cv2.split(frame)
    
    frame = cv2.resize(frame,(340,220))
    
    cv2.imshow('orig',frame)
    
    
    LH = cv2.getTrackbarPos('LH','sliders')
    UH = cv2.getTrackbarPos('UH','sliders')
    LS = cv2.getTrackbarPos('LS','sliders')
    US = cv2.getTrackbarPos('US','sliders')
    LV = cv2.getTrackbarPos('LV','sliders')
    UV = cv2.getTrackbarPos('UV','sliders')
    
    h = cv2.inRange(h,133,239)
    s = cv2.inRange(s,0,255)
    v = cv2.inRange(v,23,98)
    
    
    
    
    
    total = cv2.bitwise_and(h,s)
    total = cv2.bitwise_and(total,v)
    opening = cv2.morphologyEx(total, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(total, cv2.MORPH_CLOSE, kernel)
    opening = cv2.resize(opening,(340,220))
    total = cv2.resize(total,(340,220))
    closing = cv2.resize(closing,(340,220))
    
    
    _, contours, _ = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow('op',opening)
    cv2.imshow('clos',closing)
    cv2.drawContours(frame,contours,-1,(0,0,255),1)
    
    res = cv2.bitwise_and(frame,frame,mask=opening)
    cv2.imshow('res',res)

#     threshH = cv2.inRange(h,LH,UH)
#     
#     lower_red = np.array([LH,LS,LV])
#     upper_red = np.array([UH,US,UV])
#     
#     mask = cv2.inRange(hsv, lower_red, upper_red)
#     res = cv2.bitwise_and(frame,frame, mask= mask)
# 
    
#     cv2.imshow('H',threshH)
#     cv2.imshow('res',res)
#     
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
            


cv2.destroyAllWindows()
cap.release()
