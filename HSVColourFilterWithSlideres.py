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



while(1):
    _, frame = cap.read()
    hsv = cv2.cvtColor(balls, cv2.COLOR_BGR2HSV)
#     
    h,s,v = cv2.split(balls)
    cv2.imshow('balls',balls)
    
    
    LH = cv2.getTrackbarPos('LH','sliders')
    UH = cv2.getTrackbarPos('UH','sliders')
    LS = cv2.getTrackbarPos('LS','sliders')
    US = cv2.getTrackbarPos('US','sliders')
    LV = cv2.getTrackbarPos('LV','sliders')
    UV = cv2.getTrackbarPos('UV','sliders')
    
    h = cv2.inRange(h,LH,UH)
    s = cv2.inRange(s,LS,US)
    v = cv2.inRange(v,LV,UV)
    
    
    
    cv2.imshow('H',h)
    cv2.imshow('S',s)
    cv2.imshow('V',v)
    
    total = cv2.bitwise_and(h,s)
    total = cv2.bitwise_and(total,v)
    
    cv2.imshow('res',total)
    
    

#     threshH = cv2.inRange(h,LH,UH)
#     
#     lower_red = np.array([LH,LS,LV])
#     upper_red = np.array([UH,US,UV])
#     
#     mask = cv2.inRange(hsv, lower_red, upper_red)
#     res = cv2.bitwise_and(frame,frame, mask= mask)
# 
    cv2.imshow('frame',frame)
#     cv2.imshow('H',threshH)
#     cv2.imshow('res',res)
#     
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
            
im2,contours,heir = cv2.findContours(total,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(balls,contours,-1,(255,0,0),2)
cv2.imshow('balls',balls)
cv2.waitKey(0)
cv2.destroyAllWindows()
cap.release()
