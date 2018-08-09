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
    
    
    
    
    frame = cv2.resize(frame,(340,220))
    
    
    
    
    LH = cv2.getTrackbarPos('LH','sliders')
    UH = cv2.getTrackbarPos('UH','sliders')
    LS = cv2.getTrackbarPos('LS','sliders')
    US = cv2.getTrackbarPos('US','sliders')
    LV = cv2.getTrackbarPos('LV','sliders')
    UV = cv2.getTrackbarPos('UV','sliders')
    
    b,g,r = cv2.split(frame)
    _,b = cv2.threshold(b,UH,255,cv2.THRESH_BINARY_INV)
    _,r = cv2.threshold(r,LH,255,cv2.THRESH_BINARY)
    _,g = cv2.threshold(g,LS,255,cv2.THRESH_BINARY_INV)
    
    img, contours, hierarchy = cv2.findContours(r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    print(contours)
    if len(contours) != 0:
        output = r.copy()
        cv2.drawContours(frame, contours, -1, (0,255,0), 2)
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
         
    cv2.imshow('r',r)
    cv2.imshow('res',frame)
    
    
    
     
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
            


cv2.destroyAllWindows()
cap.release()
