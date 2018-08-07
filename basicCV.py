'''
Created on Jul 17, 2018

@author: malik7115
'''



import numpy as np
import cv2 



def nothing(x):
    pass


print('start prog')


cap = cv2.VideoCapture(1)
cv2.namedWindow('orig',cv2.WINDOW_NORMAL)
cv2.resizeWindow('orig', 600,600)
cv2.namedWindow('th',cv2.WINDOW_NORMAL)
cv2.resizeWindow('th', 600,600)
cv2.namedWindow('webcam',cv2.WINDOW_NORMAL)
cv2.resizeWindow('webcam', 600,600)
cv2.namedWindow('H',cv2.WINDOW_NORMAL)
cv2.resizeWindow('H', 600,600)
cv2.namedWindow('S',cv2.WINDOW_NORMAL)
cv2.resizeWindow('S', 600,600)
cv2.namedWindow('V',cv2.WINDOW_NORMAL)
cv2.resizeWindow('V', 600,600)



img = cv2.imread('test.jpg',0)
print('image params:')
print (img.shape)

cv2.createTrackbar('low','orig',0,255,nothing)
cv2.createTrackbar('high','orig',0,255,nothing)

cv2.createTrackbar('lh','H',0,255,nothing)
cv2.createTrackbar('hh','H',0,255,nothing)

cv2.createTrackbar('ls','S',0,255,nothing)
cv2.createTrackbar('hs','S',0,255,nothing)

cv2.createTrackbar('lv','V',0,255,nothing)
cv2.createTrackbar('hv','V',0,255,nothing)


while(1):
    cv2.imshow('orig',img)
    ret,frame = cap.read()
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV_FULL)
    h,s,v = cv2.split(hsv)
    
    cv2.imshow('webcam',frame)
    cv2.imshow('H',h)
    cv2.imshow('S',s)
    cv2.imshow('V',v)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    l = cv2.getTrackbarPos('low','orig')
    h = cv2.getTrackbarPos('high','orig')
    
    
    
    ret,threshold = cv2.threshold(img,l,h,cv2.THRESH_BINARY)
    cv2.imshow('th',threshold)
    
    
cv2.destroyAllWindows()
    
    



############vid 
# while True:
#     ret,frame = cap.read()
#     cv.imshow('frame',frame)
#     hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV_FULL)
#     h,s,v = cv.split(hsv)
#     cv.imshow('H',h)
#     cv.imshow('S',s)
#     cv.imshow('V',v)
#     if cv.waitKey(20) & 0xFF == ord('q'):
#         break
#      
