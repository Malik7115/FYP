import cv2
import numpy as np
from numpy import dtype
from cv2 import bitwise_not



def nothing(x):
    pass 


################################# CCL Link:
#https://stackoverflow.com/questions/35854197/how-to-use-opencvs-connected-components-with-stats-in-python



cap = cv2.VideoCapture(1)
cv2.namedWindow('sliders')
balls = cv2.imread('connect.png',0)
orig = cv2.imread('connect.png',1)

cv2.createTrackbar('LH','sliders',0,255,nothing)
cv2.createTrackbar('UH','sliders',0,255,nothing)
cv2.createTrackbar('LS','sliders',0,255,nothing)
cv2.createTrackbar('US','sliders',0,255,nothing)
cv2.createTrackbar('LV','sliders',0,255,nothing)
cv2.createTrackbar('UV','sliders',0,255,nothing)
#################print RGB ranges
kernel = np.ones((5,5),np.uint8)


################
_,thresh = cv2.threshold(balls,0,255,cv2.THRESH_BINARY_INV)
cv2.namedWindow('res',cv2.WINDOW_AUTOSIZE)


connectivity = 8  
# Perform the operation
output = cv2.connectedComponentsWithStats(thresh, connectivity, cv2.CV_32S)
# Get the results
# The first cell is the number of labels
num_labels = output[0]
# The second cell is the label matrix
labels = output[1]
# The third cell is the stat matrix
stats = output[2]
# The fourth cell is the centroid matrix
centroids = output[3]

print((labels.shape))
cv2.imshow('thresh',thresh)

# cv2.rectangle(balls,\
#               (int(c[0,0]-w/2),int(c[0,1]-h/2)),\
#               (int(c[0,0]+w/2),int(c[0,1]-h/2)),\
#               (255,0,0),2)




largest_area = stats[1,cv2.CC_STAT_AREA]
largest_label = 0
loop = np.unique(labels)

new_img = np.zeros_like(thresh)   
for i in loop[1:]:
    mask = np.uint8(thresh == i)  
    w = stats[i,cv2.CC_STAT_WIDTH]
    h = stats[i,cv2.CC_STAT_HEIGHT]
    c = centroids[i,:]
    
   
    area = stats[i,cv2.CC_STAT_AREA]
    
    if area > largest_area:
        largest_area = area;
        largest_label = i;
        
    print('area')
    print(area)
    sx = int(centroids[i,0] - w/2)
    sy = int(centroids[i,1] - h/2)
    ex = int(centroids[i,0] + w/2)
    ey = int(centroids[i,1] + h/2)
    
    new_img[labels == largest_label] = i
    cv2.rectangle(orig,(sx,sy),(ex,ey),(255,0,0),2)


w = stats[largest_label,cv2.CC_STAT_WIDTH]
h = stats[largest_label,cv2.CC_STAT_HEIGHT]
c = centroids[largest_label,:]
sx = int(centroids[largest_label,0] - w/2)
sy = int(centroids[largest_label,1] - h/2)
ex = int(centroids[largest_label,0] + w/2)
ey = int(centroids[largest_label,1] + h/2)
cv2.rectangle(orig,(sx,sy),(ex,ey),(255,255,0),2)




print('largest area',largest_area)
print('largest label',largest_label)
cv2.imshow('orig',orig)
_,new_img = cv2.threshold(new_img,largest_label,255,cv2.THRESH_BINARY)

final = cv2.bitwise_and(orig,orig,mask = new_img)

cv2.imshow('final',new_img)

# cv2.rectangle(balls,(centroids[]),) 
# print(centroids.shape)
# print(centroids[0,:])

cv2.waitKey(0)
cv2.destroyAllWindows()

# while(1):
#     
#     
#     ret, frame = cap.read()
#     
#     
#     
#     
#     frame = cv2.resize(frame,(340,220))
#     
#     
#     
#     
#     LH = cv2.getTrackbarPos('LH','sliders')
#     UH = cv2.getTrackbarPos('UH','sliders')
#     LS = cv2.getTrackbarPos('LS','sliders')
#     US = cv2.getTrackbarPos('US','sliders')
#     LV = cv2.getTrackbarPos('LV','sliders')
#     UV = cv2.getTrackbarPos('UV','sliders')
#     
#     b,g,r = cv2.split(frame)
#     _,b = cv2.threshold(b,UH,255,cv2.THRESH_BINARY_INV)
#     _,r = cv2.threshold(r,LH,255,cv2.THRESH_BINARY)
#     _,g = cv2.threshold(g,LS,255,cv2.THRESH_BINARY_INV)
#     
#     img, contours, hierarchy = cv2.findContours(r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     
#     print(len(contours))
#     
#     if len(contours) != 0:
#         output = r.copy()
#         cv2.drawContours(frame, contours, -1, (0,255,0), 2)
#         c = max(contours, key = cv2.contourArea)
#         
#         M = cv2.moments(c)
#         cX = int(M["m10"] / M["m00"])
#         cY = int(M["m01"] / M["m00"])
#         cv2.circle(frame, (cX, cY), 3, (255, 0, 0), -1)
#         
#         
#         x,y,w,h = cv2.boundingRect(c)
#         cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,255),2)
#         
#     
#         
#     
#     cv2.imshow('r',r)
#     cv2.imshow('res',frame)
#     
#     
#     
#      
#     k = cv2.waitKey(5) & 0xFF
#     if k == 27:
#         break
#     
#             
# 
# 
# cv2.destroyAllWindows()
# cap.release()
