import cv2
import numpy as np


def nothing(x):
    pass 


cap = cv2.VideoCapture(1)

cv2.namedWindow('sliders')
cv2.createTrackbar('LH','sliders',150,255,nothing)

check = 0

while(1):
    ret,frame = cap.read()
    LH = cv2.getTrackbarPos('LH','sliders')
    b,g,r = cv2.split(frame)
    _,r = cv2.threshold(r,253,255,cv2.THRESH_BINARY)
    
    
    connectivity = 8  
    # Perform the operation
    output = cv2.connectedComponentsWithStats(r, connectivity, cv2.CV_32S)
    # Get the results
    # The first cell is the number of labels
    num_labels = output[0]
    # The second cell is the label matrix
    labels = output[1]
    # The third cell is the stat matrix
    stats = output[2]
    # The fourth cell is the centroid matrix
    centroids = output[3]
    
    largest_area = stats[1,cv2.CC_STAT_AREA]
    largest_label = 0
    loop = np.unique(labels)
    
    #############################CCl part
    largest_area = 0

    mask = np.zeros_like(r)   

    for i in loop[1:]:
        
        area = stats[i,cv2.CC_STAT_AREA]
        if area > largest_area:
            largest_area = area;
            largest_label = i;
            
        mask[labels == largest_label] = i
            
        
        
    _,mask = cv2.threshold(mask,largest_label,255,cv2.THRESH_BINARY)
    r = cv2.bitwise_and(r,r,mask = mask)        
    w = stats[largest_label,cv2.CC_STAT_WIDTH]
    h = stats[largest_label,cv2.CC_STAT_HEIGHT]
    c = centroids[largest_label,:]
    sx = int(centroids[largest_label,0] - w/2)
    sy = int(centroids[largest_label,1] - h/2)
    ex = int(centroids[largest_label,0] + w/2)
    ey = int(centroids[largest_label,1] + h/2)
    cv2.rectangle(frame,(sx,sy),(ex,ey),(255,255,0),2)

    
    
    
    
    cv2.imshow('red',r)
    cv2.imshow('res',frame)
    
    
    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
    
    
