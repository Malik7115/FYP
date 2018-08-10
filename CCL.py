import cv2
import numpy as np
import imutils
from numpy import int

def nothing(x):
    pass 

cap = cv2.VideoCapture(1)
balls = cv2.imread('connect.png',0)
orig = cv2.imread('connect.png',1)



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
cv2.imshow('res',thresh)

# cv2.rectangle(balls,\
#               (int(c[0,0]-w/2),int(c[0,1]-h/2)),\
#               (int(c[0,0]+w/2),int(c[0,1]-h/2)),\
#               (255,0,0),2)
    
for i in np.unique(labels):
    w = stats[i,cv2.CC_STAT_WIDTH]
    h = stats[i,cv2.CC_STAT_HEIGHT]
    c = centroids[i,:]
    
    sx = int(centroids[i,0] - w/2)
    sy = int(centroids[i,1] - h/2)
    ex = int(centroids[i,0] + w/2)
    ey = int(centroids[i,1] + h/2)
    
    print('ss')
    print(sx,sy,ex,ey)
    print(w,h)
    
    
    cv2.rectangle(orig,(sx,sy),(ex,ey),(255,0,0),2)
    
cv2.imshow('orig',orig)

# cv2.rectangle(balls,(centroids[]),) 
print(centroids.shape)
print(centroids[0,:])

cv2.waitKey(0)
cv2.destroyAllWindows()

