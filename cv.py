import cv2 
import numpy as np

def find_target(img): 
    red_center = find_red(img)
    white_center = find_white(img)

    if len(red_center) == 0 or len(white_center) == 0:
        print("no target")
        return []

    if red_center[0]-white_center[0] < 10 and red_center[1]-white_center[1] < 10:
        center = (int(red_center[0]+white_center[0])/2 , int(red_center[1]+white_center[1])/2)
        print(center)
        return center
    # else throw error? 

def isolate_target(img):
    img = cv2.resize(img,(0,0),fx=.25, fy=.25)

    # Split the image into 3 channels: b,g,r to view them individually
    # We then want to search for white spots in a channel since that is the isolated color,

    b,g,r = cv2.split(img)

    # Threshold the image, keeping frames that are closer to white (a value of 255)
    # Thresholding the individual color channel allows us to cut out anything below
    # a specific value on the grayscale spectrum.
    ret,blue_thresh = cv2.threshold(b,155,255,cv2.THRESH_BINARY_INV)
    ret,red_thresh = cv2.threshold(r,190,255,cv2.THRESH_BINARY)
    ret,green_thresh = cv2.threshold(g,155,255,cv2.THRESH_BINARY)

    #subtracting threshes to remains with only red and remove noise
    masked = red_thresh-green_thresh
    masked = cv2.medianBlur(masked,5)
    return img,masked 

def find_red(img):

    img,masked = isolate_target(img)

    circles = cv2.HoughCircles(masked,cv2.HOUGH_GRADIENT,2.0,masked.shape[0]/2,
                               param1=100,param2=10,minRadius=10,maxRadius=70)

    # return empty list if no POI found
    if circles is None:
        #print("no Target")
        return []

    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)

    # create a list of locations not including radii:
    return_list = circles.tolist()[0]
    for loc in return_list:
        del loc[2]   

    return return_list[0]

def find_white(img):
    img,masked = isolate_target(img)

    #invert to isolate the white part of the target
    masked = cv2.bitwise_not(masked)

    #gcircs=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    ret,thresh = cv2.threshold(masked,230,255,cv2.THRESH_BINARY)
    
    a,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    largest_contour = None
    c_list = [] #no quadrilaterals or segments
    m_dist = [] #max distance of furthest contour point from contour's center
    for contour in contours:
    
        if len(contour)>=20: #I get every x & y val of each contour point
            xs = []
            ys = []
            ds = []
            for point in contour:
                x = point[0][0]
                y = point[0][1]
                xs.append(x)
                ys.append(y)
            x_bar = int(sum(xs)/len(xs))
            y_bar = int(sum(ys)/len(ys))
            center = (x_bar, y_bar) # center of contour
            for point in contour:
                d = ((point[0][0]-center[0])**2+(point[0][1]-center[1])**2)**.5
                ds.append(d) # a list of distances of points from their center
            avg = sum(ds)/len(ds)
            m_dist.append(max(ds)-avg) # the distance of the furthest contour point from the avg distance from the center
            c_list.append(contour) # c_list is all contours with >50 points
        max_distance = 0
        if len(m_dist) != 0:
            max_distance = max(m_dist)

    for i,c in enumerate(c_list):
        if m_dist[i] == max_distance: #contour with max distance
            largest_contour = c
    
    original = img.copy() #I want the unmarked original img
    if largest_contour is not None:
        cv2.drawContours(img,[largest_contour],-1,(0,255,0),-1) #marks img w/ target
        #print(center)
        return center
    else:
        #print('no white')
        return []

def find_POI(img): #find_POI(img, vehicle_position):
   
    location_list = []

    location_list.extend(find_target(img))
    #location_list.extend(find_obstacles(img))       #TODO
    #location_list.extend(find_persons(img))         #Kenneth is working on this

    return location_list

    """One purpose of the CV thread is to find points of interest (POI) (obstacles, person, bullseye) in real time. 
    xx To accomplish this, the CV thread will take a photo and pass the photo and the vehicle's position as an argument to this function.

    The function will analyze the current image and return the position of any POI detected. 
    If there are any POI detected, return a list of tuple positions of the obstacle locations. If there are no POI found in an image, return an empty list.

    Write this function in cv.py and include at least 3 unit test cases (multiple, single, and no POI)."""

