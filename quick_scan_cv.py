import cv
import cv2
import numpy as np
import math
import time
from os import listdir
ballTargetImgIndex = 0

def quick_scan_cv(configs, autonomyToCV, gcs_timestamp, connection_timestamp):
    print("Starting Quickscan CV")

    #Set output image folder based on simulation attribute in configs
    out_imagef_path = configs["quick_scan_specific"]["quick_scan_images"]
    cam = None
    if not configs['cv_simulated']['toggled_on']:
        cam = cv.init_camera(configs)
        if cam is None: 
            print("Camera not found")

    rad_threshold = configs["quick_scan_specific"]["rad_threshold"] # between 2 and 3 degrees
    all_kpts = []   #list of list of keypoints/descriptors (each list is an image)
    all_desc = []
    image_ctr = 0       #counter for saving images

    #Poll to ensure start is set to True
    initial_start = get_autonomy_start_and_stop(autonomyToCV)[0]
    while (initial_start == False):
        if get_autonomy_start_and_stop(autonomyToCV)[0]:
            break

    #Retreive images and store onto disk
    print("\nBeginning to take and store images")
    while (get_autonomy_start_and_stop(autonomyToCV) == (True, False)):
        #Get image via simulation; TODO: get image via vehicle
        if configs['cv_simulated']['toggled_on']:
            #simulation image
            img = cv.cv_simulation(configs) 
        else: 
            #image taken from camera
            if cam is not None:
                img = take_picture(cam)
            else:
                break
            
        pitch, roll = get_autonomytoCV_vehicle_angle(autonomyToCV)

        isBall, isTarget = isBallorTarget(img, configs, True)
        
        autonomyToCV.xbeeMutex.acquire()
        if autonomyToCV.xbee:
            if (isBall or isTarget):
                lat, lon = get_autonomyToCV_location(autonomyToCV)
                print("POI @ " + str(lat) + ", " + str(lon))
                poi_message = {
                          "type": "poi",
                          "id": 0,
                          "sid":  configs["vehicle_id"],
                          "tid": 0,
                          "time": round(time.clock() - connection_timestamp) + gcs_timestamp,

                          "lat": lat,                   # Latitude of point of interest
                          "lng": lon,                   # Longitude of point of interest
                        }

                # Instantiate a remote XBee device object to send data.
                address = configs["mission_control_MAC"]
                xbee = autonomyToCV.xbee
                send_xbee = RemoteXBeeDevice(xbee, address)
                xbee.send_data(send_xbee, json.dumps(poi_message))
        autonomyToCV.xbeeMutex.release()


        #determines whether to save image to output folder based on angle of vehicle or simulation
        '''
        if (abs(pitch) < rad_threshold and
                abs(roll)  < rad_threshold and
                configs['cv_simulated']['toggled_on'] == False):
                '''

        if not configs['cv_simulated']['toggled_on']:
            image_out =  out_imagef_path + str(image_ctr) + ".jpg"
            cv2.imwrite(image_out, img)
            image_ctr += 1

    #Get keypoints and discriptors of all images
    print("Getting images for processing")
    files = listdir(out_imagef_path)
    for fname in files:
        path = out_imagef_path + fname
        img = cv2.imread(path, 0)   #color for now
        kpt, desc = feature_keypts(img)
        all_kpts.append(kpt)
        all_desc.append(desc)

    #TODO: return list of keypoints (with new coordinates) and descriptors
    #for now: None
    return None #stitch_keypoints(kpts_list=all_kpts, descs_list=all_desc)

def take_picture(camera):
    try:
        count = camera.get(cv2.CAP_PROP_FRAME_COUNT)
        camera.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
        camera.grab()
        _, img = camera.retrieve()
        crop_img = img[78:630, 270:1071]
        return crop_img
    except KeyboardInterrupt:
        raise
    except:
        # Try taking the picture again
        time.sleep(1)
        take_picture(camera)


def get_autonomy_start_and_stop(autonomyToCV):
    autonomyToCV.startMutex.acquire()
    start = autonomyToCV.start
    autonomyToCV.startMutex.release()

    autonomyToCV.stopMutex.acquire()
    stop = autonomyToCV.stop
    autonomyToCV.stopMutex.release()

    return start, stop

def get_autonomytoCV_vehicle_angle(autonomyToCV):
    autonomyToCV.vehicleMutex.acquire()
    vehicle = autonomyToCV.vehicle
    pitch = vehicle.attitude.pitch
    roll = vehicle.attitude.roll
    autonomyToCV.vehicleMutex.release()
    return pitch, roll

def get_autonomytoCV_location(autonomyToCV):
    autonomyToCV.vehicleMutex.acquire()
    location = autonomyToCV.vehicle.location.global_frame
    lat = location.lat
    lon = location.lon
    autonomyToCV.vehicleMutex.release()
    return lat, lon




def feature_keypts(img):
    orb = cv2.ORB_create(nfeatures=1000)
    kp, desc = orb.detectAndCompute(img, None)

    return kp, desc

def feature_match(kp1, desc1, kp2, desc2, testing=False):
    #FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=100)   # or pass empty dictionary

    #Matching with FLANN feature match
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    if desc1.dtype != 'float32':
        desc1 = np.float32(desc1)
    if desc2.dtype != 'float32':
        desc2 = np.float32(desc2)

    matches = flann.knnMatch(desc1, desc2, k=2)
    matchesMask = [[0, 0] for i in xrange(len(matches))]

    no_good_matches = 0
    good_matches = []

    #Using 2 matches and "ratio test as per Lowe's paper"
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.75*n.distance:
            no_good_matches += 1
            good_matches.append(m)
            matchesMask[i]=[1,0]

    print("Number of good matches: " + str(no_good_matches))

    if testing:
        #All matches and masks to produce image
        return matches, matchesMask
    return good_matches


def stitch_keypoints(kpts_list, descs_list):
    #Use initial image as base case
    if len(kpts_list) > 0:
        final_kp = kpts_list[0]
        final_desc = descs_list[0]
    else:
        return kpts_list, descs_list

    #Iterate through each image
    for img_no in range(1, len(kpts_list)):
        img_kp = kpts_list[img_no]
        img_desc = descs_list[img_no]

        matches = feature_match(final_kp, final_desc, img_kp, img_desc)

        points1 = np.zeros((len(matches), 2), dtype=np.float32)
        points2 = points1.copy()

        for i, match in enumerate(matches):
            points1[i,:] = final_kp[match.queryIdx].pt
            points2[i,:] = img_kp[match.trainIdx].pt

        h, mast = cv2.findHomography(points1, points2, cv2.RANSAC)
        points2 = np.array([points2])
        #result_coordinates = cv2.perspectiveTransform(points2, h)
        #print("original ")
        #print(points2)
        #print("transformed " )
        #print(result_coordinates)

    #TODO: Match transformed points to original and stitch
    return None


def stitch_image(img_list):
    stitcher = cv2.Stitcher_create()
    status, stitched = stitcher.stitch(img_list)
    print("Stitch status: " + str(status))
    if status == 0:
        cv2.imwrite("cv_stitched_map.jpg", stitched)

def isBallorTarget(img_rgb, configs, see_results=False):
    global ballTargetImgIndex

    ballTargetImgIndex += 1
    img_rgb = cv2.GaussianBlur(img_rgb, (3, 3), 2)
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,100,100])
    upper_red = np.array([15,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([160,100,100])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask = mask0 + mask1
    output_img = img_rgb.copy()
    output_img[np.where(mask==0)] = 0
    output_img[np.where(mask!=0)] = 255
    

    output_gray = cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)
    target_circles = cv2.HoughCircles(output_gray,cv2.HOUGH_GRADIENT,1.2,.01,
                            param1=100,param2=50,minRadius=0,maxRadius=0)

    ball_circles = cv2.HoughCircles(output_gray,cv2.HOUGH_GRADIENT,1.3,10,
                            param1=30,param2=43,minRadius=0,maxRadius=0)

    #for smaller ball images, let restrictions be smaller
    if ball_circles is None:
        ball_circles = cv2.HoughCircles(output_gray,cv2.HOUGH_GRADIENT,1.3,10,
                            param1=30,param2=25,minRadius=0,maxRadius=0)

    isBall = False
    isTarget = False
    center_buffer = 8
    rbuffer = 1
    target_pair = None
    ball_pair = None

    if target_circles is not None:
        circles = np.uint16(np.around(target_circles))
        #Looks for a target
        for i, c1 in enumerate(circles[0,:]):
            for c2 in circles[0, i:]:
                if ( (( c1[0] - center_buffer) <= c2[0] <= (c1[0]) + center_buffer) and
                    (( c1[1] - center_buffer) <= c2[1] <= (c1[1]) + center_buffer) and
                    (((1.5 - rbuffer) * c1[2]) <= c2[2] <= ((1.5 + rbuffer) * c1[2]))
                    ):
                    isTarget = True
                    target_pair = (c1, c2)
                if isTarget:
                    break
            if isTarget:
                break

    if ball_circles is not None:
        if len(ball_circles[0]) == 1 or not isTarget:
            isBall = True
            ball_pair = (ball_circles[0][0],)
        elif len(ball_circles[0]) != 1:
            circles = np.uint16(np.around(ball_circles))
            for i, c1 in enumerate(circles[0,:]):
                for c2 in circles[0, i:]:
                    if (( 0 < xy_distance(c1[0], c2[0], c1[1], c2[1]) > (c1[2] + c2[2]) and
                        not isBall) or not isTarget
                        ):
                        isBall = True
                        ball_pair = (c1, c2)

                    if isBall:
                        break
                if isBall:
                    break

    if see_results:
        #print(ball_pair)
        #print(target_pair)
        #print(ball_circles)
        #print(target_circles)

        if target_pair is not None:
            # draw circles for Target Pair
            c1 = target_pair[0]
            cv2.circle(img_rgb,(c1[0],c1[1]),c1[2],(255,0,0),10)
            # draw the center of the circle
            cv2.circle(img_rgb,(c1[0],c1[1]),2,(255,0, 0),10)

            # draw the outer circle
            c2 = target_pair[1]
            cv2.circle(img_rgb,(c2[0],c2[1]),c2[2],(255,0,0),10)
            # draw the center of the circle
            cv2.circle(img_rgb,(c2[0],c2[1]),2,(255,0,0),10)


        if target_circles is not None:
            circles = np.uint16(np.around(target_circles))
            #Looks for a target
            for i, c1 in enumerate(circles[0,:]):
                cv2.circle(img_rgb,(c1[0],c1[1]),c1[2],(255,0,0),10)
                # draw the center of the circle
                cv2.circle(img_rgb,(c1[0],c1[1]),2,(255,0,0),3)


        if ball_pair is not None:
            # draw circles for Ball Pair
            c1 = ball_pair[0]
            cv2.circle(img_rgb,(c1[0],c1[1]),c1[2],(255,0,0),10)
            # draw the center of the circle
            cv2.circle(img_rgb,(c1[0],c1[1]),2,(255,0,0),6)
            if len(ball_pair) > 1:
                # draw the outer circle
                c2 = ball_pair[1]
                cv2.circle(img_rgb,(c2[0],c2[1]),c2[2],(255,0,0),10)
                # draw the center of the circle
                cv2.circle(img_rgb,(c2[0],c2[1]),2,(255,0,0),6)

        if ball_circles is not None:
            circles = np.uint16(np.around(ball_circles))
            #Looks for a target
            for i, c1 in enumerate(circles[0,:]):
                cv2.circle(img_rgb,(c1[0],c1[1]),c1[2],(255,0,0),10)
                # draw the center of the circle
                cv2.circle(img_rgb,(c1[0],c1[1]),2,(255,0,0),3)
        #if isBall or isTarget:
        display_img = cv2.resize(img_rgb, None, fx=0.9, fy=0.9)
        print("Ball: " + str(isBall))
        print("Target: " + str(isTarget))
        if configs['quick_scan_specific']['demo']:
            cv2.imshow("output", display_img)
            cv2.waitKey(500)
    return isBall, isTarget

def xy_distance(x1, x2, y1, y2):
    np.seterr('ignore')
    return (((x1 - x2) ** 2) + ((y1 - y2) ** 2)) ** 0.5


if __name__ == '__main__':
    directory = "./quick_scan_images/"
    files = listdir(directory)
    configs = {}
    d = {'demo': True}
    configs['quick_scan_specific'] = d
    for f in files:
        img = cv2.imread(directory + f)
        isBall, isTarget = isBallorTarget(img, configs, True)
        if isBall or isTarget:
            cv2.imwrite("./pomona_results/" + f, img)
        #print(f + ": " + str(isBall) + ", " + str(isTarget))
    
