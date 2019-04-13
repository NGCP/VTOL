import cv
import cv2
import numpy as np
from os import listdir

def quick_scan_cv(configs, autonomyToCV):
    print("Starting Quickscan CV")

    #Set output image folder based on simulation attribute in configs
    out_imagef_path = configs["quick_scan_specific"]["quick_scan_images"]
    if configs['cv_simulated']['toggled_on'] == True:
        out_imagef_path = configs['cv_simulated']["directory"]

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
        img = cv.cv_simulation(configs) #get image function
        pitch, roll = get_autonomytoCV_vehicle_angle(autonomyToCV)

        #determines whether to save image to output folder based on angle of vehicle or simulation
        if (abs(pitch) < rad_threshold and
                abs(roll)  < rad_threshold and
                configs['cv_simulated']['toggled_on'] == False):

            cv2.imwrite(out_imagef_path + str(i) + ".jpg", img)
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
    return stitch_keypoints(kpts_list=all_kpts, descs_list=all_desc)


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

    # Using 1 match and applying Threshold
    #for i, m in enumerate(matches):
    #    if m[0].distance < 200:
    #        no_good_matches += 1
    #        good_matches.append(m[0])
    #        matchesMask[i] = [1]

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
        result_coordinates = cv2.perspectiveTransform(points2, h)
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

if __name__ == '__main__':
    import sys
    from quick_scan import *
    quick_scan_cv(parse_configs(sys.argv), QuickScanAutonomyToCV())
