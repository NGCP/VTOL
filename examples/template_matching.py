import cv2
import numpy as np
from math import sqrt, acos


MATCH_COUNT = 10

# :param full_image: path of the full image
# :param template_image: path of the image you want to find
def template_match(full_image, template_image):
    full = cv2.imread(full_image)
    crop = cv2.imread(template_image)
    img2 = cv2.cvtColor(full, cv2.COLOR_BGR2GRAY)
    img1 = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    # Initiate ORB detector
    orb = cv2.ORB_create(nfeatures=100000, scoreType=cv2.ORB_FAST_SCORE)

    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors.
    matches = bf.match(des1,des2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)[:MATCH_COUNT]

    if len(matches) >= MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        # matchesMask = mask.ravel().tolist()

        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        mp = cv2.perspectiveTransform(np.float32([[w/2.0, h/2.0]]).reshape(-1,1,2), M)[0][0]
        
        cv2.circle(img2, (mp[0], mp[1]), 5, 255, -1)
    else:
        raise Exception("Not enough matches! (minimum is %d matches)" % MATCH_COUNT)

    # Draw matches.
    img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 5, cv2.LINE_AA)
    img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=2)
    result = cv2.polylines(full, [np.int32(dst)], True, (255, 255, 255), 5, cv2.LINE_AA)
    cv2.circle(result, (mp[0], mp[1]), 2, (255, 255, 255), -1)
    cv2.circle(result, (mp[0], mp[1]), 10, (255, 255, 255), 2)

    print("Center Point: (%f, %f)" % (mp[0], mp[1]))

    # vector of upper edge
    vec = dst[3][0] - dst[0][0]

    # angle upper edge to x axis
    angle = acos(np.dot(vec, np.array([1, 0])) / (sqrt(vec[0]**2 + vec[1]**2)))    
    print("Angle: %f radians" % angle)

    # print("Corners:")
    # print("\n".join([str(i[0]) for i in dst]))

    # cv2.namedWindow("image", cv2.WINDOW_NORMAL)
    cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
    cv2.imshow('image', img3)
    cv2.waitKey(0)
    cv2.imshow('image', result)
    cv2.waitKey(0)

    # return ((mp[0], mp[1]), angle)


if __name__ == '__main__':
    full_image = "images/mainimage.jpg"    
    template_image = "images/template.jpg"
    template_match(full_image, template_image)