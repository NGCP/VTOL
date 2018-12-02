import sys
sys.path.append("../")
import numpy.testing as npt
import cv2
from cv import find_target
import unittest

class TestStringMethods(unittest.TestCase):

    def test_find_target(self):
        # checks that actual (x,y) are within 15 of the expected (x,y)
        # 15 = abs(desired-actual) < 1.5 * 10**(-decimal), decimal = -1 
        img = cv2.imread("poiTestImages/target1.jpg",1)
        npt.assert_almost_equal(find_target(img), [383, 153], -1)

        img = cv2.imread("poiTestImages/target2.jpg")
        npt.assert_almost_equal(find_target(img), [371,  97], -1)

        img = cv2.imread("poiTestImages/target3.jpg")
        npt.assert_almost_equal(find_target(img), [443, 351], -1)

    def test_find_POI_None(self):
        img = cv2.imread("poiTestImages/noTarget.jpg")
        self.assertEqual(find_target(img), [])

        img = cv2.imread("poiTestImages/noTarget2.jpg")
        self.assertEqual(find_target(img), [])

        img = cv2.imread("poiTestImages/noTarget3.jpg")
        self.assertEqual(find_target(img), [])

    def test_find_POI_Multiple(self):
        pass
        # do not yet have a general POI detection algorithm
        # currently only have target detection

if __name__ == "__main__":
    unittest.main()




