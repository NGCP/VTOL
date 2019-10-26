'''Test for XY data structure'''
from xy_sorted_list import XYSortedList

def test_insert():
    '''inserts points'''
    data = XYSortedList()
    point1 = [1, 1]
    data.insert(point1)
    point2 = [2, 2]
    data.insert(point2)

def test_poll_range():
    '''Inserts points and checks if the right points are returned in poll_range'''
    data = XYSortedList()
    points = [[1.5, 2.5], [0, 1], [1, 1.5], [2.5, 3]]
    for point in points:
        data.insert(point)
    poll1 = data.poll_range(0, 2.5, 1, 2.5)
    assert points[0] in poll1
    assert points[1] in poll1
    assert points[2] in poll1
    assert points[3] not in poll1

def test_remove():
    '''Checks if points are removed'''
    data = XYSortedList()
    points = [[1.5, 2.5], [0, 1], [2.5, 3]]
    for point in points:
        data.insert(point)
    poll1 = data.poll_range(0, 2.5, 1, 2.5)
    assert points[0] in poll1
    data.remove(points[0])
    poll2 = data.poll_range(0, 2.5, 1, 2.5)
    assert points[0] not in poll2
