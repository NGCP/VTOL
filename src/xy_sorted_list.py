'''x, y data structure. Currently, points are handled as [x, y] lists'''
import numpy as np

class XYSortedList:
    def __init__(self):
        self.x_sorted_list = np.array()
        self.y_values = {}

    # Insert an x, y point. point should be indexable with x in point[0] and y in point[1]
    def insert(self, point):
        x_val = point[0]
        y_val = point[1]
        idx = self.x_sorted_list.searchsorted(x_val)
        np.concatenate((self.x_sorted_list[:idx], x, self.x_sorted_list[idx:]))
        self.y_values.setdefault(x_val, []).append(y_val)

    # Returns a list of points in x, y range.
    def poll_range(self, x_start, x_end, y_start, y_end):
        '''Returns a list of points in x range from x_start to x_end inclusive
        and y range from y_start to y_end inclusive'''
        pass

    def remove(self, point, id=0):
        '''Removes the specified point'''
        x_val = point[0]
        y_val = point[1]
        pass
