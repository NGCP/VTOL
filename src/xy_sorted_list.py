'''x, y data structure. Currently, points are handled as [x, y] lists'''
import numpy as np

class XYSortedList:
    '''Data structure class for storing and range searching of x, y values'''
    def __init__(self):
        self.x_sorted_list = np.array(dtype=np.float64)
        self.y_values = {}

    def insert(self, point):
        '''Inserts an x, y point. point should be indexable with x in point[0] and y in point[1]'''
        x_val = point[0]
        y_val = point[1]
        idx = self.x_sorted_list.searchsorted(x_val)
        np.concatenate((self.x_sorted_list[:idx], x_val, self.x_sorted_list[idx:]))
        self.y_values.setdefault(x_val, []).append(y_val)

    def poll_range(self, x_start, x_end, y_start, y_end):
        '''Returns a list of points in x, y range.'''
        pass
