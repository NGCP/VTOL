'''x, y data structure. Points are handled as [x, y] lists'''
import numpy as np

class XYSortedList:
    '''Data structure for storing and range searching x, y points'''
    def __init__(self):
        self.x_sorted_list = []
        self.y_values = {}

    def insert(self, point):
        '''Inserts an x, y point. point should be indexable with x in point[0] and y in point[1]'''
        x_val = point[0]
        y_val = point[1]
        idx = np.searchsorted(self.x_sorted_list, x_val)
        self.x_sorted_list = self.x_sorted_list[:idx] + [x_val] + self.x_sorted_list[idx:]
        self.y_values.setdefault(x_val, []).append(y_val)

    def poll_range(self, x_start, x_end, y_start, y_end):
        '''Returns a list of points in x range from x_start to x_end inclusive
        and y range from y_start to y_end inclusive'''
        start_idx = np.searchsorted(self.x_sorted_list, x_start)
        end_idx = np.searchsorted(self.x_sorted_list, x_end)
        result = []
        for i in range(start_idx, end_idx+1):
            x_val = self.x_sorted_list[i]
            for y_val in self.y_values[x_val]:
                if y_start <= y_val <= y_end:
                    result.append([x_val, y_val])
        return result

    def remove(self, point):
        '''Removes the specified point. Returns True if point found and removed, False otherwise'''
        x_val = point[0]
        y_val = point[1]
        x_idx = np.searchsorted(self.x_sorted_list, x_val)
        if self.x_sorted_list[x_idx] != x_val:
            return False
        try:
            self.y_values[x_val].remove(y_val)
            return True
        except ValueError:
            return False
