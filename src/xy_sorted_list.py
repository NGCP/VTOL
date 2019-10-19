import numpy as np

class XYSortedList:
    def __init__(self):
        self.x_sorted_list = np.array(dtype=np.float64)
        self.y_values = {}

    # Insert an x, y point. point should be indexable with x in point[0] and y in point[1]
    def insert(self, point):
        x = point[0]
        y = point[1]
        idx = self.x_sorted_list.searchsorted(x)
        np.concatenate((self.x_sorted_list[:idx], x, self.x_sorted_list[idx:]))
        self.y_values[x] = y

    
