'''x, y data structure. Currently, points are handled as [x, y] lists'''
from rtree import index

class XYSortedList:
    '''Data structure class for storing and range searching of x, y values'''
    def __init__(self):
        self.r_tree = index.Index()

    def insert(self, point, id=0):
        '''Inserts an x, y point. point should be indexable with x in point[0] and y in point[1]'''
        x_val = point[0]
        y_val = point[1]
        self.r_tree.insert(id, (x_val, y_val, x_val, y_val), obj=point.copy())

    def poll_range(self, x_start, x_end, y_start, y_end):
        '''Returns a list of points in x range from x_start to x_end inclusive
        and y range from y_start to y_end inclusive'''
        return list(self.r_tree.intersection((x_start, y_start, x_end, y_end), objects=True))

    def remove(self, point, id=0):
        '''Removes the specified point'''
        x_val = point[0]
        y_val = point[1]
        self.r_tree.delete(id, (x_val, y_val, x_val, y_val))
