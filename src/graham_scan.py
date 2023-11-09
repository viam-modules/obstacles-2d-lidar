import matplotlib.pyplot as plt
from .pointcloud.point_cloud import PlanarPointCloud

class GrahamScan:
    '''
    self.convex_hull is the convex hull as list of points in the normalized 2d axis coordinates'''
    def __init__(self, ppc: PlanarPointCloud):
        self.points = ppc.points_norm.tolist()
        self.convex_hull = self.compute_convex_hull()

    def get_cross_product(self, p1, p2, p3):
        return ((p2[0] - p1[0]) * (p3[1] - p1[1])) - ((p2[1] - p1[1]) * (p3[0] - p1[0]))

    def get_slope(self, p1, p2):
        if p1[0] == p2[0]:
            return float('inf')
        else:
            return 1.0 * (p1[1] - p2[1]) / (p1[0] - p2[0])

    def compute_convex_hull(self):
        hull = []
        self.points.sort(key=lambda x: [x[0], x[1]])
        start = self.points.pop(0)
        hull.append(start)
        self.points.sort(key=lambda p: (self.get_slope(p, start), -p[1], p[0]))
        for pt in self.points:
            hull.append(pt)
            while len(hull) > 2 and self.get_cross_product(hull[-3], hull[-2], hull[-1]) < 0:
                hull.pop(-2)
        return hull
    
    def plot(self):
        points = self.convex_hull
        x, y = zip(*points)
        x += x[:1]
        y += y[:1]
        plt.plot(x, y, marker=None, linestyle='dashed')
  