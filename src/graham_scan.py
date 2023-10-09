import random
import matplotlib.pyplot as plt
import numpy as np
class ConvexHull:
    def __init__(self, points):
        if not points:
            self.points = [(random.randint(0, 100), random.randint(0, 100)) for i in range(50)]
        else:
            self.points = points
        self.points = self.compute_convex_hull()

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
        points = self.points
        x, y = zip(*points)

        # Close the polygon by adding the first point at the end
        x += x[:1]
        y += y[:1]

        # Plot the polygon
        plt.plot(x, y, marker='o', linestyle='dashed')

        # Add labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Polygon Plot')
        # plt.show()
        
    def translate_convex_hull_in_first_quadrant(self):
        points = np.array(self.points)
        min_x = np.min(points[:,0])
        if min_x <0:
            self.min_x = min_x
            points[:, 0] -= 3*min_x
        min_y = np.min(points[:,1])
        if min_y <0:
            self.min_y = min_y
            points[:, 1] -= 3*min_y

        return points.tolist()


# Example usage:
# ch = ConvexHull([(1, 2), (3, 4), (5, 6)])
# ch.plot_convex_hull()
# plt.show()
