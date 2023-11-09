
from typing import List
from .rotating_caliper import BoundingBox2D
import numpy as np
import matplotlib.pyplot as plt
import math
from viam.proto.common import GeometriesInFrame, Geometry, PointCloudObject


def plot_point_cloud(X, Y):
    # Create a scatter plot of the points
    plt.scatter(X, Y, s=10, c='b', marker='o', label='Point Cloud')

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Point Cloud Plot')

    # Add a legend
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.show()

def create_projection_from_list(points: np.ndarray):
    number_of_points = points.shape[0]
    X = []
    Y =[]
    for i in range(number_of_points):
        X.append(points[i][0])
        Y.append(points[i][1])
    return X, Y

def normalize_pcd(X,Y):
    maxi = max(abs(X.max()), abs(X.min()), abs(Y.max()), abs(Y.min()))
    print(f"maxi is {maxi}")
    print(f"X shape is {X.shape}")
    for i in range(X.shape[0]):
        X[i] = X[i]/maxi
        Y[i] = Y[i]/maxi
    return X, Y
        
    
    
def plot_inliers_and_outliers(X, Y, inlier_mask=None):
    X =X.flatten()
    outlier_mask = np.logical_not(inlier_mask)
    plt.scatter(
    X[inlier_mask], Y[inlier_mask], color="yellowgreen", marker=".", label="Inliers"
    )
    plt.scatter(
        X[outlier_mask], Y[outlier_mask], color="gold", marker=".", label="Outliers"
    )
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.show() 
    
    
def rotate_rectangle(vertices, angle_radians):
    # Find the center of the rectangle
    center_x = (vertices[0][0] + vertices[2][0]) / 2
    center_y = (vertices[0][1] + vertices[2][1]) / 2

    # Translate the rectangle to the origin
    translated_vertices = [(x - center_x, y - center_y) for x, y in vertices]

    # Rotate each vertex by the given angle
    rotated_vertices = []
    for x, y in translated_vertices:
        new_x = x * math.cos(angle_radians) - y * math.sin(angle_radians)
        new_y = x * math.sin(angle_radians) + y * math.cos(angle_radians)
        rotated_vertices.append((new_x, new_y))

    # Translate the rotated rectangle back to its original position
    final_vertices = [(x + center_x, y + center_y) for x, y in rotated_vertices]

    return final_vertices

def plot_geometry(geo:Geometry):
    A = [geo.center.x+geo.box.dims_mm.x/2, geo.center.y-geo.box.dims_mm.y/2]
    B = [geo.center.x-geo.box.dims_mm.x/2, geo.center.y-geo.box.dims_mm.y/2]
    C =  [geo.center.x-geo.box.dims_mm.x/2, geo.center.y+geo.box.dims_mm.y/2]
    D =  [geo.center.x+geo.box.dims_mm.x/2, geo.center.y+geo.box.dims_mm.y/2]
    li = [A,B, C, D]
    li2 = rotate_rectangle(li, geo.center.theta)
    li3 = [np.array(v) for v in li2]
    poly3 = BoundingBox2D(li3)
    poly3.plot()
    
    
def geometries_to_pointcloud_objects(geometries: List[Geometry], ref_frame:str) -> List[PointCloudObject]:
    res =[]
    for geo in geometries:
        geometry_in_frame = GeometriesInFrame(reference_frame=ref_frame, geometries=[geo])
        pcd_object = PointCloudObject(point_cloud=b'0', geometries=geometry_in_frame)
        res.append(pcd_object)
    return res


def set_pyplot_style():
    plt.style.use("dark_background")
    plt.grid(True, color='gainsboro', linestyle='-', linewidth=.1)
    # 
    plt.xlim(-0.5, 0.6)
    plt.ylim((-.5, 0.6))
    plt.axis("square")
    # plt.axis('equal')
    plt.xticks(fontsize=4)
    plt.yticks(fontsize=4)
    
    
