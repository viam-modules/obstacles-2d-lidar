# pylint: disable=invalid-name
import math
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from viam.proto.common import GeometriesInFrame, Geometry, PointCloudObject

from .rotating_caliper import BoundingBox2D


def plot_point_cloud(X, Y):
    """
    Plots a 2D point cloud.

    Args:
        X (np.ndarray): Array of x-coordinates of the points.
        Y (np.ndarray): Array of y-coordinates of the points.
    """
    plt.scatter(X, Y, s=10, c="b", marker="o", label="Point Cloud")

    # Add labels and title
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Point Cloud Plot")

    # Add a legend
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.show()


def create_projection_from_list(points: np.ndarray):
    """
    Creates separate lists of x and y coordinates from an array of points.

    Args:
        points (np.ndarray): Array of points where each point is a tuple (x, y).

    Returns:
        Tuple[List[float], List[float]]: Two lists containing the x and y coordinates respectively.
    """
    number_of_points = points.shape[0]
    X = []
    Y = []
    for i in range(number_of_points):
        X.append(points[i][0])
        Y.append(points[i][1])
    return X, Y


def normalize_pcd(X, Y):
    """
    Normalizes the point cloud coordinates to fit within the range [-1, 1].

    Args:
        X (np.ndarray): Array of x-coordinates of the points.
        Y (np.ndarray): Array of y-coordinates of the points.

    Returns:
        Tuple[np.ndarray, np.ndarray]: Normalized x and y coordinates.
    """
    maxi = max(abs(X.max()), abs(X.min()), abs(Y.max()), abs(Y.min()))
    print(f"maxi is {maxi}")
    print(f"X shape is {X.shape}")
    for i in range(X.shape[0]):
        X[i] = X[i] / maxi
        Y[i] = Y[i] / maxi
    return X, Y


def plot_inliers_and_outliers(X, Y, inlier_mask=None):
    """
    Plots inliers and outliers of a point cloud.

    Args:
        X (np.ndarray): Array of x-coordinates of the points.
        Y (np.ndarray): Array of y-coordinates of the points.
        inlier_mask (np.ndarray): Boolean array indicating inliers (True) and outliers (False).
    """
    X = X.flatten()
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
    """
    Rotates a rectangle by a given angle around its center.

    Args:
        vertices (List[Tuple[float, float]]): List of four vertices of the rectangle.
        angle_radians (float): Angle by which to rotate the rectangle in radians.

    Returns:
        List[Tuple[float, float]]: List of rotated vertices.
    """
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


def plot_geometry(geo: Geometry):
    """
    Plots the geometry of a rectangular prism.

    Args:
        geo (Geometry): Geometry object containing the center and
        dimensions of the rectangular prism.
    """
    A = [geo.center.x + geo.box.dims_mm.x / 2, geo.center.y - geo.box.dims_mm.y / 2]
    B = [geo.center.x - geo.box.dims_mm.x / 2, geo.center.y - geo.box.dims_mm.y / 2]
    C = [geo.center.x - geo.box.dims_mm.x / 2, geo.center.y + geo.box.dims_mm.y / 2]
    D = [geo.center.x + geo.box.dims_mm.x / 2, geo.center.y + geo.box.dims_mm.y / 2]
    li = [A, B, C, D]
    li2 = rotate_rectangle(li, geo.center.theta)
    li3 = [np.array(v) for v in li2]
    poly3 = BoundingBox2D(li3)
    poly3.plot()


def geometries_to_pointcloud_objects(
    geometries: List[Geometry], ref_frame: str
) -> List[PointCloudObject]:
    """
    Converts a list of Geometry objects to a list of PointCloudObject.

    Args:
        geometries (List[Geometry]): List of Geometry objects.
        ref_frame (str): Reference frame for the geometries.

    Returns:
        List[PointCloudObject]: List of PointCloudObject containing the geometries.
    """
    res = []
    for geo in geometries:
        geometry_in_frame = GeometriesInFrame(
            reference_frame=ref_frame, geometries=[geo]
        )
        pcd_object = PointCloudObject(point_cloud=b"0", geometries=geometry_in_frame)
        res.append(pcd_object)
    return res


def set_pyplot_style():
    """
    Sets the style for matplotlib plots.
    """
    plt.style.use("dark_background")
    plt.grid(True, color="gainsboro", linestyle="-", linewidth=0.1)
    #
    plt.xlim(-0.5, 0.6)
    plt.ylim((-0.5, 0.6))
    plt.axis("square")
    # plt.axis('equal')
    plt.xticks(fontsize=4)
    plt.yticks(fontsize=4)
