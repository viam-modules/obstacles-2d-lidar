# pylint: disable=invalid-name
"""
This module provides tools for vectorw calculation
and rotating calipers algorithm.
"""

import math

import matplotlib.pyplot as plt
import numpy as np
from viam.proto.common import Geometry, Pose, RectangularPrism, Vector3

from src.graham_scan import GrahamScan


class Vector2d:
    """
    A class representing a 2-dimensional vector with start and end points.

    Args:
        end (tuple): The end point of the vector.
        start (tuple): The start point of the vector.
        norm (float, optional): The norm (magnitude) of the vector.
    """

    def __init__(self, end, start, norm=None) -> None:
        self.end = end
        self.start = start
        self.coordinates = np.array([end[0] - start[0], end[1] - start[1]])
        self.norm = norm
        self.angle = None

    def get_norm2(self):
        """
        Returns the norm (magnitude) of the vector.

        Returns:
            float: The norm of the vector.
        """
        if self.norm is None:
            self.norm = np.linalg.norm(self.coordinates)
        return self.norm

    def get_angle(self):
        """
        Returns the angle of the vector with respect to the x-axis.

        Returns:
            float: The angle of the vector in radians.
        """
        if self.angle is None:
            self.angle = np.arctan2(self.coordinates[1], self.coordinates[0])

    def is_colinear(self, vec) -> bool:
        """
        Checks if the vector is colinear with another vector.

        Args:
            vec (Vector2d): Another vector to check for colinearity.

        Returns:
            bool: True if the vectors are colinear, False otherwise.
        """
        pd = self.coordinates.dot(vec.vector)
        if abs(pd) != self.get_norm2() * vec.get_norm2():
            return False
        return True


def angle_between_vectors(vector1: Vector2d, vector2: Vector2d):
    """
    Calculates the angle between two vectors.

    Args:
        vector1 (Vector2d): The first vector.
        vector2 (Vector2d): The second vector.

    Returns:
        float: The angle between the vectors in radians.
    """
    dot_product = vector1.coordinates.dot(vector2.coordinates)
    magnitude1 = vector1.get_norm2()
    magnitude2 = vector2.get_norm2()

    # Ensure that the magnitude is not zero to avoid division by zero
    if magnitude1 == 0 or magnitude2 == 0:
        raise ValueError("Vectors cannot have zero magnitude")

    cos_theta = dot_product / (magnitude1 * magnitude2)

    # Use the cross product to determine the sign of the angle
    cross_product = (
        vector1.coordinates[0] * vector2.coordinates[1]
        - vector1.coordinates[1] * vector2.coordinates[0]
    )
    sine_theta = cross_product / (magnitude1 * magnitude2)

    # Calculate the angle in radians
    angle = math.atan2(sine_theta, cos_theta)

    return angle


class BoundingBox2D:
    """
    A class representing a 2D bounding box.

    Args:
        corners (list): The corners of the bounding box.
        area (float, optional): The area of the bounding box.
    """

    def __init__(self, corners: list, area=None):
        self.corners = corners
        c_x, c_y = 0, 0

        for c in corners:
            c_x += c[0]
            c_y += c[1]
        self.center = [c_x / 4, c_y / 4]
        self.area = area

    def get_angle_of_corner(self, corner: int):
        """
        Returns the angle of a specific corner of the bounding box.

        Args:
            corner (int): The index of the corner.

        Returns:
            float: The angle of the corner in radians.
        """
        return angle_between_vectors(
            Vector2d(self.corners[(corner - 1) % 4], self.corners[corner]),
            Vector2d(self.corners[(corner + 1) % 4], self.corners[corner]),
        )

    def get_scaled_geometry(
        self, scale_to: float = 1, prism_z_dim_mm: float = 1
    ) -> Geometry:
        """
        Returns the scaled geometry of the bounding box as a RectangularPrism.

        Args:
            scale_to (float, optional): The scaling factor for the bounding box dimensions.
            prism_z_dim_mm (float, optional): The height of the prism.

        Returns:
            Geometry: The scaled geometry as a RectangularPrism.
        """
        vec1 = Vector2d(self.corners[1], self.corners[0])
        vec2 = Vector2d(self.corners[3], self.corners[0])
        x_rectangle = vec1.get_norm2()
        y_rectangle = vec2.get_norm2()
        theta = angle_between_vectors(Vector2d([1, 0], [0, 0]), vec1)
        return Geometry(
            center=Pose(
                x=self.center[0] * scale_to,
                y=self.center[1] * scale_to,
                z=0,
                o_x=0,
                o_y=0,
                o_z=1,
                theta=theta,
            ),
            box=RectangularPrism(
                dims_mm=Vector3(
                    x=x_rectangle * scale_to, y=y_rectangle * scale_to, z=prism_z_dim_mm
                )
            ),
        )

    def plot(self):
        """
        Plots the bounding box using matplotlib.
        """
        x = [edge[0] for edge in self.corners]
        y = [edge[1] for edge in self.corners]

        x.append(x[0])
        y.append(y[0])

        plt.plot(x, y, linewidth=0.5)


def get_minimum_bounding_box(g: GrahamScan) -> BoundingBox2D:  # pylint: disable=too-many-locals
    """
    Calculates the minimum bounding box for a given set of points.

    Args:
        g (GrahamScan): The GrahamScan object containing the convex hull points.

    Returns:
        BoundingBox2D: The minimum bounding box.
    """
    bb = None
    hull_points_2d = np.array(g.convex_hull)
    edges = []
    n_edges = len(hull_points_2d) - 1
    for i in range(n_edges):
        edges.append(Vector2d(hull_points_2d[i + 1], hull_points_2d[i]))

    edges_angles = []
    for i in range(n_edges):
        angle = np.arctan2(edges[i].coordinates[1], edges[i].coordinates[0])
        edges_angles.append(abs(angle % (np.pi / 2)))

    min_area = math.inf
    for i in range(n_edges):
        theta = edges_angles[i]
        R = np.array(
            [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
        )

        rot_points = np.dot(R, np.transpose(hull_points_2d))

        min_x = np.min(rot_points[0], axis=0)
        max_x = np.max(rot_points[0], axis=0)
        min_y = np.min(rot_points[1], axis=0)
        max_y = np.max(rot_points[1], axis=0)

        width = max_x - min_x
        height = max_y - min_y
        area = width * height
        if area < min_area:
            min_area = area

            # Calculate corner points and restore to original coordinate system
            corner_points = []

            # this garanties clock-wise or anti-clockwise order
            corner_points.append(np.dot([max_x, min_y], R))
            corner_points.append(np.dot([min_x, min_y], R))
            corner_points.append(np.dot([min_x, max_y], R))
            corner_points.append(np.dot([max_x, max_y], R))

            bb = BoundingBox2D(corner_points, area=area)

    return bb
