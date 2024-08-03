# pylint: disable=invalid-name, missing-function-docstring

import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN


class PlanarPointCloud:
    """
    self.X array of shape (n_points, 1)
    self.Y array of shape (n_points, 1)
    """

    def __init__(self, points: list = [], normalize: bool = False) -> None:  # pylint: disable=dangerous-default-value
        """_summary_

        Args:
            points (list, optional): _description_. Defaults to [].
            normalized (bool, optional): Defines if the input point passed
            to the constructor are normalized. Defaults to False.
            scale (float, optional): None if not normalized.
        """

        self.points = np.array(points)
        self.points_norm = None

        if normalize:
            self.normalize_point_cloud()

        self.scale = None  # indicates if the pointcloud has been normalized yet

    @property
    def X(self):
        """
        self.X array of shape (n_points, 1)
        """
        X = self.points[:, 0]
        return X.reshape(-1, 1)

    @property
    def Y(self):
        """
        self.Y array of shape (n_points, 1)
        """
        Y = self.points[:, 1]
        return Y.reshape(-1, 1)

    @property
    def X_norm(self):
        if self.scale is None:
            self.normalize_point_cloud()
        return self.points_norm[:, 0].reshape(-1, 1)

    @property
    def Y_norm(self):
        if self.scale is None:
            self.normalize_point_cloud()
        return self.points_norm[:, 1].reshape(-1, 1)

    def normalize_point_cloud(self):
        self.scale = max(
            abs(self.X.max()), abs(self.X.min()), abs(self.Y.max()), abs(self.Y.min())
        )
        self.points_norm = self.points / self.scale

    def get_ppc_from_mask(self, mask, keep_scale=True):
        res = PlanarPointCloud(points=self.points[mask])
        if keep_scale:
            if self.scale is None:
                self.normalize_point_cloud()
            else:
                res.scale = self.scale
                res.points_norm = self.points_norm[mask]
        return res

    def plot(self, normalized=False):
        if normalized:
            plt.scatter(
                self.X_norm,
                self.Y_norm,
                s=10,
                c="b",
                marker="o",
                label="Normalized Point Cloud",
            )

        else:
            plt.scatter(self.X, self.Y, s=10, c="b", marker="o", label="Point Cloud")

        # Add labels and title
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.title("Point Cloud Plot")

        # Add a legend
        plt.legend()

        # Show the plot
        plt.grid(True)
        plt.show()


def get_dbscan(X, Y):
    """
    Performs DBSCAN clustering on the given data.

    Args:
        X (np.ndarray): Array of x-coordinates of the points.
        Y (np.ndarray): Array of y-coordinates of the points.

    Returns:
        Tuple[DBSCAN, int, int]: Fitted DBSCAN model,
        number of clusters, and number of noise points.
    """
    combined_array = np.concatenate((X, Y), axis=1)
    db = DBSCAN(eps=0.05, min_samples=2).fit(combined_array)
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)
    return db, n_clusters_, n_noise_


def plot_dbscan_clusters(db: DBSCAN, X: np.ndarray):
    """
    Plots the clusters identified by DBSCAN.

    Args:
        db (DBSCAN): Fitted DBSCAN model.
        X (np.ndarray): Array of points.
    """
    labels = db.labels_
    unique_labels = set(labels)
    core_samples_mask = np.zeros_like(labels, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    cmap = plt.cm.get_cmap("Spectral")
    colors = [cmap(each) for each in np.linspace(0, 1, len(unique_labels))]

    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = labels == k

        # xy = X[class_member_mask & core_samples_mask]
        xy = X[class_member_mask]
        plt.plot(
            xy[:, 0],
            xy[:, 1],
            "o",
            markerfacecolor=tuple(col),
            markeredgecolor=(0, 0, 0, 1),
            markeredgewidth=0.2,
            markersize=4,
        )


def build_ppc_from_dbscan(
    db: DBSCAN, points: np.array, normalized: bool = False
) -> list[PlanarPointCloud]:
    """
    Builds a list of PlanarPointCloud objects from the DBSCAN clustering result.

    Args:
        db (DBSCAN): Fitted DBSCAN model.
        points (np.ndarray): Array of points.
        normalized (bool, optional): Whether to normalize the point cloud. Defaults to False.

    Returns:
        List[PlanarPointCloud]: List of PlanarPointCloud objects for each cluster.
    """
    ppcs = []
    labels = db.labels_
    unique_labels = set(labels)
    core_samples_mask = np.zeros_like(labels, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True

    for l in unique_labels:
        class_member_mask = labels == l
        sub_points = points[class_member_mask & core_samples_mask]
        ppcs.append(PlanarPointCloud(sub_points, normalized))

    return ppcs


class PointCloud:
    """
    self.points is a list of list of coordinates
    should be ordered like  'x', 'y','z'
    """

    def __init__(self, points: list, metadata: dict = None) -> None:
        self.points = np.array(points)
        self.metadata = metadata

    def get_planar_from_3D(self, axis_normal: str = "z") -> PlanarPointCloud:
        """_summary_

        Projects a 3D pointcloud in 2D. Assume that 3D point cloud is ordered like 'x', 'y', 'z'
        and returns the  XY projection.

        Args:
            pc (PointCloud): input

        Returns:
            PlanarPointCloud: ouput
        """

        if axis_normal == "z":
            return PlanarPointCloud(self.points[:, :2])
        if axis_normal == "y":
            return PlanarPointCloud(self.points[:, 1:2])
        if axis_normal == "x":
            return PlanarPointCloud(self.points[:, 0:])
        if axis_normal == "auto":
            projection_2d = self.detect_and_project_plane()
            return PlanarPointCloud(points=projection_2d)

        raise ValueError(
            "Invalid normal axis. Please provide 'x', 'y' or 'z' or 2 as argument for normal axis."  # pylint: disable=line-too-long
        )

    def detect_and_project_plane(self):
        """Find the plane where all the points belong and remove the dimension orthogonal
        to that plane.
        """
        if len(self.points) < 3:
            raise AssertionError("not enough points to define a plane")

        vectors = self.points - self.points[0]
        normal_vector = np.cross(vectors[1], vectors[2])  # check if all points
        parallel = all(
            np.dot(normal_vector, vec) == 0 for vec in vectors[3:]
        )  # belong to the same plane
        if parallel:
            return self.project(normal_vector)
        raise AssertionError("point cloud is not comprehended onto a plane")

    def project(self, normal_vector: np.array):
        normal_vector = normal_vector / np.linalg.norm(
            normal_vector
        )  # normalize normal vector
        projection = self.points - np.outer(
            np.dot(self.points, normal_vector), normal_vector
        )  # project
        projection_2d = projection[:, :2]

        return projection_2d


def get_pc_from_ppc(
    ppc: PlanarPointCloud, z: float = 0, metadata: dict = None, scale_to: float = 1
):
    """ "Add" a dimension to a planar pointcloud to make it a 3d.

    Args:
        ppc (PlanarPointCloud): _description_
        z (float, optional): _description_. Defaults to 0.
        metadata (dict, optional): _description_. Defaults to None.
        scale_to (float, optional): _description_. Defaults to 1.

    Returns:
        PointClouc: _description_
    """
    n_points = ppc.points.shape[0]
    z_values = np.ones((n_points, 1)) * z
    points = np.concatenate((ppc.points * scale_to, z_values), axis=1)
    return PointCloud(points=points.tolist(), metadata=metadata)
