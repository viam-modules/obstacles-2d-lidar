# pylint: disable=invalid-name

import numpy as np

from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from src.pointcloud.point_cloud import PlanarPointCloud


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

    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]

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
