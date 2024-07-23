import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN
from viam.logging import getLogger

from src.dbscan import plot_dbscan_clusters
from src.graham_scan import GrahamScan
from src.pointcloud.point_cloud import PlanarPointCloud
from src.ransac import RansacRegressor
from src.rotating_caliper import get_minimum_bounding_box
from src.utils import plot_geometry, set_pyplot_style

LOGGER = getLogger(__name__)
TO_MM = 1000


class Detector:
    """
    A class to detect obstacles from planar point clouds using DBSCAN and RANSAC algorithms.

    Args:
        normalize (bool, optional): If True, normalization will be applied to the input data.
        dbscan_eps (float, optional): The maximum distance between two samples for
        one to be considered as in the neighborhood of the other in DBSCAN.
        dbscan_min_samples (int, optional): The number of samples in a neighborhood for a
        point to be considered as a core point in DBSCAN.
        min_points_cluster (int, optional): Minimum number of points required in a cluster.
        min_bbox_area (float, optional): Minimum area of the bounding box
        to consider it a valid obstacle.
        ransac_min_samples (int, optional): Minimum number of data points
        required to fit the RANSAC model.
        ransac_residual_threshold (float, optional): Maximum distance for a data
        point to be classified as an inlier in RANSAC.
        ransac_stop_probability (float, optional): Probability that at least one
        outlier-free subset of the data is found in RANSAC.
        prism_z_dim (float, optional): The height of the prism used to scale the geometry.
        save_results (bool, optional): If True, the results will be saved as images.
    """

    def __init__(
        self,
        normalize: float = True,
        dbscan_eps: float = 0.05,
        dbscan_min_samples: int = 2,
        min_points_cluster: int = 2,
        min_bbox_area: float = 0.15,
        ransac_min_samples: int = 2,
        ransac_residual_threshold: float = 0.2,
        ransac_stop_probability: float = 0.99,
        prism_z_dim: float = 1,
        save_results: bool = True,
    ) -> None:
        self.normalize = normalize
        self.dbscan = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples)
        self.min_points_cluster = min_points_cluster
        self.min_bbox_area = min_bbox_area
        self.save_results = save_results
        self.ransac = RansacRegressor(
            min_samples=ransac_min_samples,
            residual_threshold=ransac_residual_threshold,
            stop_probability=ransac_stop_probability,
        )
        self.prism_z_dim = prism_z_dim

    def get_obstacles_from_planar_pcd(self, ppc_input: PlanarPointCloud):
        """
        Detects obstacles from a given planar point cloud.

        Args:
            ppc_input (PlanarPointCloud): The input planar point cloud from planar LiDAR.

        Returns:
            List[Tuple[PlanarPointCloud, Geometry]]: A list of tuples where each tuple contains
            a planar point cloud cluster and its corresponding geometry.
        """

        res = []
        self.fit_dbscan(ppc_input)
        clusters = self.build_ppcs_from_dbscan(ppc_input)
        for cluster in clusters:  # pylint: disable=modified-iterating-list
            if cluster.points.shape[0] < self.min_points_cluster:
                continue
            g = GrahamScan(cluster)
            min_bb = get_minimum_bounding_box(g)

            # if the bounding box found is too big, refine it with RANSAC 2d linear model
            if min_bb.area > self.min_bbox_area:
                self.ransac.fit(cluster.X_norm, cluster.Y_norm)
                _, inlier_mask, outlier_mask = self.ransac.get_one_wall()

                # making sure that we don't refine the same cluster again and again
                # and that we have a proper wall
                n_inliers = inlier_mask.sum()
                if (n_inliers != cluster.points.shape[0]) and (n_inliers > 3):
                    ppc_1 = cluster.get_ppc_from_mask(inlier_mask)
                    ppc_2 = cluster.get_ppc_from_mask(outlier_mask)

                    clusters.append(ppc_1)  # pylint: disable=modified-iterating-list
                    clusters.append(ppc_2)  # pylint: disable=modified-iterating-list

                else:
                    geo = min_bb.get_scaled_geometry(
                        scale_to=ppc_input.scale * TO_MM,
                        prism_z_dim_mm=self.prism_z_dim,
                    )
                    res.append((cluster, geo))

            else:
                geo = min_bb.get_scaled_geometry(
                    scale_to=ppc_input.scale * TO_MM,
                    prism_z_dim_mm=self.prism_z_dim,
                )
                res.append((cluster, geo))

        if self.save_results:
            set_pyplot_style()
            plot_dbscan_clusters(self.dbscan, ppc_input.points_norm)
            for _, geo in res:
                plot_geometry(geo)
            plt.savefig("./results2.png", format="png", dpi=300)

        return res

    def fit_dbscan(self, ppc: PlanarPointCloud):
        """
        Fits the DBSCAN algorithm to the given planar point cloud.

        Args:
            ppc (PlanarPointCloud): The input planar point cloud.
        """
        if self.normalize:
            self.dbscan.fit(ppc.points_norm)
        else:
            self.dbscan.fit(ppc.points)

    def build_ppcs_from_dbscan(self, ppc: PlanarPointCloud) -> list[PlanarPointCloud]:
        """
        Builds clusters from the DBSCAN labels and converts them into planar point clouds.

        Args:
            ppc (PlanarPointCloud): The input planar point cloud.

        Returns:
            List[PlanarPointCloud]: A list of planar point cloud clusters.
        """
        ppcs = []
        labels = self.dbscan.labels_
        unique_labels = set(labels)
        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[self.dbscan.core_sample_indices_] = True

        for l in unique_labels:
            class_member_mask = labels == l
            mask = class_member_mask & core_samples_mask
            if mask.sum() > self.min_points_cluster:
                cluster = ppc.get_ppc_from_mask(mask)
                ppcs.append(cluster)
        return ppcs
