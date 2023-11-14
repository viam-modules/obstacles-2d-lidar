from sklearn.cluster import DBSCAN
import numpy as np
from .utils import set_pyplot_style, plot_geometry
from .ransac import RansacRegressor
from .dbscan import plot_dbscan_clusters
from .graham_scan import GrahamScan
from .rotating_caliper import get_minimum_bounding_box
from .pointcloud.point_cloud import PlanarPointCloud
from viam.logging import getLogger
import matplotlib.pyplot as plt

LOGGER = getLogger(__name__)
TO_MM = 1000

class Detector():
    '''
    normalize means that the ML will be done on normalized inputs
    '''
    def __init__(self,
                 normalize:float = True,  
                 dbscan_eps:float=0.05,
                 dbscan_min_samples:int=2,
                 min_points_cluster:int=2,
                 min_bbox_area:float=0.15,
                 ransac_min_samples:int=2, 
                 ransac_residual_threshold:float=.2, 
                 ransac_stop_probability:float=.99,
                 prism_z_dim: float=1, 
                 save_results:bool= False) -> None:
        
        
        self.normalize = normalize
        self.dbscan = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples)
        self.min_points_cluster = min_points_cluster
        self.min_bbox_area = min_bbox_area
        self.save_results = save_results
        self.ransac= RansacRegressor(min_samples=ransac_min_samples, 
                                            residual_threshold=ransac_residual_threshold, 
                                            stop_probability=ransac_stop_probability)
        self.prism_z_dim = prism_z_dim
        
    def get_obstacles_from_planar_pcd(self, input:PlanarPointCloud, normalize = True):
        """
        Returns a list of (PlanarPointCloud, Geometry)

        Args:
            ppc (PlanarPointCloud): input from planar lidar
            normalize (bool, optional): Whether DBSCAN and RANSAC should use the normalized coordinates.

        Returns:
            List[PlanarPointCloud, Geometry]:
        """        
        
        res = []
        self.fit_dbscan(input)
        clusters = self.build_ppcs_from_dbscan(input)
        for cluster in clusters:
            if cluster.points.shape[0] < self.min_points_cluster:
                continue
            else:
                g = GrahamScan(cluster) 
                min_bb = get_minimum_bounding_box(g)
                
                #if the bounding box found is too big, refine it with RANSAC 2d linear model
                if min_bb.area >self.min_bbox_area:
                    self.ransac.fit(cluster.X_norm, cluster.Y_norm)
                    _, inlier_mask, outlier_mask = self.ransac.get_one_wall()
                    
                    #making sure that we don't refine the same cluster again and again
                    #and that we have a proper wall
                    n_inliers = inlier_mask.sum()
                    if  (n_inliers != cluster.points.shape[0]) and (n_inliers >3):
                        ppc_1 = cluster.get_ppc_from_mask(inlier_mask)
                        ppc_2 = cluster.get_ppc_from_mask(outlier_mask)
                        
                        clusters.append(ppc_1)
                        clusters.append(ppc_2)
                    
                    else:
                        geo =min_bb.get_scaled_geometry(scale_to= input.scale * TO_MM,
                                                 prism_z_dim_mm=self.prism_z_dim)
                        res.append((cluster, geo))

                else:
                    geo =min_bb.get_scaled_geometry(scale_to= input.scale * TO_MM,
                                                 prism_z_dim_mm=self.prism_z_dim)
                    res.append((cluster, geo))
        
        if self.save_results:
            set_pyplot_style()
            plot_dbscan_clusters(self.dbscan, input.points_norm)
            for _, geo in res:
                plot_geometry(geo)
            plt.savefig("./results.png", format='png', dpi=300)       
        
        return res
            

    def fit_dbscan(self, ppc:PlanarPointCloud):
        if self.normalize:
            self.dbscan.fit(ppc.points_norm)
        else:
            self.dbscan.fit(ppc.points)
    
    
    def build_ppcs_from_dbscan(self, ppc:PlanarPointCloud) -> list[PlanarPointCloud]:
        ppcs = []
        labels = self.dbscan.labels_
        unique_labels = set(labels)
        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[self.dbscan.core_sample_indices_] = True
        
        for l in unique_labels:
            class_member_mask = (labels == l)
            mask = class_member_mask & core_samples_mask
            if mask.sum() >self.min_points_cluster:
                cluster = ppc.get_ppc_from_mask(mask)
                ppcs.append(cluster)
        return ppcs
        