from sklearn.cluster import DBSCAN
from typing import List
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.camera import Camera
import numpy as np
import utils
import ransac
import dbscan
import graham_scan
import rotating_caliper
import dbscan
from pointcloud.point_cloud import PlanarPointCloud
from viam.logging import getLogger

LOGGER = getLogger(__name__)

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
                 save_results:bool= False) -> None:
        
        
        self.normalize = normalize
        self.dbscan = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples)
        self.min_points_cluster = min_points_cluster
        self.min_bbox_area = min_bbox_area
        self.save_results = save_results

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
                g = graham_scan.GrahamScan(cluster) 
                if self.save_results:
                    g.plot()
                
                min_bb = rotating_caliper.get_minimum_bounding_box(np.array(g.convex_hull))
                
                #if the bounding box found is too big, refine it with RANSAC 2d linear model
                if min_bb.area >self.min_bbox_area:
                    _, inlier_mask, outlier_mask = ransac.get_one_wall(cluster.X_norm, cluster.Y_norm)
                    
                    #making sure that we don't refine the same cluster again and again
                    if  (inlier_mask.sum() != cluster.points.shape[0]) or (inlier_mask.sum() != 0):
                        ppc_1 = cluster.get_ppc_from_mask(inlier_mask)
                        ppc_2 = cluster.get_ppc_from_mask(outlier_mask)
                        
                        clusters.append(ppc_1)
                        clusters.append(ppc_2)
                    
                    else:
                        geo =min_bb.get_geometry()
                        res.append((cluster, geo))
                        if self.save_results:
                            utils.plot_geometry(geo)

                else:
                    geo =min_bb.get_geometry()
                    res.append((cluster, geo))
                    if self.save_results:
                        utils.plot_geometry(geo)
        if self.save_results:
            dbscan.plot_clusters(self.dbscan, input.points_norm)       
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
        