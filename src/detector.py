import asyncio


from sklearn.cluster import DBSCAN
from typing import List
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.camera import Camera
from viam.services.vision import Detection
from viam.gen.common.v1.common_pb2 import GeoObstacle
from viam.gen.common.v1.common_pb2 import Geometry
from viam.proto.common import GeometriesInFrame, Geometry, RectangularPrism, PoseInFrame, Pose
import numpy as np
import open3d as o3d
from PIL import Image
import os
import utils
import ransac
import dbscan
import graham_scan
import rotating_caliper
import pointcloud.point_cloud as point_cloud
import matplotlib.pyplot as plt
import dbscan
from dbscan import Point, Cluster
from pointcloud.point_cloud import PlanarPointCloud
from viam.logging import getLogger

LOGGER = getLogger(__name__)

class Detector():
    def __init__(self,
                 lidar: Camera=None, 
                 dbscan_eps:float=0.05,
                 dbscan_min_samples:int=2,
                 min_points_cluster:int=2,
                 save_results:bool= True) -> None:
        
        self.dbscan = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples)
        self.min_points_cluster = min_points_cluster
        self.save_results = save_results
    
    def get_obstacles_from_planar_pcd(self, ppc:PlanarPointCloud, normalized = True)-> List[Geometry]:
        
        res = []
        if normalized:
            if not hasattr(ppc, "X_norm"):
                ppc.normalize_point_cloud()
            X, Y  = ppc.X_norm,ppc.Y_norm
        else:
            X, Y = ppc.X, ppc.Y    
        
        self.fit_dbscan(X,Y)
        clusters = self.get_clusters_from_dbscan()
        for cluster in clusters:
            if cluster.points.shape[0]>self.min_points_cluster:
                h = graham_scan.ConvexHull(cluster.points.tolist())
                # h.plot()
                #Compute and plot 
                min_bb = rotating_caliper.get_minimum_bounding_box(np.array(h.points))
                if min_bb.area >0.15:
                    ##refine clustering with ransac
                    x = cluster.points[:, 0].reshape(-1,1)
                    y = cluster.points[:, 1].reshape(-1,1)
                    
                    ransac_regressor, inlier_mask, outlier_mask = ransac.get_one_wall(x, y)
                    
                    clusters.append(dbscan.Cluster(points=np.concatenate((x[inlier_mask], y[inlier_mask]), axis=1)))
                    clusters.append(dbscan.Cluster(points = np.concatenate((x[outlier_mask], y[outlier_mask]), axis=1)))

                else:
                    geo =min_bb.get_geometry()
                    res.append(geo)
                    # utils.plot_geometry(geo)
        return res
    
    def fit_dbscan(self, X, Y):
        self.combined_array = np.concatenate((X, Y), axis=1)
        self.dbscan.fit(self.combined_array)
        labels = self.dbscan.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)
        
    def get_clusters_from_dbscan(self)-> list[Cluster]:
        return dbscan.build_clusters(self.dbscan, self.combined_array) 