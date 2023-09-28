import asyncio

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.camera import Camera
import numpy as np
import open3d as o3d
from PIL import Image
import os
import utils
import ransac
import dbscan
import graham_scan2
import rotating_caliper

from sklearn.decomposition import PCA, FastICA

import matplotlib.pyplot as plt
import time
import math

async def connect():
    creds = Credentials(
        type='robot-location-secret',
        payload='adpcqgrzdhwa1q42bdeu4bsn6ee7vemx9fjuerl21ux75ydv')
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address('mac-server-main.wszwqu7wcv.viam.cloud', opts)

async def get_pcd_from_viam(path_to_pcd):
    robot = await connect()

    print('Resources:')
    print(robot.resource_names)
    
    # rplidar
    rplidar = Camera.from_robot(robot, "rplidar")
    pc = await rplidar.get_point_cloud()
    print(f"type of pc[0] is {type(pc[0])}")
    print(f"type of pc[1] is {type(pc[1])}")
    print(f"pc[1] is {pc[1]}")
    # print(pc[0])
    print('\n')
    # print(pc[1])
    # pcd = o3d.geometry.PointCloud(pc[0])

    with open(path_to_pcd, "wb") as f:
        f.write(pc[0])
   
    

def process_pcd(path_to_pcd:str):
    centers = np.asarray([[1, 1], [-1, -1], [1, -1]])
    print(centers.shape)
    
    pcd = o3d.io.read_point_cloud(path_to_pcd)
    print(f"type of pcd is {type(pcd)}")
    points = np.asarray(pcd.points)
    x, y = utils.create_projection_from_list(points)
    
    
    X = np.array(x).reshape(-1, 1)
    Y = np.array(y ).reshape(-1, 1)
    X, Y= utils.normalize_pcd(X, Y)


    #Compute and plot clusters
    db, n_clusters, n_noise = dbscan.get_dbscan(X,Y)
    clusters = dbscan.build_clusters(db, np.concatenate((X, Y), axis=1))
    # dbscan.plot_clusters(db, np.concatenate((X, Y), axis=1))
    
    
    
    for cluster in clusters:
        if cluster.points.shape[0]>2:
            h = graham_scan2.ConvexHull(cluster.points.tolist())
            h.plot_convex_hull()

            #Compute and plot 
            min_bb = rotating_caliper.get_minimum_bounding_box(np.array(h.points))
            if min_bb.area >0.15:
                ##refine clustering with ransac
                # print(f"cluster points shape is {cluster.points.shape}  ")
                x = cluster.points[:, 0].reshape(-1,1)
                y = cluster.points[:, 1].reshape(-1,1)
                # print(f"x points shape is {x.shape}  ")
                # print(f"y points shape is {y.shape}  ")
                ransac_regressor, inlier_mask, outlier_mask = ransac.get_one_wall(x, y)
                # print(f"REACHED HERE")
                # print(np.concatenate((x[inlier_mask], y[inlier_mask]), axis = 1).shape)
                clusters.append(dbscan.Cluster(points=np.concatenate((x[inlier_mask], y[inlier_mask]), axis=1)))
                clusters.append(dbscan.Cluster(points = np.concatenate((x[outlier_mask], y[outlier_mask]), axis=1)))
            else:
                min_bb.plot()
                    
        # print(min_bb.edges[0].shape)

    
    dbscan.plot_clusters(db, np.concatenate((X, Y), axis=1))
    
    # print(f"X and Y size are {X.shape, Y.shape}")
    
    # for _ in range(5):
    #     _, inlier_mask, outlier_mask = ransac.get_one_wall(X,Y)
    #     utils.plot_inliers_and_outliers(X, Y, inlier_mask)
    #     X, Y = X[outlier_mask], Y[outlier_mask]
    
if __name__ == '__main__':
    
    path_to_pcd = "./data/pointcloud_data.pcd"
    path_to_pcd2= "./data/pointcloud_data2.pcd"
    # ##asyncio.run(get_pcd_from_viam(path_to_pcd))
    process_pcd(path_to_pcd)
    process_pcd(path_to_pcd2)
