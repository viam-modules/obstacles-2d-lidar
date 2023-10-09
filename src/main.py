import asyncio

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
import point_cloud
import matplotlib.pyplot as plt

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
   
if __name__ == '__main__':
    
    path_to_pcd = "./data/pointcloud_data.pcd"
    path_to_pcd2= "./data/pointcloud_data2.pcd"
    # ##asyncio.run(get_pcd_from_viam(path_to_pcd))
    with open('./src/data/pointcloud_data.pcd', 'rb') as file:
        pcd_bytes = file.read()
    
    pc = point_cloud.decode_pcd_bytes(pcd_bytes)
    ppc = point_cloud.get_planar_from_3D(pc)
    ppc.normalize_point_cloud()

    db, n_clusters, n_noise = dbscan.get_dbscan(ppc.X_norm,ppc.Y_norm)
    clusters = dbscan.build_clusters(db, np.concatenate((ppc.X_norm, ppc.Y_norm), axis=1))
    count  = 0
    for cluster in clusters:
        
        if cluster.points.shape[0]>2:
            h = graham_scan.ConvexHull(cluster.points.tolist())
            h.plot()

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
                utils.plot_geometry(geo)
    
    dbscan.plot_clusters(db, np.concatenate((ppc.X_norm, ppc.Y_norm), axis=1))
    