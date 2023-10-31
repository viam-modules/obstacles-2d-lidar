
import pointcloud.point_cloud as pointcloud
from pointcloud.decode_pcd import *
from detector import Detector


with open('./src/data/pointcloud_data.pcd', 'rb') as file:
            pcd_bytes = file.read()
        
pc = decode_pcd_bytes(pcd_bytes)
ppc = pc.get_planar_from_3D()
ppc.normalize_point_cloud()

detector = Detector()

obstacles = detector.get_obstacles_from_planar_pcd(ppc)
