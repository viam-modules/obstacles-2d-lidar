from point_cloud import PointCloud, PlanarPointCloud
from decode_pcd import *
from encode_pcd import *
import struct
import open3d as o3d
import numpy as np


path_to_pcd = "./data/pointcloud_data.pcd"
path_to_pcd2= "./src/data/pointcloud_data2.pcd"
# ##asyncio.run(get_pcd_from_viam(path_to_pcd))
with open('./src/data/pointcloud_data.pcd', 'rb') as file:
    pcd_bytes = file.read()



def test_decode_pcd(pcd_bytes):
    pc = decode_pcd_bytes(pcd_bytes)
    return pc

pc = test_decode_pcd(pcd_bytes)
encoder = Encoder(pc.metadata)
pcd_bytes2 = encoder.encode(pc)
# pcd_bytes2 = encode_pointcloud_to_pcd_bytes(pc)

bunny_path =  './src/data/bunny_ascii.pcd'
encoded_pcd_output_path  = './src/data/output_test.pcd'
with open(encoded_pcd_output_path, 'wb') as f:
    f.write(pcd_bytes2)


pcd = o3d.io.read_point_cloud(encoded_pcd_output_path)
pcd_poinnts = np.array(pcd.points)
print(pcd_poinnts.shape)
print(pcd_poinnts.max())


ppc = pc.get_planar_from_3D()

print("FINISHED")