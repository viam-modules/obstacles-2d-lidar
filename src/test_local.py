
import pointcloud.point_cloud as pointcloud
from pointcloud.decode_pcd import *
from pointcloud.encode_pcd import *
from detector import Detector
from viam.proto.common import PointCloudObject, GeometriesInFrame

with open('./src/data/pointcloud_data.pcd', 'rb') as file:
            pcd_bytes = file.read()
        
pc = decode_pcd_bytes(pcd_bytes)
ppc = pc.get_planar_from_3D()
ppc.normalize_point_cloud()

detector = Detector()

##Next is what the module is "asking" to the detector
obstacles = detector.get_obstacles_from_planar_pcd(ppc, normalize=True)

encoder = Encoder(pc.metadata)
res = []
for ppc, geo in obstacles:
    ## get pcd_bytes
    pc_1 = pointcloud.get_pc_from_pcc(ppc = ppc, z = 0, metadata=pc.metadata)
    cluster_pcd_bytes = encoder.encode(pc_1)
    
    #get geo_in_frames
    geometry_in_frame = GeometriesInFrame(reference_frame="ref_frame", geometries=[geo])
    res.append(PointCloudObject(point_cloud=cluster_pcd_bytes,
                                geometries=geometry_in_frame))
    
print(res)
    
