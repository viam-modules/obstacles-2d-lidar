from typing import ClassVar, List, Mapping, Sequence, Any, Dict, Optional, Union, cast
from typing_extensions import Self
from PIL import Image
from viam.components.camera import Camera
from viam.media.video import RawImage
from viam.proto.service.vision import Classification, Detection
from viam.services.vision import Vision
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ServiceConfig
from viam.proto.common import PointCloudObject, ResourceName
from viam.proto.common import GeometriesInFrame, Geometry
from viam.resource.base import ResourceBase
from viam.utils import ValueTypes
from viam.logging import getLogger
from detector import Detector
import pointcloud.point_cloud as pointcloud
from pointcloud.decode_pcd import *
from pointcloud.encode_pcd import *
import utils

LOGGER = getLogger(__name__)

class ObstacleDetectorModule(Vision, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam", "vision"), "obstacles-2d-detector")
     
    def __init__(self, name: str):
        super().__init__(name=name)
        
    @classmethod
    def new_service(cls,
                 config: ServiceConfig,
                 dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        service = cls(config.name)
        service.reconfigure(config, dependencies)
        return service
    
     # Validates JSON Configuration
    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Sequence[str]:
        camera_name = config.attributes.fields["camera_name"].string_value
        if camera_name == "":
            raise Exception(
                "A camera name is required for 2d_obstacles_vision service.")
        return [camera_name]

    def reconfigure(self,
            config: ServiceConfig,
            dependencies: Mapping[ResourceName, ResourceBase]):
        
        camera_name = config.attributes.fields["camera_name"].string_value
        self.camera = dependencies[Camera.get_resource_name(camera_name)]
        self.detector = Detector()
        
        pass
        
    async def get_object_point_clouds(self,
                                      camera_name: str,
                                      *,
                                      extra: Optional[Dict[str, Any]] = None,
                                      timeout: Optional[float] = None,
                                      **kwargs) -> List[PointCloudObject]:
        
        
        
        # pcd_bytes = await self.camera.get_point_cloud()
        with open('./src/data/pointcloud_data.pcd', 'rb') as file:
            pcd_bytes = file.read()
        
        pc = decode_pcd_bytes(pcd_bytes)
        ppc = pc.get_planar_from_3D()
        ppc.normalize_point_cloud()
    
        
        obstacles = self.detector.get_obstacles_from_planar_pcd(ppc, normalize=True)
        encoder = Encoder(pc.metadata)
        res = []
        for ppc, geo in obstacles:
            
            #do PCD encoding
            pc_1 = pointcloud.get_pc_from_pcc(ppc = ppc, z = 0, metadata=pc.metadata)
            cluster_pcd_bytes = encoder.encode(pc_1)
            
            #get geo_in_frames
            geometry_in_frame = GeometriesInFrame(reference_frame=camera_name, geometries=[geo])
            res.append(PointCloudObject(point_cloud=cluster_pcd_bytes,
                                        geometries=geometry_in_frame))
        return res
        
    
    async def get_detections(self, image: RawImage, *, extra: Mapping[str, Any], timeout: float) -> List[Detection]:
        return  NotImplementedError
    
    async def get_classifications(self, image: RawImage, count: int, *, extra: Mapping[str, Any]) -> List[Classification]:
        return NotImplementedError
    
    async def get_classifications_from_camera(self) -> List[Classification]:
        return NotImplementedError
    
    async def get_detections_from_camera(self, camera_name: str, *, extra: Mapping[str, Any]) -> List[Detection]:
        return NotImplementedError
    
    
    async def do_command(self,
                        command: Mapping[str, ValueTypes],
                        *,
                        timeout: Optional[float] = None,
                        **kwargs):
        raise NotImplementedError