from typing import ClassVar, List, Mapping, Sequence, Any, Dict, Optional
from typing_extensions import Self
from viam.components.camera import Camera
from viam.media.video import RawImage
from viam.proto.service.vision import Classification, Detection
from viam.services.vision import Vision
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ServiceConfig
from viam.proto.common import PointCloudObject, ResourceName
from viam.proto.common import GeometriesInFrame
from viam.resource.base import ResourceBase
from viam.utils import ValueTypes
from viam.logging import getLogger
from .detector import Detector
from .pointcloud.point_cloud import get_pc_from_pcc
from .pointcloud.decode_pcd import decode_pcd_bytes
from .pointcloud.encode_pcd import Encoder

LOGGER = getLogger(__name__)

class ObstacleDetectorModule(Vision, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam", "vision"), "obstacles_2d_lidar")
     
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
                "A camera name is required for obstacles_2d_lidar vision service module.")
        return [camera_name]

    def reconfigure(self,
            config: ServiceConfig,
            dependencies: Mapping[ResourceName, ResourceBase]):
        
        self.camera_name = config.attributes.fields["camera_name"].string_value
        self.camera = dependencies[Camera.get_resource_name(self.camera_name)]
        
        def get_attribute_from_config(attribute_name:str,  default, of_type=None):
            if attribute_name not in config.attributes.fields:
                return default

            if default is None:
                if of_type is None:
                    raise Exception("If default value is None, of_type argument can't be empty")
                type_default = of_type
            else:    
                type_default = type(default)

            if type_default == int:
                return int(config.attributes.fields[attribute_name].number_value)
            elif type_default == float:
                return config.attributes.fields[attribute_name].number_value
            elif type_default == str:
                return config.attributes.fields[attribute_name].string_value

        dbscan_eps = get_attribute_from_config('dbscan_eps', 0.05)
        dbscan_min_samples = get_attribute_from_config('dbscan_min_samples',2)
        min_points_cluster = get_attribute_from_config('min_points_cluster', 2)
        min_bbox_area = get_attribute_from_config('min_bbox_area', 0.15)
        ransac_min_samples = get_attribute_from_config("ransac_min_samples", 2)
        ransac_residual_threshold = get_attribute_from_config('ransac_residual_threshold', 0.2)
        ransac_stop_probability = get_attribute_from_config("ransac_stop_probability", 0.99)
        prism_z_dim = get_attribute_from_config('obstacles_height_mm', 1.0)
        self.normal_vector = get_attribute_from_config("normal_vector", 'z')
        self.min_range_mm = get_attribute_from_config("min_range_mm", None, float)

        self.detector = Detector(normalize=True,
                                 dbscan_eps=dbscan_eps,
                                 dbscan_min_samples=dbscan_min_samples,
                                 min_points_cluster=min_points_cluster,
                                 min_bbox_area=min_bbox_area,
                                 ransac_min_samples=ransac_min_samples, 
                                 ransac_residual_threshold=ransac_residual_threshold, 
                                 ransac_stop_probability=ransac_stop_probability, 
                                 prism_z_dim = prism_z_dim)
        
    async def get_object_point_clouds(self,
                                      camera_name: str,
                                      *,
                                      extra: Optional[Dict[str, Any]] = None,
                                      timeout: Optional[float] = None,
                                      **kwargs) -> List[PointCloudObject]:
        
        
        
        pcd_bytes, _ = await self.camera.get_point_cloud()
        pc = decode_pcd_bytes(pcd_bytes, self.min_range_mm)
        ppc = pc.get_planar_from_3D(axis_normal=self.normal_vector)
        ppc.normalize_point_cloud()
        
        obstacles = self.detector.get_obstacles_from_planar_pcd(ppc, normalize=True)
        encoder = Encoder(pc.metadata)
        res = []
        for ppc, geo in obstacles:
            #do PCD encoding
            pc_1 = get_pc_from_pcc(ppc = ppc, z = 0, metadata=pc.metadata)
            cluster_pcd_bytes = encoder.encode_new(pc_1)
            #get geo_in_frames
            geometry_in_frame = GeometriesInFrame(reference_frame=self.camera_name, geometries=[geo])
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