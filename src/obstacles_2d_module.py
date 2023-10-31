from typing import ClassVar, List, Mapping, Sequence, Any, Dict, Optional, Union, cast
from typing_extensions import Self
from PIL import Image
from viam.media.video import RawImage
from viam.proto.service.vision import Classification, Detection
from viam.services.vision import Vision
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ComponentConfig, ServiceConfig
from viam.proto.common import PointCloudObject, ResourceName
from viam.proto.common import GeometriesInFrame, Geometry
from viam.resource.base import ResourceBase
from viam.utils import ValueTypes
from viam.logging import getLogger
from detector import Detector
import pointcloud.point_cloud as point_cloud
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
        # camera_name = config.attributes.fields["camera_name"].string_value
        # if camera_name == "":
        #     raise Exception(
        #         "A camerax name is required as an attribute for an AWS vision service.")
        # return [camera_name]
    
        pass
    def reconfigure(self,
            config: ServiceConfig,
            dependencies: Mapping[ResourceName, ResourceBase]):
        
        self.detector = Detector()
        
        pass
        
    async def get_object_point_clouds(self,
                                      camera_name: str,
                                      *,
                                      extra: Optional[Dict[str, Any]] = None,
                                      timeout: Optional[float] = None,
                                      **kwargs) -> List[PointCloudObject]:
        
        
        path_to_pcd = "./data/pointcloud_data.pcd"
        path_to_pcd2= "./data/pointcloud_data2.pcd"
        # ##asyncio.run(get_pcd_from_viam(path_to_pcd))
        with open('./src/data/pointcloud_data.pcd', 'rb') as file:
            pcd_bytes = file.read()
        
        pc = point_cloud.decode_pcd_bytes(pcd_bytes)
        ppc = point_cloud.get_planar_from_3D(pc)
        ppc.normalize_point_cloud()
        LOGGER.error(f"TEST LOGGER")
        geometries = self.detector.get_obstacles_from_planar_pcd(ppc)
        LOGGER.error(f"TEST LOGGER2")
        res = utils.geometries_to_pointcloud_objects(geometries, camera_name)
        LOGGER.error(f"TYPE RES {type(res)}")
        LOGGER.error(f"TYPE RES[0] {type(res[0])}")
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