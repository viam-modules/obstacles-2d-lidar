from typing import Any, ClassVar, List, Mapping, Optional, Sequence

from typing_extensions import Self
from viam.components.camera import Camera
from viam.logging import getLogger
from viam.media.video import CameraMimeType, ViamImage
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ServiceConfig
from viam.proto.common import GeometriesInFrame, PointCloudObject, ResourceName
from viam.proto.service.vision import Classification, Detection
from viam.resource.base import ResourceBase
from viam.resource.types import (
    Model,
    ModelFamily,
)
from viam.services.vision import CaptureAllResult, Vision
from viam.utils import ValueTypes

from src.detector import Detector
from src.pointcloud.decode_pcd import decode_pcd_bytes
from src.pointcloud.encode_pcd import Encoder
from src.pointcloud.point_cloud import get_pc_from_ppc

LOGGER = getLogger(__name__)


class ObstacleDetectorModule(Vision, Reconfigurable):
    """Object detector module is a subclass a Viam Vision
    Service that provides 2d obstacles detection capabilities."""

    MODEL: ClassVar[Model] = Model(ModelFamily("viam", "vision"), "obstacles_2d_lidar")

    def __init__(self, name: str):
        super().__init__(name=name)

    @classmethod
    def new_service(
        cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        "returns new service"
        service = cls(config.name)
        service.reconfigure(config, dependencies)
        return service

    # Validates JSON Configuration
    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Sequence[str]:
        "returns list of dependencies after validating the config"
        camera_name = config.attributes.fields["camera_name"].string_value
        if camera_name == "":
            raise ValueError(
                "A camera name is required for obstacles_2d_lidar vision service module."
            )
        return [camera_name]

    def reconfigure(
        self, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.camera_name = config.attributes.fields["camera_name"].string_value
        self.camera = dependencies[Camera.get_resource_name(self.camera_name)]

        def get_attribute_from_config(attribute_name: str, default, of_type=None):
            if attribute_name not in config.attributes.fields:
                return default

            if default is None:
                if of_type is None:
                    raise ValueError(
                        "If default value is None, of_type argument can't be empty"
                    )
                type_default = of_type
            else:
                type_default = type(default)

            if type_default is bool:
                return config.attributes.fields[attribute_name].bool_value
            if type_default is int:
                return int(config.attributes.fields[attribute_name].number_value)
            if type_default is float:
                return config.attributes.fields[attribute_name].number_value
            if type_default is str:
                return config.attributes.fields[attribute_name].string_value
            if type_default is dict:
                return dict(config.attributes.fields[attribute_name].struct_value)

            raise ValueError("can't parse attribute from config.")

        dbscan_eps = get_attribute_from_config("dbscan_eps", 0.05)
        dbscan_min_samples = get_attribute_from_config("dbscan_min_samples", 2)
        min_points_cluster = get_attribute_from_config("min_points_cluster", 2)
        min_bbox_area = get_attribute_from_config("min_bbox_area", 0.15)
        ransac_min_samples = get_attribute_from_config("ransac_min_samples", 2)
        ransac_residual_threshold = get_attribute_from_config(
            "ransac_residual_threshold", 0.2
        )
        ransac_stop_probability = get_attribute_from_config(
            "ransac_stop_probability", 0.99
        )
        prism_z_dim = get_attribute_from_config("obstacles_height_mm", 1.0)
        self.normal_vector = get_attribute_from_config("normal_vector", "z")
        self.min_range_mm = get_attribute_from_config("min_range_mm", None, float)

        self.detector = Detector(
            normalize=True,
            dbscan_eps=dbscan_eps,
            dbscan_min_samples=dbscan_min_samples,
            min_points_cluster=min_points_cluster,
            min_bbox_area=min_bbox_area,
            ransac_min_samples=ransac_min_samples,
            ransac_residual_threshold=ransac_residual_threshold,
            ransac_stop_probability=ransac_stop_probability,
            prism_z_dim=prism_z_dim,
        )

    async def get_properties(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> Vision.Properties:
        return Vision.Properties(
            classifications_supported=False,
            detections_supported=False,
            object_point_clouds_supported=True,
        )

    async def get_object_point_clouds(
        self,
        camera_name: str,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> List[PointCloudObject]:
        pcd_bytes, _ = await self.camera.get_point_cloud()
        pc = decode_pcd_bytes(pcd_bytes, self.min_range_mm)
        ppc = pc.get_planar_from_3D(axis_normal=self.normal_vector)
        ppc.normalize_point_cloud()

        obstacles = self.detector.get_obstacles_from_planar_pcd(ppc)
        encoder = Encoder(pc.metadata)
        res = []
        for ppc, geo in obstacles:
            # do PCD encoding
            pc_1 = get_pc_from_ppc(ppc=ppc, z=0, metadata=pc.metadata)
            cluster_pcd_bytes = encoder.encode_new(pc_1)
            # get geo_in_frames
            geometry_in_frame = GeometriesInFrame(
                reference_frame=self.camera_name, geometries=[geo]
            )
            res.append(
                PointCloudObject(
                    point_cloud=cluster_pcd_bytes, geometries=geometry_in_frame
                )
            )
        return res

    async def get_detections(
        self,
        image: ViamImage,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> List[Detection]:
        return NotImplementedError

    async def get_classifications(
        self,
        image: ViamImage,
        count: int,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> List[Classification]:
        return NotImplementedError

    async def get_classifications_from_camera(
        self,
        camera_name: str,
        count: int,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> List[Classification]:
        return NotImplementedError

    async def get_detections_from_camera(
        self,
        camera_name: str,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> List[Detection]:
        return NotImplementedError

    async def capture_all_from_camera(
        self,
        camera_name: str,
        return_image: bool = False,
        return_classifications: bool = False,
        return_detections: bool = False,
        return_object_point_clouds: bool = False,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> CaptureAllResult:
        """


        Args:
            camera_name (str): _description_
            return_image (bool, optional): _description_. Defaults to False.
            return_classifications (bool, optional): _description_. Defaults to False.
            return_detections (bool, optional): _description_. Defaults to False.
            return_object_point_clouds (bool, optional): _description_. Defaults to False.
            extra (Optional[Mapping[str, Any]], optional): _description_. Defaults to None.
            timeout (Optional[float], optional): _description_. Defaults to None.

        Raises:
            NotImplementedError: _description_

        Returns:
            CaptureAllResult: _description_
        """

        pcd_bytes, _ = await self.camera.get_point_cloud()
        pc = decode_pcd_bytes(pcd_bytes, self.min_range_mm)
        ppc = pc.get_planar_from_3D(axis_normal=self.normal_vector)
        ppc.normalize_point_cloud()

        res = None
        if return_object_point_clouds:
            obstacles = self.detector.get_obstacles_from_planar_pcd(ppc)
            encoder = Encoder(pc.metadata)
            res = []
            for ppc, geo in obstacles:
                # do PCD encoding
                pc_1 = get_pc_from_ppc(ppc=ppc, z=0, metadata=pc.metadata)
                cluster_pcd_bytes = encoder.encode_new(pc_1)
                # get geo_in_frames
                geometry_in_frame = GeometriesInFrame(
                    reference_frame=self.camera_name, geometries=[geo]
                )
                res.append(
                    PointCloudObject(
                        point_cloud=cluster_pcd_bytes, geometries=geometry_in_frame
                    )
                )

        if not return_image:
            viam_im = None
        else:
            viam_im = ViamImage(pcd_bytes, CameraMimeType.PCD)

        return CaptureAllResult(image=viam_im, objects=res)

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        raise NotImplementedError
