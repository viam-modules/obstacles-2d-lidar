import os
from typing import Dict, List

import pytest
from fake_camera import FakeCamera
from google.protobuf.struct_pb2 import Struct
from viam.media.video import CameraMimeType, ViamImage
from viam.proto.app.robot import ServiceConfig
from viam.proto.common import PointCloudObject
from viam.services.vision import Detection, Vision

from src.obstacles_2d_module import ObstacleDetectorModule

CAMERA_NAME = "fake-camera"
PATH_TO_INPUT = os.path.join("tests", "data", "pointcloud_data.pcd")

WORKING_CONFIG_DICT = {
    "camera_name": CAMERA_NAME,
}


PASSING_PROPERTIES = Vision.Properties(
    classifications_supported=False,
    detections_supported=False,
    object_point_clouds_supported=True,
)

PASSING_NUMBER_OF_PCD_OBJECTS = 15


def get_config(config_dict: Dict):
    """returns a config populated with picture_directory and camera_name
    attributes.

    Returns:
        ServiceConfig: _description_
    """
    struct = Struct()
    struct.update(dictionary=config_dict)
    config = ServiceConfig(attributes=struct)
    return config


def get_camera(camera_name=CAMERA_NAME, path_to_input=PATH_TO_INPUT):
    return FakeCamera(camera_name, path_to_input)


def get_vision_service(config_dict: Dict) -> ObstacleDetectorModule:
    """Returns a vision service with its dependencies
    after calling
    `validate_config()` and `reconfigure()`.

    Args:
        config_dict (Dict): config as dictionnary

    Returns:
        ObstacleDetectorModule:2d obstacle detector module
    """
    service = ObstacleDetectorModule("test")
    cam = get_camera()
    camera_name = cam.get_resource_name(CAMERA_NAME)
    cfg = get_config(config_dict)
    service.validate_config(cfg)
    service.reconfigure(cfg, dependencies={camera_name: cam})
    return service


class TestMotionDetector:
    def test_empty_config(self):
        with pytest.raises(ValueError):
            _ = get_vision_service(config_dict={})

    def test_passing_config(self):
        _ = get_vision_service(config_dict=WORKING_CONFIG_DICT)

    @pytest.mark.asyncio
    async def test_get_properties(self):
        service = get_vision_service(config_dict=WORKING_CONFIG_DICT)
        p = await service.get_properties()
        assert p == PASSING_PROPERTIES

    @pytest.mark.asyncio
    async def test_get_detections_from_camera(self):
        service = get_vision_service(WORKING_CONFIG_DICT)
        pcd = await service.get_object_point_clouds(camera_name=CAMERA_NAME)
        check_pcd_result(pcd)
        assert isinstance(pcd, list)
        assert len(pcd) == PASSING_NUMBER_OF_PCD_OBJECTS
        obj = pcd[0]
        assert isinstance(obj, PointCloudObject)
        assert obj.geometries is not None
        assert obj.point_cloud is not None

    @pytest.mark.asyncio
    async def test_capture_all_from_camera(self):
        service = get_vision_service(WORKING_CONFIG_DICT)
        capture_all_result = await service.capture_all_from_camera(
            CAMERA_NAME,
            return_image=True,
            return_classifications=True,
            return_detections=True,
            return_object_point_clouds=True,
        )
        check_pcd_result(capture_all_result.objects)
        assert isinstance(capture_all_result.image, ViamImage)
        assert capture_all_result.image.mime_type == CameraMimeType.PCD

    @pytest.mark.asyncio
    async def test_default_camera(self):
        service = get_vision_service(WORKING_CONFIG_DICT)
        result = await service.get_object_point_clouds("")
        check_pcd_result(result)

        result = await service.capture_all_from_camera(
            "",
            return_object_point_clouds=True,
        )
        check_pcd_result(result.objects)

        with pytest.raises(ValueError) as excinfo:
            await service.get_object_point_clouds("not-camera")
        assert "not-camera" in str(excinfo.value)

        with pytest.raises(ValueError) as excinfo:
            await service.capture_all_from_camera(
                "not-camera",
                return_object_point_clouds=True,
            )
        assert "not-camera" in str(excinfo.value)


def check_pcd_result(
    pcd: List[Detection], target_num_objects: int = PASSING_NUMBER_OF_PCD_OBJECTS
):
    assert isinstance(pcd, list)
    assert len(pcd) == PASSING_NUMBER_OF_PCD_OBJECTS
    obj = pcd[0]
    assert isinstance(obj, PointCloudObject)
    assert obj.geometries is not None
    assert obj.point_cloud is not None
