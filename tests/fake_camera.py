from typing import Any, Dict, List, Optional, Tuple

from viam.components.camera import Camera
from viam.gen.component.camera.v1.camera_pb2 import GetPropertiesResponse
from viam.media.video import NamedImage, ViamImage
from viam.proto.common import ResponseMetadata


class FakeCamera(Camera):
    def __init__(self, name: str, path_to_input: str):
        super().__init__(name=name)
        with open(path_to_input, "rb") as file:
            self.pcd_bytes = file.read()

    async def get_image(
        self,
        mime_type: str = "",
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> ViamImage:
        raise NotImplementedError

    async def get_images(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Tuple[List[NamedImage], ResponseMetadata]:
        raise NotImplementedError

    async def get_properties(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> GetPropertiesResponse:
        raise NotImplementedError

    async def get_point_cloud(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bytes, str]:
        return self.pcd_bytes, self.name
