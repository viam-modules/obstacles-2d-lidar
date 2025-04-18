import asyncio

from viam.services.vision import Vision
from viam.module.module import Module
from viam.resource.registry import Registry, ResourceCreatorRegistration
from src.obstacles_2d_module import ObstacleDetectorModule


async def main():
    """
    This function creates and starts a new module, after adding all desired
    resource models. Resource creators must be registered to the resource
    registry before the module adds the resource model.
    """
    Registry.register_resource_creator(
        Vision.API,
        ObstacleDetectorModule.MODEL,
        ResourceCreatorRegistration(
            ObstacleDetectorModule.new_service, ObstacleDetectorModule.validate_config
        ),
    )
    module = Module.from_args()

    module.add_model_from_registry(Vision.API, ObstacleDetectorModule.MODEL)
    await module.start()


if __name__ == "__main__":
    asyncio.run(main())
