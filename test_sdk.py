import asyncio

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.vision import VisionClient

async def connect():
    creds = Credentials(
        type='robot-location-secret',
        payload='adpcqgrzdhwa1q42bdeu4bsn6ee7vemx9fjuerl21ux75ydv')
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address('mac-server-main.wszwqu7wcv.viam.cloud', opts)

async def main():
    robot = await connect()

    print('Resources:')
    print(robot.resource_names)

    detector_module = VisionClient.from_robot(robot, "detector-module")
    detector_module_return_value = await detector_module.get_object_point_clouds(camera_name="cam")
    print(f"detector-module get_classifications_from_camera return value: {detector_module_return_value}")
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
