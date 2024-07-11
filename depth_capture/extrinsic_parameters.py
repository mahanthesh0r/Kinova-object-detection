
import sys 
import os

from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient

from kortex_api.autogen.messages import DeviceConfig_pb2, Session_pb2, DeviceManager_pb2, VisionConfig_pb2


def print_extrinsic_parameters(extrinsics):
    print("Rotation matrix:")
    print("[{0: .6f} {1: .6f} {2: .6f}".format ( \
            extrinsics.rotation.row1.column1, extrinsics.rotation.row1.column2, extrinsics.rotation.row1.column3))
    print(" {0: .6f} {1: .6f} {2: .6f}".format ( \
            extrinsics.rotation.row2.column1, extrinsics.rotation.row2.column2, extrinsics.rotation.row2.column3))
    print(" {0: .6f} {1: .6f} {2: .6f}]".format ( \
            extrinsics.rotation.row3.column1, extrinsics.rotation.row3.column2, extrinsics.rotation.row3.column3))
    print("Translation vector: [{0:.6f} {1:.6f} {2:.6f}]".format( \
                                extrinsics.translation.t_x, extrinsics.translation.t_y, extrinsics.translation.t_z))
    

def example_vision_get_device_id(device_manager):
    vision_device_id = 0
    
    # getting all device routing information (from DeviceManagerClient service)
    all_devices_info = device_manager.ReadAllDevices()

    vision_handles = [ hd for hd in all_devices_info.device_handle if hd.device_type == DeviceConfig_pb2.VISION ]
    if len(vision_handles) == 0:
        print("Error: there is no vision device registered in the devices info")
    elif len(vision_handles) > 1:
        print("Error: there are more than one vision device registered in the devices info")
    else:
        handle = vision_handles[0]
        vision_device_id = handle.device_identifier
        print("Vision module found, device Id: {0}".format(vision_device_id))

    return vision_device_id

#
# Example showing how to retrieve the extrinsic parameters
#
def example_routed_vision_get_extrinsics(vision_config, vision_device_id):
    print("\n\n** Example showing how to retrieve the extrinsic parameters **")

    print("\n-- Using Vision Config Service to get extrinsic parameters --")
    extrinsics = vision_config.GetExtrinsicParameters(vision_device_id)
    print_extrinsic_parameters(extrinsics)


def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ""))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        device_manager = DeviceManagerClient(router)
        vision_config = VisionConfigClient(router)

        # example core
        vision_device_id = example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            example_routed_vision_get_extrinsics(vision_config, vision_device_id)

if __name__ == "__main__":
    main()