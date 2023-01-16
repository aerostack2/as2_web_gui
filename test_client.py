""" Test client """
import time
import math

from as2_web_api.websocket_interface import WebSocketClientInterface


def planner(websocket_client: WebSocketClientInterface, uav_id: int, layers: dict):
    """ Mission planner """
    trayectory = []
    for layer in layers:
        print(layer)
        if layer['name'] == 'TakeOffPoint' or layer['name'] == 'LandPoint':
            trayectory.append([layer['values']['lat'], layer['values']['lng']])

        elif layer['name'] == 'Path':
            path = layer['values']
            for waypoint in path:
                trayectory.append([waypoint['lat'], waypoint['lng']])

    if len(trayectory) > 0:
        websocket_client.info_messages.send_uav_info(
            {'id': uav_id, 'desiredPath': trayectory})


def new_mission_callback(msg: dict, args: tuple):
    """ Mission process callback """
    confirm = 'confirmed'
    extra = []
    
    websocket_client = args[0]

    if msg['payload']['status'] == 'request':
        if len(msg['payload']['uavList']) == 0:
            confirm = 'rejected'
            extra.append('No UAVs')

        if len(msg['payload']['layers']) == 0:
            confirm = 'rejected'
            extra.append('No layers')

        if msg['payload']['id'] != 'New Mission':
            confirm = 'rejected'
            extra.append(
                'Invalid id, only "New Mission" is allowed for now :)')

    websocket_client.request_messages.mission_confirm(
        websocket_client.data.mission_id,
        confirm,
        msg['payload']['id'],
        msg['from'],
        extra
    )

    websocket_client.data.mission_id += 1

    if confirm == 'confirmed':
        new_mission_info = msg['payload']
        new_mission_info['status'] = confirm
        new_mission_info['id'] = websocket_client.data.mission_id

        websocket_client.info_messages.send_mission_info(new_mission_info)

        # planner(websocket_client, msg['payload']['uavList'][0], msg['payload']['layers'])


def deg2rad(angle):
    """ Convert degrees to radians """
    return angle * math.pi / 180


if __name__ == "__main__":
    client = WebSocketClientInterface("ws://127.0.0.1:8000/ws/user/", True)

    client.add_msg_callback('request', 'missionConfirm', new_mission_callback, client)

    time.sleep(1)
    
    client.info_messages.send_uav_info(
        {
            'id': 'UAV0',
            'state': 'landed',
            'pose': {'lat': 28.144099, 'lng': -16.503337, 'height': 1, 'yaw': deg2rad(0)},
            'sensors': {'battey': 80, 'temperature': 40}
        })

    # time.sleep(1)

    # latitude = 28.144099
    # longitude = -16.503337
    # height = 1
    # increment = 0.00001
    # odometry = []
    # yaw = 0

    # for i in range(1000):
    #     time.sleep(1. / 5)
    #     latitude += increment
    #     longitude += increment
    #     yaw += increment * 1000000
    #     height += increment * 10000
    #     odometry.append([latitude, longitude])
    #     client.info_messages.send_uav_info(
    #         {
    #             'id': 'UAV 0',
    #             'pose': {'lat': latitude, 'lng': longitude, 'height': height, 'yaw': deg2rad(yaw)},
    #             'odom': odometry
    #         })
