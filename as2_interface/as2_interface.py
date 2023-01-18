"""
as2_interface.py
"""

import sys
import os
sys.path.insert(0, os.getcwd())

import threading
import time
import copy
from typing import Callable, List
import rclpy
from AerostackUI.websocket_interface import WebSocketClientInterface
import as2_interface.Unit_conversion as utm
import as2_interface.SwarmingLib as swarm
import numpy as np
from as2_python_api.drone_interface_gps import DroneInterfaceGPS


GPS_CENTER = [28.143989765431503, -16.503195203840733]
# GPS_CENTER = [40.158873, -3.811725]
IGNORE_YAW = True


class UavInterface(DroneInterfaceGPS):
    """ UAV Interface """
    info_lock = threading.Lock()

    def __init__(self, uav_id: str):
        self.drone_interface = super(UavInterface, self)
        self.drone_interface.__init__(uav_id, verbose=False)
        self.uav_id = uav_id

    def info_lock_decor(func: Callable) -> Callable:
        def wrapper(self, *args, **kwargs):
            with self.info_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @info_lock_decor
    def get_info(self):
        """ Get info """
        gps_pose = self.drone_interface.gps_pose
        orientation = self.drone_interface.orientation
        height = self.drone_interface.position[2]
        id = self.drone_interface.drone_id
        info = self.drone_interface.info

        info_collection = {
            'id': id,
            'state': info,
            'pose': {'lat': gps_pose[0], 'lng': gps_pose[1], 'height': height,
                     'yaw': orientation[2]},
        }
        return info_collection

    def run_uav_mission(self, mission: list, thread: threading.Thread) -> None:
        """ Run UAV mission """

        # print("MISSION")
        # print(mission)

        # uav = str(self.uav_id)
        drone_interface = self.drone_interface

        # TODO: CHANGE WHEN NOT SIMULATION
        # print(f"Arming for UAV {uav}")
        # drone_interface.arm()
        # print(f"Offboard for UAV {uav}")
        # drone_interface.offboard()

        for element in mission:
            # print(f"Element for UAV {uav}: ")
            # print(element)
            speed = float(element['speed'])

            if element['name'] == 'TakeOffPoint':

                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                # print(f"{uav} - Send takeoff")
                # print(waypoint[2])
                drone_interface.takeoff(height=waypoint[2], speed=speed)
                # print(f"{uav} - Takeoff done")

                # print(f"{uav} - Send takeoff waypoint")
                drone_interface.go_to_gps_point(waypoint, speed, IGNORE_YAW)
                # print(f"{uav} - Takeoff waypoint done")

            elif element['name'] == 'LandPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                # print(f"{uav} - Send land point")
                # drone_interface.follow_gps_path([waypoint])
                drone_interface.go_to_gps_point(waypoint, speed, IGNORE_YAW)
                # print(f"{uav} - Land point done")

                # print(f"{uav} - Send land")
                drone_interface.land()
                # print(f"{uav} - Land done")

            elif element['name'] == 'Path':
                waypoint = element['values']

                # print(f"{uav} - Send path")
                # drone_interface.follow_gps_path(waypoint, speed)
                for wp in waypoint:
                    drone_interface.go_to_gps_point(
                        [wp[0], wp[1], wp[2]], speed, IGNORE_YAW)
                # print(f"{uav} - Path done")

            elif element['name'] == 'WayPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                # print(f"Send waypoint: {waypoint}")
                # drone_interface.follow_gps_wp([waypoint], speed)
                drone_interface.go_to_gps_point(waypoint, speed, IGNORE_YAW)
                time.sleep(2.0)  # TODO: Remove this
                # print(f"{uav} - Waypoint done")

            elif element['name'] == 'Area':
                waypoint = element['values']
                # print(f"{uav} - Send area")
                # drone_interface.follow_gps_path(waypoint[1:], speed)

                for wp in waypoint[1:]:
                    drone_interface.go_to_gps_point(
                        [wp[0], wp[1], wp[2]], speed, IGNORE_YAW)
                # print(f"{uav} - Area done")

            else:
                # print("Unknown layer")
                # print("Element: ", element)
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        if thread is not None:
            thread.join()


class MissionManager():
    """ Mission Manager """

    def __init__(self):
        self.mission_id = 0
        self.mission_list = {}

    def mission_interpreter(self, msg: dict) -> dict:
        """ Mission interpreter """
        # print(f"- Mission interpreter")
        # print(msg)

        confirm = 'confirmed'
        extra = []

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

        # TODO: Check if mission mission_id change
        confirm_msg = {
            'id': self.mission_id,
            'status': confirm,
            'extra': extra
        }

        self.mission_id += 1

        return confirm_msg

    def mission_planner(self, mission_id: str, mission_info: dict) -> dict:
        """Convert Aerostack UI mission to ROS mission

        Args:
            mission_id (number): Mission id
            uavList (list): List of UAVs
            layers (list): List of layers
        """

        send_mission = {
            'id': mission_id,
            'uavList': mission_info['uavList'],
            'layers': []
        }

        first_uav = mission_info['uavList'][0]

        mission = {}
        last_position = {}
        for uav in mission_info['uavList']:
            mission[str(uav)] = []
            last_position[str(uav)] = [None, None, None]

        for layer in mission_info['layers']:

            send_layer, mission, new_last_position = self.layer_interpreter(
                layer, mission, last_position, first_uav, mission_info)

            for uav in new_last_position.items():
                last_position[uav] = new_last_position[uav]

            send_mission['layers'].append(send_layer)

        self.mission_list[mission_id] = mission
        return send_mission

    def swarm_planning(self, uav_list: list, initial_position: list,
                       last_position: list, height: float, values: list,
                       algorithm: str, street_spacing: float, wp_space: float,
                       theta: float) -> tuple:
        """ Swarm planning """
        zone = None
        letter = None

        # Convert to utm
        initial_position_utm = {}
        for uav in uav_list:
            east, north, zone_number, zone_letter = utm.GPS_to_UTM(
                initial_position[uav][0], initial_position[uav][1])
            initial_position_utm[uav] = [east, north]
            zone = zone_number
            letter = zone_letter

        last_position_utm = {}
        for uav in uav_list:
            east, north, zone_number, zone_letter = utm.GPS_to_UTM(
                last_position[uav][0], last_position[uav][1])
            last_position_utm[uav] = [east, north]

        values_utm = []
        for value in values:
            east, north, zone_number, zone_letter = utm.GPS_to_UTM(
                value['lat'], value['lng'])
            values_utm.append([east, north])

        # Swarm planning
        uav_initial_position = []
        for uav in initial_position_utm.items():
            uav_initial_position.append(
                [initial_position_utm[uav][0], initial_position_utm[uav][1]])

        uav_last_position = []
        for uav in last_position_utm.items():
            uav_last_position.append(
                [last_position_utm[uav][0], last_position_utm[uav][1]])

        vel_input = np.full(len(uav_initial_position), 1)
        vel_sum = sum(vel_input)
        uav_weight = np.zeros_like(vel_input, dtype=float)
        for i in range(0, len(vel_input)):
            uav_weight[i] = float(vel_input[i])/vel_sum

        waypoints, wpt_grid = swarm.compute_area(
            np.array(uav_initial_position),
            np.array(uav_last_position),
            uav_weight,
            np.array(values_utm),
            altitude=height,
            street_spacing=street_spacing,
            wpt_separation=wp_space,
            path_algorithm=algorithm,
            distribution_algorithm="binpat",
            theta=theta
        )

        # Data format
        uav_list_wp_utm = {}
        for index, uav_wp in enumerate(waypoints):
            uav_wp_aux = []
            for wp in uav_wp:
                if not np.isnan(wp[0]) or not np.isnan(wp[1]):
                    uav_wp_aux.append(wp)
                else:
                    break
            uav_list_wp_utm[uav_list[index]] = uav_wp_aux

        uav_list_wp_gps = {}
        uav_list_wp_gps_v2 = {}

        for uav in uav_list:

            uav_list_wp_gps[uav] = [
                {'lat': initial_position[uav][0], 'lng': initial_position[uav][1]}]
            uav_list_wp_gps_v2[uav] = [
                [initial_position[uav][0], initial_position[uav][1], height]]

            utm_values = uav_list_wp_utm[uav]
            for utm_value in utm_values:
                gps_value = utm.UTM_to_GPS(
                    utm_value[0], utm_value[1], zone, letter)
                uav_list_wp_gps[uav].append(
                    {'lat': gps_value[0], 'lng': gps_value[1]})
                uav_list_wp_gps_v2[uav].append(
                    [gps_value[0], gps_value[1], height])

            uav_list_wp_gps[uav].append(
                {'lat': last_position[uav][0], 'lng': last_position[uav][1]})
            uav_list_wp_gps_v2[uav].append(
                [last_position[uav][0], last_position[uav][1], height])

        return uav_list_wp_gps, uav_list_wp_gps_v2

    def get_next_position(self, sublist: list, mission: list, last_position: list,
                          first_uav: str, mission_info: dict) -> dict:
        """ Get next position """
        mission_aux = copy.deepcopy(mission)
        last_position_aux = copy.deepcopy(last_position)

        last_position_list = {}
        last_position_flag = {}
        for uav in mission_info['uavList']:
            last_position_list[str(uav)] = [None, None, None]
            last_position_flag[str(uav)] = False

        for layer in sublist:

            send_layer, mission, last_position = self.layer_interpreter(
                layer, mission_aux, last_position_aux, first_uav, mission_info)

            for uav in last_position:
                if last_position_flag[uav] == False:
                    last_position_list[uav] = last_position[uav]
                    last_position_flag[uav] = True

            all_flags = True
            for uav in last_position_flag:
                if not last_position_flag[uav]:
                    all_flags = False

            if all_flags:
                return last_position_list

        raise Exception("Next position for UAV not found")

    def layer_interpreter(self, layer: dict, mission: dict, last_position: dict,
                          first_uav: str, mission_info: dict) -> tuple:
        """ Layer interpreter """
        name = layer['name']
        uav_list = layer['uavList']
        height = layer['height']
        # print("Layer interpreter: " + name)
        # print(layer)
        values = layer['values']
        speed = layer['speed']
        send_layer = {
            'name': name,
            'uavList': uav_list,
            'height': height,
            'speed': speed,
            'values': values
        }

        new_last_position = {}

        if (name == 'TakeOffPoint' or name == 'LandPoint' or name == 'WayPoint'):
            uav = uav_list[0]
            if uav == 'auto':
                uav = first_uav

            marker_position = [values['lat'], values['lng'], height[1]]
            new_last_position[uav] = marker_position

            mission[uav].append({
                'name': name,
                'speed': speed,
                'values': [marker_position]
            })
            send_layer['uavList'] = [uav]

        elif (name == 'Path'):
            uav = uav_list[0]
            if uav == 'auto':
                uav = first_uav

            waypoints = []
            for point in layer['values']:
                waypoints.append(
                    [point['lat'], point['lng'], layer['height'][1]])

            new_last_position[uav] = waypoints[len(waypoints)-1]

            mission[uav].append({
                'name': name,
                'speed': speed,
                'values': waypoints
            })

            send_layer['uavList'] = [uav]

        elif name == 'Area':

            if uav_list[0] == 'auto':
                uav_list_aux = mission_info['uavList']
            else:
                uav_list_aux = uav_list

            index = mission_info['layers'].index(layer)
            range_to_end = range(index+1, len(mission_info['layers']))
            sublist = [mission_info['layers'][i] for i in range_to_end]

            algorithm = layer['algorithm']
            street_spacing = layer['streetSpacing']
            wp_space = layer['wpSpace']
            theta = layer['orientation']

            if float(theta) >= 0 and float(theta) <= 360:
                theta = float(theta)
            else:
                theta = None

            next_position = self.get_next_position(
                sublist, mission, last_position, first_uav, mission_info)

            uav_path, path = self.swarm_planning(
                uav_list_aux, last_position, next_position, float(height[1]), 
                values[0], str(algorithm), float(street_spacing), float(wp_space), 
                theta)

            send_layer['uavPath'] = {}
            for uav in uav_list_aux:
                new_last_position[uav] = next_position[uav]
                mission[uav].append({
                    'name': name,
                    'speed': speed,
                    'values': path[uav]
                })

            send_layer['uavPath'] = uav_path
            send_layer['uavList'] = uav_list_aux

        else:
            raise Exception("Unknown layer name")

        return send_layer, mission, new_last_position


class AerostackUI():
    """ Aerostack UI """

    def __init__(self, uav_id_list: list):
        self.client = WebSocketClientInterface("ws://127.0.0.1:8000/ws/user/")
        
        # self.mission_manager = MissionManager()

        # self.client.add_msg_callback(
        #     'request', 'missionConfirm', self.mission_confirm_callback)
        # self.client.add_msg_callback(
        #     'request', 'missionStart', self.start_mission_callback)

        # self.uav_id_list = uav_id_list

        # self.drone_interface = {}
        # for uav_id in self.uav_id_list:
        #     # print(f"UAV {uav_id} initialize")
        #     drone_node = UavInterface(uav_id)
        #     self.drone_interface[uav_id] = drone_node

        # time.sleep(1)

        self.get_info_thread = threading.Thread(target=self.run)
        self.get_info_thread.start()

    def fake_mission(self, msg: dict):
        """ Fake mission """
        time.sleep(3)
        self.mission_confirm_callback(msg, [])

    def mission_confirm_callback(self, msg: dict, args):
        """ Mission confirm callback """
        # print("-Mission confirm callback")
        confirm_msg = self.mission_manager.mission_interpreter(msg)

        self.client.missionConfirm(
            confirm_msg['id'],
            confirm_msg['status'],
            msg['payload']['id'],
            msg['from'],
            confirm_msg['extra']
        )

        if confirm_msg['status'] == 'confirmed':

            mission_planner_msg = self.mission_manager.mission_planner(
                str(confirm_msg['id']),
                msg['payload']
            )

            mission_planner_msg['status'] = confirm_msg['status']
            self.client.send_mission_info(mission_planner_msg)

    def start_mission_callback(self, msg: dict, args):
        """ Start mission callback """
        # print("AerostackUI - Start mission")
        # print(msg)

        mission_id = str(msg['payload']['id'])
        mission_list = self.mission_manager.mission_list[mission_id]

        # print("- Start mission ", mission_id)
        self.thread_uav = {}
        for uav in mission_list:
            drone_interface = self.drone_interface[uav]

            if drone_interface == None:
                continue

            print("Starting mission for uav ", uav)
            self.thread_uav[uav] = None
            mission_for_uav = mission_list[uav]

            self.thread_uav[uav] = threading.Thread(
                target=drone_interface.run_uav_mission,
                args=[mission_for_uav, self.thread_uav[uav]])
            self.thread_uav[uav].start()

    def run(self):
        """ Run """

        # print("Running info publisher")
        # odom = {}

        # for uav in self.uav_id_list:
        #     odom[uav] = []

        while self.client.connection:
            continue
            for idx, uav in enumerate(self.uav_id_list):

                drone_interface_i = self.drone_interface[uav]

                send_info = drone_interface_i.get_info()

                pose_fail1 = {'lat': 0.0, 'lng': 0.0}
                pose_fail2 = {'lat': float('NaN'), 'lng': float('NaN')}

                if send_info['pose']['lat'] != pose_fail1['lat'] or \
                   send_info['pose']['lng'] != pose_fail1['lng'] or \
                   send_info['pose']['lat'] != pose_fail2['lat'] or \
                   send_info['pose']['lng'] != pose_fail2['lng']:

                    if len(odom[uav]) > 1000:
                        odom[uav].pop(0)

                    odom[uav].append(
                        [send_info['pose']['lat'], send_info['pose']['lng']])
                    send_info['odom'] = odom[uav]
                    self.client.send_uav_info(send_info)
                    # print(f"UAV {uav} info sent:")
                    # print(send_info)
                else:
                    print("Error sending info for ", uav)
                    # print(send_info['pose'])

            time.sleep(0.5)
            # else:
            #     print("Conecction lost")
            #     time.sleep(1)
        print("Connection lost")
        self.get_info_thread.join()


if __name__ == '__main__':
    rclpy.init()

    aerostackUI = AerostackUI([
        'drone_0'
    ])
