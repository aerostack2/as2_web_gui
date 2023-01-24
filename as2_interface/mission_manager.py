"""
mission_manager.py
"""

import SwarmingLib as swarm
import Unit_conversion as utm
import numpy as np
import copy


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
                values[0], str(algorithm), float(
                    street_spacing), float(wp_space),
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


