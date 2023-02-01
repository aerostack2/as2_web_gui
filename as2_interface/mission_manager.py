"""
mission_manager.py
"""

import as2_interface.SwarmingLib as swarm
import as2_interface.Unit_conversion as utm
import numpy as np
import copy
from AerostackUI.websocket_interface import WebSocketClientInterface
from AerostackUI.aerostack_ui_logger import AerostackUILogger
from .uav_manager import UavManager
import threading


class MissionInterpreter():
    """ Mission Interpreter """

    @staticmethod
    def interpreter(msg: dict, logger: AerostackUILogger) -> dict:
        """ Mission interpreter """
        logger.debug("MissionManager", "interpreter", f"Interpreting mission: {msg}")

        if msg['payload']['status'] != 'request':
            message_info = f"Invalid mission status: {msg['payload']['status']}, only 'request' is allowed for now"
            logger.info("MissionManager", "interpreter", message_info)

        if msg['payload']['id'] != 'New Mission':
            message_info = f"Invalid mission id: {msg['payload']['id']}, only 'New Mission' is allowed for now"
            logger.info("MissionManager", "interpreter", message_info)
            return False, message_info

        if len(msg['payload']['uavList']) == 0:
            message_info = "No UAVs selected in mission"
            logger.info("MissionManager", "interpreter", message_info)
            return False, message_info

        if len(msg['payload']['layers']) == 0:
            message_info = "No layers selected in mission"
            logger.info("MissionManager", "interpreter", message_info)
            return False, message_info

        return True, str("")

    @staticmethod
    def planner(mission_id: str, mission_info: dict) -> dict:
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

            send_layer, mission, new_last_position = MissionInterpreter.layer_interpreter(
                layer, mission, last_position, first_uav, mission_info)

            for uav in new_last_position:
                last_position[uav] = new_last_position[uav]

            send_mission['layers'].append(send_layer)

        return send_mission

    @staticmethod
    def layer_interpreter(layer: dict, mission: dict, last_position: dict,
                          first_uav: str, mission_info: dict) -> tuple:
        """ Layer interpreter """
        name = layer['name']
        uav_list = layer['uavList']
        height = float(layer['height'])
        values = layer['values']
        speed = float(layer['speed'])
        send_layer = {
            'name': name,
            'uavList': uav_list,
            'height': height,
            'speed': speed,
            'values': values
        }

        new_last_position = {}

        if name in ('TakeOffPoint', 'LandPoint', 'WayPoint', 'Path'):

            uav = uav_list[0]
            if uav == 'auto':
                uav = first_uav

            waypoints = []
            if name == 'Path':
                for point in layer['values']:
                    waypoints.append(
                        [point[0], point[0], height])
                new_last_position[uav] = waypoints[len(waypoints)-1]
            else:
                waypoints = [[values[0], values[1], height]]
                new_last_position[uav] = waypoints[0]

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
            theta = float(layer['orientation'])

            if theta < 0 or theta > 360:
                theta = None

            next_position = MissionInterpreter.get_next_position(
                sublist, mission, last_position, first_uav, mission_info)

            area_path = MissionInterpreter.swarm_planning(
                uav_list_aux, last_position, next_position, height,
                values[0], str(algorithm), float(
                    street_spacing), float(wp_space),
                theta)

            send_layer['uavPath'] = {}
            for uav in uav_list_aux:
                new_last_position[uav] = next_position[uav]
                mission[uav].append({
                    'name': name,
                    'speed': speed,
                    'values': area_path[uav]
                })

            send_layer['uavPath'] = area_path
            send_layer['uavList'] = uav_list_aux

        else:
            raise Exception("Unknown layer name")

        return send_layer, mission, new_last_position

    @staticmethod
    def get_next_position(sublist: list, mission: list, last_position: list,
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

            send_layer, mission, last_position = MissionInterpreter.layer_interpreter(
                layer, mission_aux, last_position_aux, first_uav, mission_info)

            for uav in last_position:
                if last_position_flag[uav] == False:
                    last_position_list[uav] = last_position[uav]
                    last_position_flag[uav] = True        

            for uav in last_position_flag:
                if not last_position_flag[uav]:
                    raise Exception("Next position for UAV not found")

        return last_position_list

    @staticmethod
    def swarm_planning(uav_list: list, initial_position: list,
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
                value[0], value[1])
            values_utm.append([east, north])

        # Swarm planning
        uav_initial_position = []
        for uav in initial_position_utm:
            uav_initial_position.append(
                [initial_position_utm[uav][0], initial_position_utm[uav][1]])

        uav_last_position = []
        for uav in last_position_utm:
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

        for uav in uav_list:

            uav_list_wp_gps[uav] = [
                [initial_position[uav][0], initial_position[uav][1], height]]

            utm_values = uav_list_wp_utm[uav]
            for utm_value in utm_values:
                gps_value = utm.UTM_to_GPS(
                    utm_value[0], utm_value[1], zone, letter)
                uav_list_wp_gps[uav].append(
                    [gps_value[0], gps_value[1], height])

            uav_list_wp_gps[uav].append(
                [last_position[uav][0], last_position[uav][1], height])

        return uav_list_wp_gps


class MissionManager():
    """ Mission Manager """

    def __init__(self,
                 client: WebSocketClientInterface,
                 uav_manager: UavManager,
                 logger: AerostackUILogger):

        self.client = client
        self.uav_manager = uav_manager
        self.logger = logger

        self.mission_id = 0
        self.mission_list = {}

        self.client.add_msg_callback(
            'request', 'missionConfirm', self.mission_confirm_callback)
        self.client.add_msg_callback(
            'request', 'missionPause', self.pause_resume_mission_callback, 'pause')
        self.client.add_msg_callback(
            'request', 'missionResume', self.pause_resume_mission_callback, 'resume')
        self.client.add_msg_callback(
            'request', 'missionStart', self.start_mission_callback)

    def mission_confirm_callback(self, msg: dict, args):
        """ Mission confirm callback """
        self.logger.debug("MissionManager", "mission_confirm_callback", f"Received mission confirm message: {msg}")

        confirm, extra = MissionInterpreter.interpreter(msg, self.logger)
        status = 'confirmed' if confirm else 'rejected'

        self.client.request_messages.mission_confirm(
            self.mission_id,
            status,
            msg['payload']['id'],
            msg['from'],
            extra
        )
        self.logger.debug("MissionManager", "mission_confirm_callback", "Sending mission response to client")

        if confirm:
            mission_planner_msg = MissionInterpreter.planner(
                self.mission_id,
                msg['payload']
            )

            mission_planner_msg['status'] = status

            self.logger.debug("MissionManager", "mission_confirm_callback", f"Mission confirmed, sending mission info: {mission_planner_msg}")
            self.client.info_messages.send_mission_info(mission_planner_msg)
            self.mission_list[self.mission_id] = mission_planner_msg

            self.mission_id += 1

    def pause_resume_mission_callback(self, msg: dict, args):
        """ Pause or resume mission callback """
        self.logger.debug("MissionManager", "pause_resume_mission_callback", f"Received pause/resume message: {msg}")
        # mission_id = str(msg['payload']['id'])
        # mission_list = self.mission.mission_list[mission_id]
        # # From mission list dict, extract keys
        # uav_list = mission_list.keys()
        # print(uav_list)
        # if args[0] == 'pause':
        #     print("Pausing mission")
        #     # TODO: pause uav_list
        # elif args[0] == 'resume':
        #     print("Resuming mission")
        #     # TODO: pause uav_list
        # else:
        #     print("Error, invalid argument")
        #     return

    def start_mission_callback(self, msg: dict, args):
        """ Start mission callback """
        self.logger.debug("MissionManager", "mission_confirm_callback", f"Received starting mission message: {msg}")
        print("1")
        print(msg)
        print(self.mission_list)
        return
        mission_id = msg['payload']['id']
        mission_list = self.mission_list[mission_id]

        self.thread_uav = {}
        for uav in mission_list['uavList']:
            print("2")
            print(uav)
            drone_interface = self.uav_manager.drones_interfaces[uav]

            if drone_interface == None:
                continue
            print("3")
            self.logger.info("MissionManager", "start_mission_callback", f"Starting mission for uav {uav}")
            print("4")
            self.thread_uav[uav] = threading.Thread(
                target=drone_interface.run_uav_mission,
                args=[mission_list[uav], self.thread_uav[uav]])
            print("5")
            self.thread_uav[uav].start()