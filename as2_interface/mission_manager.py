# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Rafael Pérez Seguí'

"""
mission_manager.py
"""

import copy
import numpy as np
from swarm_pylib.swarm_pylib import Swarm
from AerostackUI.websocket_interface import WebSocketClientInterface
from AerostackUI.aerostack_ui_logger import AerostackUILogger
from .uav_manager import UavManager


class MissionInterpreter():
    """ Mission Interpreter """
    use_cartesian_coordinates = False

    @staticmethod
    def interpreter(msg: dict, logger: AerostackUILogger) -> dict:
        """ Mission interpreter """
        logger.debug("MissionManager", "interpreter",
                     f"Interpreting mission: {msg}")

        if msg['payload']['status'] != 'request':
            message_info = f"Invalid mission status: \
                {msg['payload']['status']}, only 'request' is allowed for now"
            logger.info("MissionManager", "interpreter", message_info)

        if msg['payload']['id'] != 'New Mission':
            message_info = f"Invalid mission id: \
                {msg['payload']['id']}, only 'New Mission' is allowed for now"
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
    def planner(mission_id: str, mission_info: dict, use_cartesian_coordinates: bool = False) -> dict:
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
                layer, mission, last_position, first_uav, mission_info, use_cartesian_coordinates)

            for uav in new_last_position:
                last_position[uav] = new_last_position[uav]

            send_mission['layers'].append(send_layer)

        return send_mission, mission

    @staticmethod
    def layer_interpreter(layer: dict, mission: dict, last_position: dict,
                          first_uav: str, mission_info: dict, use_cartesian_coordinates: bool = False) -> tuple:
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
                        [point[0], point[1], height])
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
                sublist, mission, last_position, first_uav, mission_info, use_cartesian_coordinates)

            # area_path = MissionInterpreter.swarm_planning(
            #     uav_list_aux, last_position, next_position, height,
            #     values, str(algorithm), float(
            #         street_spacing), float(wp_space),
            #     theta)

            uavs_state = {}
            for uav in uav_list_aux:
                uavs_state[uav] = {
                    'initial_position': last_position[uav],
                    'last_position': next_position[uav]}

            area = []
            for point in values:
                area.append([point[0], point[1], height])

            if use_cartesian_coordinates:
                uavs_path = Swarm.swarm_planning(
                    uavs_state,
                    area,
                    str(algorithm),
                    'binpat',
                    float(street_spacing),
                    float(wp_space),
                    theta,
                    filter_path_waypoints=True)
            else:
                uavs_path = Swarm.swarm_planning_gps(
                    uavs_state,
                    area,
                    str(algorithm),
                    'binpat',
                    float(street_spacing),
                    float(wp_space),
                    theta,
                    filter_path_waypoints=True)

            area_path = {}
            for uav_path, uav in zip(uavs_path, uav_list_aux):
                area_path[uav] = uav_path

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
                          first_uav: str, mission_info: dict, use_cartesian_coordinates: bool = True) -> dict:
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


class MissionManager():
    """ Mission Manager """

    def __init__(self,
                 client: WebSocketClientInterface,
                 uav_manager: UavManager,
                 logger: AerostackUILogger,
                 use_cartesian_coordinates: bool = False):

        self.client = client
        self.uav_manager = uav_manager
        self.logger = logger
        self.use_cartesian_coordinates = use_cartesian_coordinates

        self.mission_id = 0
        self.mission_list = {}
        self.mission_status = {}
        self.uav_mission_run_threads = {}

        self.client.add_msg_callback(
            'request', 'missionConfirm', self.mission_confirm_callback)
        # self.client.add_msg_callback(
        #     'request', 'missionStart', self.start_mission_callback)
        self.client.add_msg_callback(
            'request', 'missionStart', self.change_mission_status_callback, 'start')
        self.client.add_msg_callback(
            'request', 'missionPause', self.change_mission_status_callback, 'pause')
        self.client.add_msg_callback(
            'request', 'missionResume', self.change_mission_status_callback, 'resume')
        self.client.add_msg_callback(
            'request', 'missionStop', self.change_mission_status_callback, 'stop')

    def mission_confirm_callback(self, msg: dict, args):
        """ Mission confirm callback """
        self.logger.debug(
            "MissionManager",
            "mission_confirm_callback",
            f"Received mission confirm message: {msg}")

        confirm, extra = MissionInterpreter.interpreter(msg, self.logger)
        status = 'confirmed' if confirm else 'rejected'

        self.client.request_messages.mission_confirm(
            self.mission_id,
            status,
            msg['payload']['id'],
            msg['from'],
            extra)

        self.logger.debug(
            "MissionManager",
            "mission_confirm_callback",
            "Sending mission response to client")

        if confirm:
            self.mission_status[self.mission_id] = 'confirmed'

            mission_planner_msg, mission = MissionInterpreter.planner(
                self.mission_id,
                msg['payload'],
                self.use_cartesian_coordinates
            )

            mission_planner_msg['status'] = status

            self.logger.debug(
                "MissionManager",
                "mission_confirm_callback",
                f"Mission confirmed, sending mission info: {mission_planner_msg}")
            self.client.info_messages.send_mission_info(mission_planner_msg)
            self.mission_list[self.mission_id] = mission

            for uav in mission:
                drone_interface = self.uav_manager.drones_interfaces[uav]
                if drone_interface is None:
                    continue
                drone_interface.load_mission(self.mission_id, mission[uav])
            self.mission_id += 1

    def change_mission_status_callback(self, msg: dict, args):
        """ Start mission callback """
        self.logger.debug(
            "MissionManager",
            "change_mission_status_callback",
            f"Received mission message: {msg}")

        mission_id = msg['payload']['id']
        uav_list = self.mission_list[mission_id].keys()
        status = ""
        # result = None
        desired_status = str(args[0])
        if desired_status == 'start':
            result = self.uav_manager.start_mission(uav_list, mission_id)
            status = 'running'
        elif desired_status == 'pause':
            result = self.uav_manager.pause_mission(uav_list)
            status = 'paused'
        elif desired_status == 'resume':
            result = self.uav_manager.resume_mission(uav_list)
            status = 'resumed'
        elif desired_status == 'stop':
            result = self.uav_manager.stop_mission(uav_list)
            status = 'cancelled'
        else:
            self.logger.error(
                "MissionManager",
                "change_mission_status_callback",
                f"Invalid mission status: {args}")
            return

        self.client.request_messages.mission_status(
            msg['header'],
            msg['payload']['id'],
            status,
            msg['from'],
            [])
