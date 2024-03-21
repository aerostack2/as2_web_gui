"""
uav_interface.py
"""

import threading
import json
from math import tan ,radians
from time import sleep
from typing import Callable
from rclpy.qos import qos_profile_system_default, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from as2_msgs.msg import MissionUpdate
from as2_python_api.mission_interpreter.mission import MissionItem, Mission, InterpreterStatus
from as2_python_api.drone_interface_gps import DroneInterfaceBase
from as2_python_api.behavior_manager.behavior_manager import DroneBehaviorManager
from as2_python_api.modules.land_module import LandModule
from as2_python_api.modules.takeoff_module import TakeoffModule
from as2_python_api.modules.go_to_module import GoToModule
from as2_python_api.modules.follow_path_module import FollowPathModule
from as2_python_api.modules.gps_module import GpsModule
from as2_python_api.modules.go_to_gps_module import GoToGpsModule
from as2_python_api.modules.follow_path_gps_module import FollowPathGpsModule
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_msgs.msg import YawMode
from AerostackUI.websocket_interface import WebSocketClientInterface
from AerostackUI.aerostack_ui_logger import AerostackUILogger


VIRTUAL_MODE = True
GPS_COORDINATES = [40.337236, -3.886678, 0.01]
YAW_ANGLE = radians(135.0) # 135.0º
GIMBAL_ANGLE = -60.0


class UavInterface(DroneInterfaceBase):
    """ UAV Interface """
    info_lock = threading.Lock()

    def __init__(self, drone_id: str, logger: AerostackUILogger, sim_mode: bool = False,
                 use_sim_time: bool = False, use_cartesian_coordinates: bool = False):

        self.logger = logger
        verbose = logger.get_log_level() >= 3
        self.use_cartesian_coordinates = use_cartesian_coordinates

        if not VIRTUAL_MODE:
            DroneInterfaceBase.__init__(self,
                                        drone_id=drone_id,
                                        verbose=verbose,
                                        use_sim_time=use_sim_time)
            if not self.use_cartesian_coordinates:
                self.gps = GpsModule(drone=self)

        else:
            self.drone_id_aux = drone_id

        self._sim_mode = sim_mode
        self._yaw_mode = YawMode()
        self._yaw_mode.mode = YawMode.FIXED_YAW
        self._yaw_mode.angle = YAW_ANGLE

        # ROS 2 Mission interpreter
        if not VIRTUAL_MODE:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )

            self.mission_update_pub = self.create_publisher(
                MissionUpdate, 'mission_update', qos_profile_system_default)

            self.mission_status_sub = self.create_subscription(
                String, 'mission_status', self.mission_status_callback,
                qos_profile)

        self.missions = {}
        self.mission_status = "IDLE"

    def info_lock_decor(func: Callable) -> Callable:
        """ Decorator for info lock """

        def wrapper(self, *args, **kwargs):
            with self.info_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @info_lock_decor
    def get_info(self):
        """ Get info """
        info = {
            'id': self.drone_id,
            'state': self.info,
        }
        if self.use_cartesian_coordinates:
            info['pose'] = [
                self.position[0],
                self.position[1]
            ]
        else:
            info['pose'] = [
                self.gps.pose[0],
                self.gps.pose[1]
            ]
        info['pose'].append(self.position[2])
        info['pose'].append(self.orientation[2])
        return info

    def start_mission(self, mission_id: int) -> None:
        """ Start UAV mission """
        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.mission_id = mission_id
        mission_update.action = MissionUpdate.START

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def __virtual_mission_status_change(self):
        """ Virtual mission status change """
        result = {}
        for behavior in self.modules:
            if isinstance(self.modules[behavior], BehaviorHandler):
                result[str(behavior)] = True
        return result

    def pause_mission(self) -> None:
        """ Pause UAV mission """
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        # return DroneBehaviorManager.pause_all_behaviors(self)
        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.action = MissionUpdate.PAUSE

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def resume_mission(self) -> None:
        """ Resume UAV mission """
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        # return DroneBehaviorManager.resume_all_behaviors(self)
        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.action = MissionUpdate.RESUME

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def stop_mission(self) -> None:
        """ Stop UAV mission """
        # self._stop_event.set()
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        # return DroneBehaviorManager.stop_all_behaviors(self)

        mission = Mission(target=self.namespace, verbose=True)
        mission.plan.append(MissionItem(behavior='rtl', args={
            'height': 15.0,
            'speed': 3.0,
            'land_speed': 1.0,
            'wait': True
        }))

        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.action = MissionUpdate.LOAD
        mission_update.mission_id = 10
        mission_update.mission = mission.json()

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def load_mission(self, mission_id: int, mission_list: list) -> None:
        """ Load mission """

        # Mission
        mission = Mission(target=self.namespace, verbose=True)

        for element in mission_list:
            speed = float(element['speed'])

            if element['name'] == 'TakeOffPoint':
                mission.plan.append(MissionItem(behavior='takeoff', args={
                    'height': element['values'][0][2], 'speed': speed, 'wait': True
                }))

                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                mission.plan.append(MissionItem(behavior='go_to_gps', args={
                    'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                    'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))
                if GIMBAL_ANGLE != 0.0:
                    x = 1.0
                    z = x * tan(radians(GIMBAL_ANGLE))
                    mission.plan.append(MissionItem(
                        behavior='point_gimbal',
                        args={
                            '_x': x, '_y': 0.0, '_z': z, 'frame_id': f"{self.namespace}/base_link",
                            'wait': True
                    }))

            elif element['name'] == 'LandPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                mission.plan.append(MissionItem(behavior='go_to_gps', args={
                    'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                    'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))
                if GIMBAL_ANGLE != 0.0:
                    mission.plan.append(MissionItem(
                        behavior='point_gimbal',
                        args={
                            '_x': 1.0, '_y': 0.0, '_z': 0.0,
                            'frame_id': f"{self.namespace}/base_link",
                            'wait': True
                    }))
                mission.plan.append(MissionItem(
                    behavior='land', args={'speed': speed}))

            elif element['name'] == 'Path':
                waypoints = element['values']
                for waypoint in waypoints:
                    # mission.plan.append(MissionItem(behavior='go_to', args={
                    #     '_x': waypoint[0], '_y': waypoint[1], '_z': waypoint[2],
                    #     'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    #     'yaw_angle': self._yaw_mode.angle, 'frame_id': 'earth', 'wait': True
                    # }))
                    mission.plan.append(MissionItem(behavior='go_to_gps', args={
                        'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                        'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                        'yaw_angle': self._yaw_mode.angle, 'wait': True
                    }))

            elif element['name'] == 'WayPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                mission.plan.append(MissionItem(behavior='go_to_gps', args={
                    'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                    'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))

            elif element['name'] == 'Area':
                waypoints = element['values']
                mission.plan.append(MissionItem(behavior='follow_path_gps', args={
                    'geopath': waypoints, 'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))
            else:
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.mission_id = mission_id
        mission_update.mission = mission.json()
        mission_update.action = MissionUpdate.LOAD

        # Publish the mission
        self.mission_update_pub.publish(mission_update)
        self.mission_status = "LOAD"

    def mission_status_callback(self, msg: String) -> None:
        """ Mission status callback """
        self.logger.debug(
            "UavInterface",
            "mission_status_callback",
            f"Mission status: {msg.data}")
        dict_data = json.loads(msg.data)
        # Check if dict has the key 'id' and 'status'
        if 'id' in dict_data and 'status' in dict_data:
            self.missions[dict_data['id']] = dict_data['status']


class UavManager():
    """ UAV Manager """

    def __init__(self,
                 uav_id_list: list,
                 client: WebSocketClientInterface,
                 logger: AerostackUILogger,
                 sim_mode: bool = False,
                 use_sim_time: bool = False,
                 use_cartesian_coordinates: bool = False):

        self.client = client
        self.logger = logger
        self.use_cartesian_coordinates = use_cartesian_coordinates

        self.publish_odom = False
        self.uav_id_list = uav_id_list

        self.missions = {}
        self.drones_interfaces = {}
        for uav_id in uav_id_list:
            self.drones_interfaces[uav_id] = UavInterface(
                uav_id, logger, sim_mode, use_sim_time, use_cartesian_coordinates)
            # origin = [40.158194, -3.3805597, 830]
            # drone_node.gps.set_origin(origin)

        self.get_info_thread = threading.Thread(target=self.run)
        self.get_info_thread.start()

    def shutdown(self):
        """ Shutdown """
        self.get_info_thread.join()
        for uav_id in self.uav_id_list:
            self.drones_interfaces[uav_id].shutdown()

    def start_mission(self, uav_list: list, mission_id: int) -> bool:
        """ Start """
        result = {}
        for uav in uav_list:
            result[uav] = self.drones_interfaces[uav].start_mission(mission_id)
        return result

    def stop_mission(self, uav_list: list) -> bool:
        """ Stop """
        result = {}
        for uav in uav_list:
            result[uav] = self.drones_interfaces[uav].stop_mission()
        return result

    def pause_mission(self, uav_list: list) -> bool:
        """ Pause """
        result = {}
        for uav in uav_list:
            result[uav] = self.drones_interfaces[uav].pause_mission()
        return result

    def resume_mission(self, uav_list: list) -> bool:
        """ Resume """
        result = {}
        for uav in uav_list:
            result[uav] = self.drones_interfaces[uav].resume_mission()
        return result

    def update_mission_status(self) -> None:
        """ Update mission status """
        self.missions = {}
        for uav in self.uav_id_list:
            missions_status = self.drones_interfaces[uav].missions
            for mission_id in missions_status:
                if self.missions[mission_id] is None:
                    self.missions[mission_id] = missions_status[mission_id]
                else:
                    self.missions[mission_id] = \
                        f"{self.missions[mission_id]}/{missions_status[mission_id]}"

    def run(self):
        """ Run """

        print("Running info publisher")

        if self.publish_odom:
            odom = {}
            for uav in self.uav_id_list:
                odom[uav] = []

        while self.client.connection:
            for idx, uav in enumerate(self.uav_id_list):

                if VIRTUAL_MODE:
                    sleep(1.0)
                    pose = []
                    if self.use_cartesian_coordinates:
                        pose = [1.0+idx, 1.0+idx, 0.0, 0.0]
                    else:
                        pose = [
                            GPS_COORDINATES[0]+idx*0.0001,
                            GPS_COORDINATES[1]+idx*0.0001,
                            GPS_COORDINATES[2],
                            -0.00735415557174667]
                    self.client.info_messages.send_uav_info({
                        'id': uav,
                        'state': {
                            "connected": True,
                            "armed": True,
                            "offboard": True,
                            "state": 0,
                            "yaw_mode": 0,
                            "control_mode": 0,
                            "reference_frame": 0
                        },
                        'pose': pose,
                    })
                    sleep(1.0)
                    continue

                drone_interface_i = self.drones_interfaces[uav]
                send_info = drone_interface_i.get_info()

                if -90.0 <= send_info['pose'][0] <= 90.0 and \
                   -180.0 <= send_info['pose'][1] <= 180.0:

                    if self.publish_odom:
                        if len(odom[uav]) > 50:
                            odom[uav].pop(0)
                        odom[uav].append(
                            [send_info['pose'][0], send_info['pose'][1]])
                        send_info['odom'] = odom[uav]

                    self.client.info_messages.send_uav_info(send_info)

                else:
                    self.logger.debug(
                        "UavManager",
                        "run",
                        f"Error in pose values: {send_info['pose']}")

            self.update_mission_status()
            for mission_id in self.missions:
                self.client.info_messages.send_mission_info({
                    'id': mission_id,
                    'status': self.missions[mission_id]})

            sleep(0.5)
        self.logger.error(
            "UavManager",
            "run",
            "Connection closed")
