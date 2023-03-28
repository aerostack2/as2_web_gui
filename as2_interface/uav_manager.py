"""
uav_interface.py
"""

import threading
from time import sleep
from typing import Callable
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


VIRTUAL_MODE = False


class UavInterface(DroneInterfaceBase, threading.Thread):
    """ UAV Interface """
    info_lock = threading.Lock()

    def __init__(self, drone_id: str, logger: AerostackUILogger, sim_mode: bool = False,
                 use_sim_time: bool = False, use_cartesian_coordinates: bool = False):

        self.logger = logger
        verbose = logger.get_log_level() >= 3
        self.use_cartesian_coordinates = use_cartesian_coordinates

        threading.Thread.__init__(self)

        if not VIRTUAL_MODE:
            DroneInterfaceBase.__init__(self,
                                    drone_id=drone_id,
                                    verbose=verbose,
                                    use_sim_time=use_sim_time)
            self.land = LandModule(drone=self)
            self.takeoff = TakeoffModule(drone=self)

            if self.use_cartesian_coordinates:
                self.go_to = GoToModule(drone=self)
                self.follow_path = FollowPathModule(drone=self)
            else:
                self.gps = GpsModule(drone=self)
                self.go_to = GoToGpsModule(drone=self)
                self.follow_path = FollowPathGpsModule(drone=self)

        else:
            self.drone_id_aux = drone_id

        self._stop_event = threading.Event()
        self._stop_event.clear()

        self._sim_mode = sim_mode
        self._yaw_mode = YawMode()
        self._yaw_mode.mode = YawMode.PATH_FACING
        self._yaw_mode.angle = 0.0

        self._mission = []

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

    def start_mission(self, mission: list) -> None:
        """ Start UAV mission """
        self._mission = mission
        self.daemon = True
        self.start()

    def __virtual_mission_status_change(self):
        """ Virtual mission status change """
        result = {}
        for behavior in self.modules:
            if isinstance(self.modules[behavior], BehaviorHandler):
                result[str(behavior)] = True
        return result


    def pause_mission(self):
        """ Pause UAV mission """
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        return DroneBehaviorManager.pause_all_behaviors(self)

    def resume_mission(self):
        """ Resume UAV mission """
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        return DroneBehaviorManager.resume_all_behaviors(self)

    def stop_mission(self):
        """ Stop UAV mission """
        self._stop_event.set()
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        return DroneBehaviorManager.stop_all_behaviors(self)

    def run(self):
        """ Run UAV mission thread """
        if VIRTUAL_MODE:
            for element in self._mission:
                if self._stop_event.is_set():
                    self.logger.info(
                        "UavInterface",
                        "run_uav_mission",
                        f"UAV {self.drone_id_aux}. Mission stopped")
                    return
                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id_aux}. Mission element: {element}")
                sleep(5.0)

        self.logger.debug(
            "UavInterface",
            "run_uav_mission",
            f"UAV {self.drone_id}. Running mission: {self._mission}")

        if self._sim_mode:
            self.logger.debug(
                "UavInterface",
                "run_uav_mission",
                f"UAV {self.drone_id}. Sim mode, arming")
            self.arm()
            self.logger.debug(
                "UavInterface",
                "run_uav_mission",
                f"UAV {self.drone_id}. Sim mode, offboard")
            self.offboard()

        for element in self._mission:
            if self._stop_event.is_set():
                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Mission stopped")
                return

            speed = float(element['speed'])

            if element['name'] == 'TakeOffPoint':
                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Takeoff height {element['values'][0][2]} \
                        and speed {speed}")
                self.takeoff(height=element['values'][0][2], speed=speed)

                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

            elif element['name'] == 'LandPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Land point {waypoint} and speed {speed}")
                self.go_to(*waypoint, speed, self._yaw_mode.mode,
                          self._yaw_mode.angle)

                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Land at speed {speed}")
                self.land()
                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Landed")

            elif element['name'] == 'Path':
                waypoints = element['values']

                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Follow path {waypoint} and speed {speed}")
                for waypoint in waypoints:
                    self.go_to(*waypoint, speed, self._yaw_mode.mode,
                              self._yaw_mode.angle)

            elif element['name'] == 'WayPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Go to {waypoint} and speed {speed}")
                self.go_to(*waypoint, speed, self._yaw_mode.mode,
                          self._yaw_mode.angle)

            elif element['name'] == 'Area':
                waypoints = element['values']
                self.logger.info(
                    "UavInterface",
                    "run_uav_mission",
                    f"UAV {self.drone_id}. Area path {waypoints} and speed {speed}")
                print("Area path: ", waypoints)
                self.follow_path(waypoints, speed, self._yaw_mode.mode, self._yaw_mode.angle)
                # for waypoint in waypoints:
                #     self.go_to(*waypoint, speed, self._yaw_mode.mode,
                #               self._yaw_mode.angle)

            else:
                # print("Unknown layer")
                # print("Element: ", element)
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        self.logger.info(
            "UavInterface",
            "run_uav_mission",
            f"UAV {self.drone_id}. Mission finished")


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
                            40.158194+idx*0.0001,
                            -3.3807955+idx*0.0001,
                            0.067396380007267,
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

            sleep(0.5)
        self.logger.error(
            "UavManager",
            "run",
            "Connection closed")
