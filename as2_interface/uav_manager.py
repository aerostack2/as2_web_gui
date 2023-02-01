"""
uav_interface.py
"""

from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from typing import Callable, List
from as2_msgs.msg import YawMode
from time import sleep
import threading
from AerostackUI.websocket_interface import WebSocketClientInterface
from AerostackUI.aerostack_ui_logger import AerostackUILogger


class UavInterface(DroneInterfaceGPS):
    """ UAV Interface """
    info_lock = threading.Lock()

    def __init__(self, drone_id: str, logger: AerostackUILogger, sim_mode: bool = False,
                 use_sim_time: bool = False):
        self.logger = logger
        verbose = logger.get_log_level() >= 3
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)
        self.sim_mode = sim_mode
        self.yaw_mode = YawMode()
        self.yaw_mode.mode = YawMode.PATH_FACING
        self.yaw_mode.angle = 0.0

    def info_lock_decor(func: Callable) -> Callable:
        def wrapper(self, *args, **kwargs):
            with self.info_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @info_lock_decor
    def get_info(self):
        """ Get info """
        return {
            'id': self.drone_id,
            'state': self.info,
            'pose': [self.gps.pose[0],
                     self.gps.pose[1],
                     self.position[2],
                     self.orientation[2]],
        }

    def run_uav_mission(self, mission: list, thread: threading.Thread) -> None:
        """ Run UAV mission """

        self.logger.debug("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Running mission: {mission}")

        if self.sim_mode:
            self.logger.debug("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Sim mode, arming")
            self.arm()
            self.logger.debug("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Sim mode, offboard")
            self.offboard()

        for element in mission:
            speed = float(element['speed'])

            if element['name'] == 'TakeOffPoint':
                self.logger.info("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Takeoff height {element['values'][0][2]} and speed {speed}")
                self.takeoff(height=element['values'][0][2], speed=speed)

                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                # self.goto(*waypoint, speed, self.yaw_mode.mode, self.yaw_mode.angle)

            elif element['name'] == 'LandPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                self.logger.info("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Land point {waypoint} and speed {speed}")
                self.goto(*waypoint, speed, self.yaw_mode.mode,
                          self.yaw_mode.angle)
                
                self.logger.info("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Land at speed {speed}")
                self.land()
                self.logger.info("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Landed")

            elif element['name'] == 'Path':
                waypoints = element['values']

                self.logger.info("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Follow path {waypoint} and speed {speed}")
                # self.follow_path(waypoints, speed, self.yaw_mode.mode, self.yaw_mode.angle)

                for waypoint in waypoints:
                    self.goto(*waypoint, speed, self.yaw_mode.mode,
                              self.yaw_mode.angle)

            elif element['name'] == 'WayPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                self.logger.info("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Goto {waypoint} and speed {speed}")
                self.goto(*waypoint, speed, self.yaw_mode.mode,
                          self.yaw_mode.angle)

            elif element['name'] == 'Area':
                waypoints = element['values']
                self.logger.info("UavInterface", "run_uav_mission", f"UAV {self.drone_id}. Area path {waypoints} and speed {speed}")
                # self.follow_path(waypoints, speed, self.yaw_mode.mode, self.yaw_mode.angle)

                for waypoint in waypoints:
                    self.goto(*waypoint, speed, self.yaw_mode.mode,
                              self.yaw_mode.angle)

            else:
                # print("Unknown layer")
                # print("Element: ", element)
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        if thread is not None:
            thread.join()


class UavManager():
    """ UAV Manager """

    def __init__(self,
                 uav_id_list: list,
                 client: WebSocketClientInterface,
                 logger: AerostackUILogger,
                 sim_mode: bool = False,
                 use_sim_time: bool = False):

        self.client = client
        self.logger = logger

        self.publish_odom = False
        self.uav_id_list = uav_id_list

        self.drones_interfaces = {}
        for uav_id in uav_id_list:
            self.drones_interfaces[uav_id] = UavInterface(
                uav_id, logger, sim_mode, use_sim_time)
            # origin = [40.158194, -3.3805597, 830]
            # drone_node.gps.set_origin(origin)

        self.get_info_thread = threading.Thread(target=self.run)
        self.get_info_thread.start()

    def shutdown(self):
        """ Shutdown """
        self.get_info_thread.join()
        for uav_id in self.uav_id_list:
            self.drones_interfaces[uav_id].shutdown()

    def pause_behaviors(self, uav_list: list) -> bool:
        """ Pause """
        for uav in uav_list:
            # self.drones_interfaces[uav].pause()
            pass
        return True

    def resume_behaviors(self, uav_list: list) -> bool:
        """ Pause """
        for uav in uav_list:
            # self.drones_interfaces[uav].pause()
            pass
        return True

    def run(self):
        """ Run """

        print("Running info publisher")

        if self.publish_odom:
            odom = {}
            for uav in self.uav_id_list:
                odom[uav] = []

        while self.client.connection:
            for idx, uav in enumerate(self.uav_id_list):

                # info_collection = {
                #     'id': 'drone_sim_rafa_0',
                #     'state': {
                #         "connected": True,
                #         "armed": True,
                #         "offboard": True,
                #         "state": 0,
                #         "yaw_mode": 0,
                #         "control_mode": 0,
                #         "reference_frame": 0
                #     },
                #     'pose': [40.158194, -3.3807955, 0.067396380007267, -0.00735415557174667],
                # }

                # info_collection2 = {
                #     'id': 'drone_sim_rafa_1',
                #     'state': {
                #         "connected": True,
                #         "armed": True,
                #         "offboard": True,
                #         "state": 0,
                #         "yaw_mode": 0,
                #         "control_mode": 0,
                #         "reference_frame": 0
                #     },
                #     'pose': [40.158394, -3.3809955, 0.067396380007267, -0.00735415557174667],
                # }
                
                # while self.client.connection:
                #     self.client.info_messages.send_uav_info(info_collection)
                #     self.logger.debug("UavManager", "run",
                #                     f"Info_collection= {info_collection}")
                #     self.client.info_messages.send_uav_info(info_collection2)
                #     self.logger.debug("UavManager", "run",
                #                     f"Info_collection= {info_collection2}")
                #     sleep(20.0)
                # return

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
                    print("Error sending info for ", uav)

            sleep(0.5)
        self.logger.error("UavManager", "run", "Connection closed")
