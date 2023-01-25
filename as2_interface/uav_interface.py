"""
uav_interface.py
"""

from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from typing import Callable, List
from as2_msgs.msg import YawMode
import time
import threading


class UavInterface(DroneInterfaceGPS):
    """ UAV Interface """
    info_lock = threading.Lock()

    def __init__(self, drone_id: str, verbose: bool = False, sim_mode: bool = False,
                 use_sim_time: bool = False):
        # self.drone_interface = super(UavInterface, self)
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)
        # self.drone_interface.__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)
        self.sim_mode = sim_mode
        self.yaw_mode = YawMode()
        self.yaw_mode.mode = YawMode.KEEP_YAW
        self.yaw_mode.angle = 0.0

    def info_lock_decor(func: Callable) -> Callable:
        def wrapper(self, *args, **kwargs):
            with self.info_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @info_lock_decor
    def get_info(self):
        """ Get info """
        gps_pose = self.gps.pose
        orientation = self.orientation
        height = self.position[2]
        id = self.drone_id
        info = self.info

        info_collection = {
            'id': id,
            'state': info,
            'pose': {'lat': gps_pose[0], 'lng': gps_pose[1], 'height': height,
                     'yaw': orientation[2]},
        }
        return info_collection

    def run_uav_mission(self, mission: list, thread: threading.Thread) -> None:
        """ Run UAV mission """

        print("Running mission: ")
        print(mission)

        if self.sim_mode:
            print(f"{self.namespace}: Arming")
            self.arm()
            print(f"{self.namespace}: Offboard")
            self.offboard()

        for element in mission:
            speed = float(element['speed'])

            if element['name'] == 'TakeOffPoint':
                print(f"{self.namespace}: Takeoff")
                self.takeoff(height=element['values'][0][2], speed=speed)
                time.sleep(10.0)
                
                print(f"{self.namespace}: Goto takeoff point")
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                self.goto(*waypoint, speed, self.yaw_mode.mode, self.yaw_mode.angle)

            elif element['name'] == 'LandPoint':
                print(f"{self.namespace}: Goto land point")
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                self.goto(*waypoint, speed, self.yaw_mode.mode, self.yaw_mode.angle)

                print(f"{self.namespace}: Land")
                self.land()
                print(f"{self.namespace}: Landed")

            elif element['name'] == 'Path':
                print(f"{self.namespace}: Follow path")
                waypoints = element['values']
                self.follow_path(waypoints, speed, self.yaw_mode.mode, self.yaw_mode.angle)

                # for wp in waypoints:
                #     self.goto(*wp, speed, self.yaw_mode.mode, self.yaw_mode.angle)

            elif element['name'] == 'WayPoint':
                print(f"{self.namespace}: Goto")
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                self.goto(*waypoint, speed, self.yaw_mode.mode, self.yaw_mode.angle)

            elif element['name'] == 'Area':
                print(f"{self.namespace}: Area")
                waypoints = element['values']
                self.follow_path(waypoints, speed, self.yaw_mode.mode, self.yaw_mode.angle)

                # for wp in waypoints:
                #     self.goto(*wp, speed, self.yaw_mode.mode, self.yaw_mode.angle)

            else:
                # print("Unknown layer")
                # print("Element: ", element)
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        if thread is not None:
            thread.join()
