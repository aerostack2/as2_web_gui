"""
uav_interface.py
"""

from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from typing import Callable, List
import time
import threading


class UavInterface(DroneInterfaceGPS):
    """ UAV Interface """
    info_lock = threading.Lock()

    def __init__(self, uav_id: str, sim_mode: bool = False):
        self.drone_interface = super(UavInterface, self)
        self.drone_interface.__init__(uav_id, verbose=False)
        self.uav_id = uav_id
        self.sim_mode = sim_mode

    def info_lock_decor(func: Callable) -> Callable:
        def wrapper(self, *args, **kwargs):
            with self.info_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @info_lock_decor
    def get_info(self):
        """ Get info """
        gps_pose = self.gps.pose
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

    def run_uav_mission(self, mission: list, thread: threading.Thread, yaw_mode: bool = True) -> None:
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
                drone_interface.go_to_gps_point(waypoint, speed, yaw_mode)
                # print(f"{uav} - Takeoff waypoint done")

            elif element['name'] == 'LandPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                # print(f"{uav} - Send land point")
                # drone_interface.follow_gps_path([waypoint])
                drone_interface.go_to_gps_point(waypoint, speed, yaw_mode)
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
                        [wp[0], wp[1], wp[2]], speed, yaw_mode)
                # print(f"{uav} - Path done")

            elif element['name'] == 'WayPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                # print(f"Send waypoint: {waypoint}")
                # drone_interface.follow_gps_wp([waypoint], speed)
                drone_interface.go_to_gps_point(waypoint, speed, yaw_mode)
                time.sleep(2.0)  # TODO: Remove this
                # print(f"{uav} - Waypoint done")

            elif element['name'] == 'Area':
                waypoint = element['values']
                # print(f"{uav} - Send area")
                # drone_interface.follow_gps_path(waypoint[1:], speed)

                for wp in waypoint[1:]:
                    drone_interface.go_to_gps_point(
                        [wp[0], wp[1], wp[2]], speed, yaw_mode)
                # print(f"{uav} - Area done")

            else:
                # print("Unknown layer")
                # print("Element: ", element)
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        if thread is not None:
            thread.join()
