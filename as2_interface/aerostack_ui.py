"""
aerostack_ui.py
"""

from AerostackUI.websocket_interface import WebSocketClientInterface
from .mission_manager import MissionManager
from .uav_interface import UavInterface
import time
import threading


class AerostackUI():
    """ Aerostack UI """

    def __init__(self, uav_id_list: list, verbose: bool = False, sim_mode: bool = False,
                 use_sim_time: bool = False):
        self.client = WebSocketClientInterface(
            "ws://127.0.0.1:8000/ws/user/", verbose=False)

        self.mission_manager = MissionManager()

        self.client.add_msg_callback(
            'request', 'missionConfirm', self.mission_confirm_callback)
        self.client.add_msg_callback(
            'request', 'missionStart', self.start_mission_callback)

        self.uav_id_list = uav_id_list

        self.drone_interface = {}
        for uav_id in self.uav_id_list:
            drone_node = UavInterface(uav_id, verbose, sim_mode, use_sim_time)
            origin = [40.158194, -3.380795, 100]
            drone_node.gps.set_origin(origin)
            self.drone_interface[uav_id] = drone_node

        time.sleep(1)

        self.get_info_thread = threading.Thread(target=self.run)
        self.get_info_thread.start()

    def fake_mission(self, msg: dict):
        """ Fake mission """
        time.sleep(3)
        self.mission_confirm_callback(msg, [])

    def mission_confirm_callback(self, msg: dict, args):
        """ Mission confirm callback """
        confirm_msg = self.mission_manager.mission_interpreter(msg)

        self.client.request_messages.mission_confirm(
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

            print(f"Mission {str(confirm_msg['id'])} confirmed")

            mission_planner_msg['status'] = confirm_msg['status']
            self.client.info_messages.send_mission_info(mission_planner_msg)
        else:
            print(f"Mission {str(confirm_msg['id'])} reject")

    def start_mission_callback(self, msg: dict, args):
        """ Start mission callback """
        print(f"Starting mission {str(msg['payload']['id'])}")

        mission_id = str(msg['payload']['id'])
        mission_list = self.mission_manager.mission_list[mission_id]

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

        print("Running info publisher")
        # odom = {}
        # for uav in self.uav_id_list:
        #     odom[uav] = []

        while self.client.connection:
            for idx, uav in enumerate(self.uav_id_list):

                drone_interface_i = self.drone_interface[uav]

                send_info = drone_interface_i.get_info()

                if -90.0 <= send_info['pose']['lat'] <= 90.0 and \
                   -180.0 <= send_info['pose']['lng'] <= 180.0:
                    # if len(odom[uav]) > 50:
                    #     odom[uav].pop(0)

                    # odom[uav].append(
                    #     [send_info['pose']['lat'], send_info['pose']['lng']])
                    # send_info['odom'] = odom[uav]
                    self.client.info_messages.send_uav_info(send_info)
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
