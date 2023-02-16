"""
aerostack_ui.py
"""

import rclpy
from AerostackUI.websocket_interface import WebSocketClientInterface
from AerostackUI.aerostack_ui_logger import AerostackUILogger
from .mission_manager import MissionManager
from .uav_manager import UavManager


class AerostackUI():
    """ Aerostack UI """

    def __init__(self, uav_id_list: list, log_level: int = 0, sim_mode: bool = False,
                 use_sim_time: bool = False, use_cartesian_coordinates: bool = False):

        rclpy.init()
        self.logger = AerostackUILogger(log_level)

        self.client = WebSocketClientInterface(
            "ws://127.0.0.1:8000/ws/user/", self.logger)

        self.uav_manager = UavManager(uav_id_list, self.client, self.logger, sim_mode, use_sim_time, use_cartesian_coordinates)
        self.mission_manager = MissionManager(self.client, self.uav_manager, self.logger, use_cartesian_coordinates)

    def shutdown(self):
        """ Clean shutdown """
        self.uav_manager.shutdown()
        self.client.shutdown()
        rclpy.shutdown()
