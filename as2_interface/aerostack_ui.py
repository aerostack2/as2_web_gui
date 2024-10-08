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
