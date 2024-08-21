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

""" Info messages module """
from .websocket_client import WebSocketClient
from .websocket_data import WebSocketClientData
from AerostackUI.aerostack_ui_logger import AerostackUILogger


class InfoMessages:
    """ Class for info messages """

    def __init__(self, websocket: WebSocketClient, data: WebSocketClientData,
                 logger: AerostackUILogger):
        self.websocket = websocket
        self.data = data
        self.logger = logger

    def __send(self, header: str, payload: dict):
        """
        Send a info message to server

        Args:
            header (str): Header of the message.
            payload (dict): Payload of the message.
        """
        msg = {
            'type': 'info',
            'header': header,
            'payload': payload
        }
        self.websocket.send(self.data.client_id, msg, 'webpage')  # To all webpage clients

    def send_uav_info(self, uav_info: dict, override=False) -> None:
        """ Send UAV info message to server

        Args:
            uav_info (dict): UAV informatio to send, with the keys "id", "state" and "pose".
            override (bool, optional): flag to indicate if override server info. Defaults to False.

        Raises:
            Exception: If the UAV info does not contain the key "id".
        """
        if not 'id' in uav_info:
            raise Exception("UAV info must contain the key 'id'")

        uav_id = uav_info['id']

        # Check if first time this UAV info is send
        if not uav_id in self.data.uav_id_list:
            # Check if the first time the UAV info is send,
            # it contains the keys "id", "state" and "pose"
            if not ('id' in uav_info and 'state' in uav_info and 'pose' in uav_info):
                raise Exception(
                    'Invalid UAV info. For the firt time an UAV is send, \
                    it must contain the keys "id", "state" and "pose"')
            self.data.uav_id_list.append(uav_id)

        info_type = 'uavInfoSet' if override else 'uavInfo'
        self.__send(info_type, uav_info)

    def send_mission_info(self, mission_info, override=False) -> None:
        """ Send mission info message to server

        Args:
            mission_info (dict): Mission info to send, with the keys "id", "uavList" and "layers".
            override (bool, optional): flag to indicate if override server info. Defaults to False.

        Raises:
            Exception: If the mission info does not contain the key "id".
        """
        if not 'id' in mission_info:
            raise Exception("Mission info must contain the key 'id'")

        mission_id = mission_info['id']

        # Check if first time this mission info is send
        if not mission_id in self.data.mission_id_list:
            # Check if the first time the mission info is send,
            # it contains the keys "id", "uavList" and "layers"
            if not ('id' in mission_info and 'uavList' in mission_info and
                    'layers' in mission_info):
                raise Exception(
                    'Invalid Mission info. For the firt time a Mission is send, \
                    it must contain the keys "id", "uavList" and "layers"')
            self.data.mission_id_list.append(mission_id)

        info_type = 'missionInfoSet' if override else 'missionInfo'
        self.__send(info_type, mission_info)
