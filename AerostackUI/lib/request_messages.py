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

""" Request messages module """
from .websocket_client import WebSocketClient
from .websocket_data import WebSocketClientData
from AerostackUI.aerostack_ui_logger import AerostackUILogger


class RequestMessages:
    """ Class for request messages """

    def __init__(self, websocket: WebSocketClient, data: WebSocketClientData,
                 logger: AerostackUILogger):
        self.websocket = websocket
        self.data = data
        self.logger = logger

    def __send(self, header: str, payload: dict, mgs_receptor: int) -> None:
        """
        Send a info message to server

        Args:
            header (str): Header of the message.
            payload (dict): Payload of the message.
        """
        msg = {
            'type': 'request',
            'header': header,
            'status': 'response',
            'payload': payload
        }
        self.websocket.send(self.data.client_id, msg, mgs_receptor)  # To all webpage clients

    def mission_confirm(self, new_mission_id: int, status: str, old_id: int,
                        author: int, extra: list = []) -> None:
        """ Send mission confirm message to server

        Args:
            new_mission_id (int): id of the new mission.
            status (str): status of the mission.
            oldId (int): id of the mission requested.
            author (int): id of the client that requested the mission.
            extra (list, optional): list of extra information. Defaults to [].
        """
        payload = {
            'id': new_mission_id,
            'status': status,
            'oldId': old_id,
            'author': author,
            'extra': extra,
        }
        self.__send('missionConfirm', payload, author)

    def mission_status(self, msg_header: str, mission_id: int, status: str,
                        author: int, extra: list = []) -> None:
        """ Send mission status change message to server

        Args:
            new_mission_id (int): id of the new mission.
            status (str): status of the mission.
            oldId (int): id of the mission requested.
            author (int): id of the client that requested the mission.
            extra (list, optional): list of extra information. Defaults to [].
        """
        payload = {
            'id': mission_id,
            'status': status,
            'author': author,
            'extra': extra,
        }
        print("Mission status msgs: ")
        print(payload)
        self.__send(msg_header, payload, author)
