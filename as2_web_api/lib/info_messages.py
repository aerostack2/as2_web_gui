""" Info messages module """
from .websocket_client import WebSocketClient
from .websocket_data import WebSocketClientData
from .websocket_logger import WebSocketClientLogger


class InfoMessages:
    """ Class for info messages """

    def __init__(self, websocket: WebSocketClient, data: WebSocketClientData,
                 logger: WebSocketClientLogger):
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
