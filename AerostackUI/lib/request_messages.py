""" Request messages module """
from .websocket_client import WebSocketClient
from .websocket_data import WebSocketClientData
from .websocket_logger import WebSocketClientLogger


class RequestMessages:
    """ Class for request messages """

    def __init__(self, websocket: WebSocketClient, data: WebSocketClientData,
                 logger: WebSocketClientLogger):
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
        print("-mission_confirm: Mission request callback")
        confirm_payload = {
            'id': new_mission_id,
            'status': status,
            'oldId': old_id,
            'author': author,
            'extra': extra,
        }
        self.__send('missionConfirm', confirm_payload, author)
