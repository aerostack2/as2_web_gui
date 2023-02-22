""" Basic messages module """
from .websocket_client import WebSocketClient
from .websocket_data import WebSocketClientData
from AerostackUI.aerostack_ui_logger import AerostackUILogger


class BasicMessages:
    """ Class for basic messages """

    def __init__(self, websocket: WebSocketClient, data: WebSocketClientData,
                 logger: AerostackUILogger):
        self.websocket = websocket
        self.data = data
        self.logger = logger

    def __send(self, header: str, payload: dict) -> None:
        """
        Send a basic message to server

        Args:
            header (str): Header of the message
            payload (dict): Payload of the message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        msg = {
            'type': 'basic',
            'header': header,
            'status': 'request',
            'payload': payload
        }
        self.websocket.send(self.data.client_id, msg, 'server')  # To server

    def handshake(self, rol: str) -> None:
        """
        Send handshake message to server
        """
        payload = {'rol': rol}
        self.__send('handshake', payload)

    def get_id(self, client_id: int) -> None:
        """
        Send getId message to server
        """
        payload = {'id': client_id}
        self.__send('getId', payload)

    def ping(self):
        """
        Send ping message to server
        """
        payload = {}
        self.__send('ping', payload)

    def get_client_list(self) -> None:
        """
        Send getClientList message to server
        """
        payload = {}
        self.__send('getClientsList', payload)

    def message_proccess(self, msg: dict) -> None:
        """
        Process a basic message

        Args:
            msg (dict): Message to process
        """
        if msg['header'] == 'handshake':
            if msg['payload']['response'] == 'success':
                self.logger.debug("BasicMessages",
                    "on_message", f"Logged in with id {msg['payload']['id']}")
                self.data.client_id = msg['payload']['id']
            else:
                self.logger.error("BasicMessages",
                    "on_message", f"Login failed with error {msg['response']['response']}")
                self.websocket.close()

        elif msg['header'] == 'getId':
            self.data.client_id = msg['payload']['id']
            self.logger.debug("BasicMessages",
                "on_message", f"Received id {self.data.client_id}")

        elif msg['header'] == 'ping':
            self.logger.debug("BasicMessages",
                "on_message", f"Ping: {msg['payload']['message']}")

        elif msg['header'] == 'getClientList':
            self.logger.debug("BasicMessages",
                "on_message", f"Client list: {msg['payload']['client_list']}")
