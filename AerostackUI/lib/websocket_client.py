""" Websocket client to communicate with server """
import json
import websocket
from AerostackUI.aerostack_ui_logger import AerostackUILogger


class WebSocketClient:
    """
    Websocket client interface to communicate with server
    """

    def __init__(self, host: str, logger: AerostackUILogger, on_open: object = None,
                 on_message: object = None, on_error: object = None, on_close: object = None):
        self.host = host
        self.logger = logger
        self.msg_id = 0

        websocket.enableTrace(False)
        self.websocket = websocket.WebSocketApp(self.host,
                                                on_open=on_open,
                                                on_message=on_message,
                                                on_error=on_error,
                                                on_close=on_close)

    def send(self, client_id, msg: dict, cliend_id_destination: int = None):
        """
        Send a JSON-decoded message to server

        Args:
            msg (dict): JSON-decoded message
            to (int, optional): Id of the destination client. Defaults to None.
        """

        msg['msg_id'] = self.msg_id
        self.msg_id += 1

        if client_id is not None:
            msg['from'] = client_id
            if cliend_id_destination is None:
                msg['to'] = 0  # Server id
            else:
                msg['to'] = cliend_id_destination

        message = json.dumps({'message': msg})
        self.logger.debug("WebSocketClient", "send", f"Sending message: {message}")
        self.websocket.send('%s' % message)
