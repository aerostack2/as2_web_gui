""" Websocket client to communicate with server """
import threading
import json
import websocket
from .lib.basic_messages import BasicMessages
from .lib.info_messages import InfoMessages
from .lib.request_messages import RequestMessages
from .lib.websocket_client import WebSocketClient
from .lib.websocket_data import WebSocketClientData
from .lib.websocket_logger import WebSocketClientLogger


class WebSocketClientInterface:
    """
    Manager websocket client
    """

    def __init__(self, host, verbose: bool = True):
        self.host = host

        self.data = WebSocketClientData()

        self.logger = WebSocketClientLogger(verbose)
        self.websocket = WebSocketClient(
            self.host, self.logger, self.on_open,
            self.on_message, self.on_error, self.on_close)

        self.basic_messages = BasicMessages(
            self.websocket, self.data, self.logger)
        self.info_messages = InfoMessages(
            self.websocket, self.data, self.logger)
        self.request_messages = RequestMessages(
            self.websocket, self.data, self.logger)

        self.message_callback_list = []
        self.connection = True

        # Execute run in a thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

        self.websocket.run_forever()

    def run(self):
        """
        Keep websocket open
        """
        self.logger("run", "Running")
        while self.connection:
            self.websocket.run_forever()
        self.thread.join()

    def close(self):
        """
        Close websocket connection
        """
        self.logger("close", "Closing")
        self.websocket.close()

    def on_error(self, websocket_input: websocket, error: str):
        """
        This function is called when websocket has an error
        """
        self.logger("on_error", f"Error: {error}")
        self.websocket.close()

    def on_close(self, websocket_input: websocket, close_status_code, close_msg: str):
        """
        This function is called when websocket is closed
        """
        self.logger("on_close", "Connection closed")
        self.logger("on_close", f"Status: {close_status_code}")
        self.logger("on_close", f"Close message: {close_msg}")
        self.connection = False

    def on_open(self, websocket_input: websocket):
        """
        This function is called when websocket is open
        """
        self.connection = True
        self.logger("on_open", "Connected")
        self.basic_messages.handshake(self.data.rol)

    def on_message(self, websocket_input: websocket, message: dict):
        """
        This function is called when websocket receives a message and manage it
        """
        self.logger("on_message", f"Message recived: {message}")
        msg = json.loads(message)['message']

        # Communication with server
        if msg['type'] == 'basic' and msg['status'] == 'response':
            self.basic_messages.message_proccess(msg)
        else:
            self.logger("on_message", f"Unknown message: {msg}")
            self.on_message_callback(msg)

    def on_message_callback(self, msg: dict):
        """ Call each callback function in list when a message is received """
        for callback in self.message_callback_list:
            if callback['header'] == msg['header'] and callback['type'] == msg['type']:
                callback['function'](msg, callback['args'])

    def add_msg_callback(self, msg_type: str, msg_header: str, callback: object, *args: list):
        """ Add a function to callback list to messages with a specific header and type.
        This function is called when a message with the header and type is received, and
        receive the message and the args as parameters."""
        self.message_callback_list.append({
            'type': msg_type,
            'header': msg_header,
            'function': callback,
            'args': args
        })
