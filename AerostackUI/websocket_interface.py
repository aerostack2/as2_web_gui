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

""" Websocket client to communicate with server """

import threading
import json
import websocket
from .lib.basic_messages import BasicMessages
from .lib.info_messages import InfoMessages
from .lib.request_messages import RequestMessages
from .lib.websocket_client import WebSocketClient
from .lib.websocket_data import WebSocketClientData
from .aerostack_ui_logger import AerostackUILogger


class WebSocketClientInterface:
    """
    Manager websocket client
    """

    def __init__(self, host, logger: AerostackUILogger):
        self.host = host

        self.data = WebSocketClientData()

        self.logger = logger
        self.websocket_client = WebSocketClient(
            self.host, self.logger, self.on_open,
            self.on_message, self.on_error, self.on_close)

        self.basic_messages = BasicMessages(
            self.websocket_client, self.data, self.logger)
        self.info_messages = InfoMessages(
            self.websocket_client, self.data, self.logger)
        self.request_messages = RequestMessages(
            self.websocket_client, self.data, self.logger)

        self.message_callback_list = []
        self.connection = True

        self.callbacks_threads = []

        # Execute run in a thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        """
        Keep websocket open
        """
        self.logger.info("WebSocketClientInterface", "run", "Running")
        while self.connection:
            self.websocket_client.websocket.run_forever()
        for callback_thread in self.callbacks_threads:
            callback_thread.join()
        self.thread.join()

    def close(self):
        """
        Close websocket connection
        """
        self.logger.info("WebSocketClientInterface", "close", "Closing")
        self.websocket_client.websocket.close()

    def on_error(self, websocket_input: websocket, error):
        """
        This function is called when websocket has an error
        """
        self.logger.error("WebSocketClientInterface", "on_error", f"Error: {error}")
        self.websocket_client.websocket.close()

    def on_close(self, websocket_input: websocket, close_status_code, close_msg: str):
        """
        This function is called when websocket is closed
        """
        self.logger.info("WebSocketClientInterface", "on_close", "Connection closed")
        self.logger.info("WebSocketClientInterface", "on_close", f"Status: {close_status_code}")
        self.logger.info("WebSocketClientInterface", "on_close", f"Close message: {close_msg}")
        self.connection = False

    def on_open(self, websocket_input: websocket):
        """
        This function is called when websocket is open
        """
        self.connection = True
        self.logger.info("WebSocketClientInterface", "on_open", "Connected")
        self.basic_messages.handshake(self.data.rol)

    def on_message(self, websocket_input: websocket, message: dict):
        """
        This function is called when websocket receives a message and manage it
        """
        self.logger.debug("WebSocketClientInterface", "on_message", f"Message recived: {message}")
        print(f"Message recived: {message}")
        msg = json.loads(message)['message']

        # Communication with server
        if msg['type'] == 'basic' and msg['status'] == 'response':
            self.basic_messages.message_proccess(msg)
        else:
            self.logger.debug("WebSocketClientInterface","on_message", f"Unknown message: {msg}")
            self.on_message_callback(msg)

    def on_message_callback(self, msg: dict):
        """ Call each callback function in list when a message is received """
        for callback in self.message_callback_list:
            if callback['header'] == msg['header'] and callback['type'] == msg['type']:
                # Create a thread for each callback
                thread = threading.Thread(target=callback['function'], args=(msg, callback['args']))
                thread.start()
                self.callbacks_threads.append(thread)

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
