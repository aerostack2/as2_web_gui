""" Module to manage websocket client logs """

class WebSocketClientLogger:
    """ Class to manage websocket client logs """

    def __init__(self, verbose: bool = True):
        self.enable = verbose

    def __call__(self, function: str, message: str) -> None:
        """ Log a message """
        if self.enable:
            print(f"WebSocketClient - {function} - {message}")