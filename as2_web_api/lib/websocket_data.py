from dataclasses import dataclass, field

@dataclass
class WebSocketClientData:
    """ Dataclass for websocket data"""
    rol: str
    client_id: int
    mission_id: int
    mission_id_list: list = field(default_factory=list)
    uav_id_list: list = field(default_factory=list)
    
    def __init__(self):
        pass