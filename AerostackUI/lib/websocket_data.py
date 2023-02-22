from dataclasses import dataclass, field

@dataclass
class WebSocketClientData:
    """ Dataclass for websocket data"""
    rol = "manager"
    client_id: int = 0
    mission_id: int = 0
    mission_id_list: list = field(default_factory=list)
    uav_id_list: list = field(default_factory=list)
