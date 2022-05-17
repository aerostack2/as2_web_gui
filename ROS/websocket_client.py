from site import ENABLE_USER_SITE
from django.db import connection
import websocket
import threading
import json
import time


def nullFunction(*args):
    """
    Default function for the websocket that does nothing
    """
    return


# Asynchronous client to websockets server
class WebSocketClient:
    """
    Manager websocket client
    """
    def __init__(self, host="ws://127.0.0.1:8000/ws/user/"):
        self.host = host
        websocket.enableTrace(False)

        self.ws = websocket.WebSocketApp(self.host,
                                         on_open=self.onOpen,
                                         on_message=self.onMessage,
                                         on_error=self.onError,
                                         on_close=self.onClose)
        self.connection = False

        self.rol = 'manager'
        self.id = None
        
        #self.on_message_function = None
        #self.new_mission_function = None
        
        self.uav_id_list = []
        self.mission_id_list = []
        
        self.msg_id = 0
        self.mission_id = 0 # Can not be 0
        
        self.message_callback_list = []

        # Execute run in a thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()


    def run(self):
        """
        Keep websocket open
        """
        print("Running")
        while self.connection == False: 
            self.ws.run_forever()

    def close(self):
        """
        Close websocket connection
        """
        print("Closing connection")
        self.ws.close()
        self.thread.join()

    def onError(self, ws, error):
        """
        This function is called when websocket has an error
        """
        print(f"Error: {error}")
        self.ws.close()

    def onClose(self, ws, close_status_code, close_msg):
        """
        This function is called when websocket is closed
        """
        # print("Connection closed")
        # print(f"Status: {close_status_code}")
        # print(f"Close message: {close_msg}")
        self.connection = False

    def onOpen(self, ws):
        """
        This function is called when websocket is open
        """
        self.connection = True
        print("Connected")
        self.handshake()
        
    def onMessage(self, ws, message):
        """
        This function is called when websocket receives a message and manage it
        """
        print(f"Message recived: {message}")
        msg= json.loads(message)['message']
        
        # Communication with server
        if msg['type'] == 'basic' and msg['status'] == 'response':
            
            if msg['header'] == 'handshake':
                if msg['payload']['response'] == 'success':
                    print("Logged in")
                    self.id = msg['payload']['id']
                else:
                    print(f"Login failed with error {msg['response']}")
                    self.ws.close()
            
            elif msg['header'] == 'getId':
                self.id = msg['payload']['id']
            
            elif msg['header'] == 'ping':
                print(msg['payload'])
                
            elif msg['header'] == 'getClientList':
                print(msg['payload'])
        
            """ 
            elif msg['type'] == 'request':
                
                if msg['header'] == 'missionConfirm' and msg['status'] == 'request':
                    print("WebSocketClient - Confirm mission")
                    self.new_mission_function(msg)
            """
        
        else:
            print("Unknown message")
            print(msg)
            self.callMsgCallback(msg)
        
    def send(self, msg, to=None):
        """
        Send a JSON-decoded message to server

        Args:
            msg (dict): JSON-decoded message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        #print("Message sent: " + message)
        msg['msg_id'] = self.msg_id
        self.msg_id+=1
            
        if self.id is not None:
            msg['from'] = self.id
            if to is None:
                msg['to'] = 0 # Server id
            else:
                msg['to'] = to
        
        message = json.dumps({'message': msg})
            
        self.ws.send('%s' % message)
        
    def callMsgCallback(self, msg):
        for callback in self.message_callback_list:
            if callback['header'] == msg['header'] and callback['type'] == msg['type']:
                callback['function'](msg, callback['args'])
        
    def addMsgCallback(self, type, header, function, *args):
        self.message_callback_list.append({
            'type': type,
            'header': header,
            'function': function,
            'args': args
        })
    
    #region Basic communication
    
    def sendBasic(self, header, payload):
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
        self.send(msg, 'server') # To server
    
    def handshake(self):
        """
        Send handshake message to server
        """
        payload = {'rol': self.rol}
        self.sendBasic('handshake', payload)
    
    def getId(self):
        """
        Send getId message to server
        """        
        payload = {'id': self.id}
        self.sendBasic('getId', payload)
    
    def ping(self):
        """
        Send ping message to server
        """        
        payload = {}
        self.sendBasic('ping', payload)
        
    def getClientList(self):
        """
        Send getClientList message to server
        """        
        payload = {}
        self.sendBasic('getClientsList', payload)
        
    #endregion
    
    #region Info communication
    
    def sendInfo(self, header, payload, to=None):
        """
        Send a info message to server

        Args:
            header (str): Header of the message
            payload (dict): Payload of the message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        msg = {
            'type': 'info',
            'header': header,
            'payload': payload
        }
        self.send(msg, 'webpage') # To all webpages
    
    def send_uav_info(self, uav_info, override=False):
        
        if not ('id' in uav_info):
            raise Exception("UAV info must contain the key 'id'")
        
        id = uav_info['id']
        
        if (not (id in self.uav_id_list)):
            self.uav_id_list.append(id)
            if (not ('id' in uav_info and 'state' in uav_info and 'pose' in uav_info)):
                raise Exception('Invalid UAV info. For the firt time an UAV is send, it must contain the keys "id", "state" and "pose"')
        
        payload = uav_info 
        
        if override:
            self.sendInfo('uavInfoSet', payload)
        else:
            self.sendInfo('uavInfo', payload)
    
    def send_mission_info(self, mission_info, override=False):
        
        if not ('id' in mission_info):
            raise Exception("Mission info must contain the key 'id'")
        
        id = mission_info['id']
        
        if (not (id in self.mission_id_list)):
            self.mission_id_list.append(id)
            if (not ('id' in mission_info and 'uavList' in mission_info and 'layers' in mission_info)):
                raise Exception('Invalid Mission info. For the firt time a Mission is send, it must contain the keys "id", "uavList" and "layers"')
        
        payload = mission_info 
        
        if override:
            self.sendInfo('missionInfoSet', payload)
        else:
            self.sendInfo('missionInfo', payload)

    #endregion
    
    #region Request communication
    def sendRequest(self, header, payload, to=None):
        """
        Send a request message to server

        Args:
            header (str): Header of the message
            payload (dict): Payload of the message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        msg = {
            'type': 'request',
            'header': header,
            'status': 'response',
            'payload': payload
        }
        self.send(msg, to)
    
    def missionConfirm(self, new_mission_id, status, oldId, author, extra=[]):
        confirm_payload = {
            'id': new_mission_id, 
            'status': status,
            'oldId': oldId,
            'author': author,
            'extra': extra,
        }
        self.sendRequest('missionConfirm', confirm_payload, to=author)    
    
    #endregion


def planner(client, uavId, layers):
    trayectory = []
    for layer in layers:
        print(layer)
        if layer['name'] == 'TakeOffPoint' or layer['name'] == 'LandPoint':
            trayectory.append([layer['values']['lat'], layer['values']['lng']])
            
        elif layer['name'] == 'Path':
            path = layer['values']
            for waypoint in path:
                trayectory.append([waypoint['lat'], waypoint['lng']])
    
    if len(trayectory) > 0:
        client.send_uav_info({'id': uavId, 'desiredPath': trayectory})

def newMissionCallback(client, msg):
    print(f"- Callback: Received mission:")
    print(msg)
    
    confirm = 'confirmed'
    extra = []
    
    if msg['payload']['status'] == 'request':
        if len(msg['payload']['uavList']) == 0:
            confirm = 'rejected'
            extra.append('No UAVs')
        
        if len(msg['payload']['layers']) == 0:
            confirm = 'rejected'
            extra.append('No layers')
            
        if msg['payload']['id'] != 'New Mission':
            confirm = 'rejected'
            extra.append('Invalid id, only "New Mission" is allowed for now :)')
    
    client.missionConfirm(
        client.mission_id,
        confirm,
        msg['payload']['id'],
        msg['from'],
        extra
    )
    
    client.mission_id += 1
    
    if confirm == 'confirmed':
        new_mission_info = msg['payload']
        new_mission_info['status'] = confirm
        new_mission_info['id'] = client.mission_id
        
        client.send_mission_info(new_mission_info)
        
        planner(client, msg['payload']['uavList'][0], msg['payload']['layers'])
        
        
    
import math
def getYaw(angle):
    return angle * math.pi / 180
                

def main():
    
    
    client = WebSocketClient("ws://127.0.0.1:8000/ws/user/")
    print(client.mission_id)
    client.new_mission_function = newMissionCallback
 
    time.sleep(1)

    client.send_uav_info({'id': 'UAV 0', 'state': 'landed', 'pose': {'lat': 28.144099, 'lng': -16.503337, 'height': 1, 'yaw': getYaw(0)}, 'sensors': {'battey': 80, 'temperature': 40}})
  
    time.sleep(1)
    
    lat = 28.144099
    lng = -16.503337
    height = 1
    
    incr = 0.00001
    odom = []
    yaw = 0
    
    for i in range(1000):
        time.sleep(1. / 5)
        lat += incr
        lng += incr
        yaw += incr * 1000000
        height += incr * 10000
        odom.append([lat, lng])
        client.send_uav_info({'id': 'UAV 0', 'pose': {'lat': lat, 'lng': lng, 'height': height, 'yaw': getYaw(yaw)}, 'odom': odom})


if __name__ == "__main__":
    main()
