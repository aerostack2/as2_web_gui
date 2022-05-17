/**
 * WebSocket class that establish the connection and manage incoming messages.
 */
class WebSocketManager {
    /**
     * Create a new instance of the class WebSocketManager.
     * @param {string} host - Host of the server.
     */
    constructor(host) {

        /**
         * Web Socket object.
         * @type {WebSocket}
         * @access private
         */
        this._webSocket = new WebSocket(host);

        // Conect web socket callbacks to class functions
        this._webSocket.onopen = this._onOpen.bind(this);
        this._webSocket.onclose = this._onClose;
        this._webSocket.onerror = this._onError;
        this._webSocket.onmessage = this._onMessage.bind(this);

        /**
         * Id of the webpage client connected to the server.
         * @type {string}
         * @access private
         */
        this._id = null;

        /**
         * List of the callbacks to handle the incoming messages.
         * @type {array}
         * @access private
         */
        this._callbacksList = [];

        // Add basic callbacks to the list
        this.addCallback('basic', 'handshake', this._onHandshake.bind(this));
        this.addCallback('basic', 'getId', this._onGetId.bind(this));
        this.addCallback('basic', 'ping', this._onPing.bind(this));
        this.addCallback('basic', 'getClientsList', this._onGetClientsList.bind(this));
    }

    // #region Public methods

    /**
     * Add a callback to the incoming messages.
     * @param {string} type - The type of the message.
     * @param {string} header - The header of the message.
     * @param {function} callback - The callback function.
     * @param  {...any} args - The arguments of the callback function.
     * @return {void}
     * @access public
     */
    addCallback(type, header, callback, ...args) {
        this._callbacksList.push({
            'type': type,
            'header': header,
            'callback': callback,
            'args': args
        });
    }

    // #region Basic messages

    /**
     * Send a basic message to the Web Socket server.
     * @param {string} header - The header of the message.
     * @param {dict} payload - The payload of the message.
     * @param {string} [to=0] - Optional. The id of the client to send the message to. Default: 0 (the server id).
     * @return {void}
     * @access public
     */
    sendBasic(header, payload = {}, to = 0) {
        this._send({
            'type': 'basic',
            'header': header,
            'status': 'request',
            'payload': payload
        }, to);
    }

    /**
     * Send a handshake message to the Web Socket server.
     * @return {void}
     * @access public
     */
    sendBasicHandshake() {
        this.sendBasic('handshake', { 'rol': 'webpage' })
    }

    /**
     * Send a request of id message to the Web Socket server.
     * @return {void}
     * @access public
     */
    sendBasicGetId() {
        this.sendBasic('getId');
    }

    /**
     * Send a ping message to the Web Socket server.
     * @return {void}
     * @access public
     */
    sendBasicPing() {
        this.sendBasic('ping');
    }

    /**
     * Send a request of clients list message to the Web Socket server.
     * @return {void}
     * @access public
     */
    sendBasicGetClientList() {
        this.sendRequest('getClientsList');
    }

    // #endregion

    // #region Request messages

    /**
     * Send a request message to the Web Socket server.
     * @param {string} header - The header of the message.
     * @param {dict} payload - The payload of the message.
     * @param {string} [to=0] - Optional. The id of the client to send the message to. Default: 0 (the server id).
     * @return {void}
     * @access public
     */
    sendRequest(header, payload = {}, to = 'broadcast') {
        this._send({
            'type': 'request',
            'header': header,
            'status': 'request',
            'payload': payload
        }, to);
    }

    /**
     * Send a request of UAV list to the Web Socket server.
     * @return {void}
     * @access public
     */
    sendRequestGetUavList() {
        this.sendRequest('getUavList');
    }

    /**
     * Send a request of Mission list to the Web Socket server.
     * @return {void}
     * @access public
     */
    sendRequestGetMissionList() {
        this.sendRequest('getMissionList');
    }

    /**
     * Send a request of Confirm Mission to the Web Socket server.
     * @param {string} missionId - Id of the mission to confirm.
     * @param {array} uavList - List of the UAVs assigned to the mission.
     * @param {array} layers - List of the layers assigned to the mission.
     * @return {void}
     * @access public
     */
    sendRequestMissionConfirm(missionId, uavList, layers) {
        this.sendRequest(
            'missionConfirm',
            {
                'status': 'request',
                'id': missionId,
                'uavList': uavList,
                'layers': layers
            }
        );
    }

    /**
     * Send a request of Start Mission to the Web Socket server.
     * @param {string} missionId - Id of the mission to start.
     * @return {void}
     * @access public
     */
    sendStartMission(missionId) {
        this.sendRequest(
            'missionStart',
            {
                'id': missionId,
            },
            'manager'
        );
    }

    /**
     * Send a request of Stop Mission to the Web Socket server.
     * @param {string} missionId - Id of the mission to start.
     * @return {void}
     * @access public
     */
    sendStopMission(missionId) {
        this.sendRequest(
            'missionStop',
            {
                'id': missionId,
            },
            'manager'
        );
    }

    /**
     * Send a request of End Mission to the Web Socket server.
     * @param {string} missionId - Id of the mission to start.
     * @return {void}
     * @access public
     */
    sendEndMission(missionId) {
        this.sendRequest(
            'missionEnd',
            {
                'id': missionId,
            },
            'manager'
        );
    }

    // #endregion

    // #region Info messages

    /**
     * Send a info message to the Web Socket server.
     * @param {string} header - The header of the message.
     * @param {dict} payload - The payload of the message.
     * @param {string} [to='broadcast'] - Optional. The id of the client to send the message to. Default: 'broadcast'.
     * @return {void}
     * @access public
     */
    sendInfo(header, payload = {}, to = 'broadcast') {
        this._send({
            'type': 'info',
            'header': header,
            'payload': payload
        }, to);
    }

    /**
     * Send a info message with a mission confirmed.
     * @param {dict} missionData - Mission information.
     * @return {void}
     * @access public
     */
    sendConfirmedMission(missionData) {
        this.sendInfo(
            'missionInfo',
            missionData,
            'broadcast'
        );

        let msg = {
            'message': {
                'type': 'info',
                'header': 'missionInfo',
                'payload': missionData
            }
        }

        this._onMessage({ 'data': JSON.stringify(msg) });
    }

    /**
     * Send info message with a new UAV added.
     * @param {dict} uavData - UAV information.
     * @return {void}
     * @access public
     */
    sendUavInfo(uavData) {
        this.sendInfo(
            'uavInfo',
            uavData,
            'broadcast'
        );

        let msg = {
            'message': {
                'type': 'info',
                'header': 'uavInfo',
                'payload': uavData
            }
        }

        this._onMessage({ 'data': JSON.stringify(msg) });
    }

    // #endregion

    // #endregion

    // #region Private methods

    /**
     * Send a message to the WebSocket server.
     * @param {dict} msg - JSON-decoded message to be sent.
     * @param {int} [to=0] - Optional. The id of the client to send the message to. Default: 0 (the server id).
     * @return {void}
     * @access private
     */
    _send(msg, to = 0) {
        // If the client is logged in, add its id to the message and the destination
        if (this._id != null) {
            msg['from'] = this._id;
            msg['to'] = to;
        }
        this._webSocket.send(JSON.stringify({ 'message': msg }));

        //console.log("Message sent");
        //console.log(msg);
    }

    // #region WebSocket callbacks

    /**
     * This function is called when the WebSocket connection is closed.
     * @param {WebSocket} webSocket - The WebSocket object (Renference: https://websockets.readthedocs.io/).
     * @param {string} error - The error message.
     * @return {void}
     * @access private
     */
    _onError(webSocket, error) {
        console.log("Error");
        console.log(error);
    }

    /**
     * This function is called when the WebSocket connection is closed.
     * @param {WebSocket} webSocket - The WebSocket object (Renference: https://websockets.readthedocs.io/).
     * @param {string} close_status_code - The close status code.
     * @param {string} close_msg - The close message.
     * @return {void}
     * @access private
     */
    _onClose(webSocket, close_status_code, close_msg) {
        console.log("Close");
        console.log(`Status code: ${close_status_code}`);
        console.log(`Message: ${close_msg}`);
    }

    /**
     * This function is called when the WebSocket connection is opened.
     * @param {WebSocket} webSocket - The WebSocket object (Renference: https://websockets.readthedocs.io/).
     * @return {void}
     * @access private
     */
    _onOpen(webSocket) {
        this.sendBasicHandshake();
    }

    /**
     * This function is called when a message is received from the WebSocket and call the callbacks associated to the message.
     * @param {WebSocket} webSocket - The WebSocket object (Renference: https://websockets.readthedocs.io/).
     * @return {void}
     * @access private
     */
    _onMessage(webSocket) {
        // Get json message
        let msg = JSON.parse(webSocket.data).message;

        // console.log("Message received");
        // console.log(msg);

        for (let i = 0; i < this._callbacksList.length; i++) {
            if (this._callbacksList[i].type == msg.type && this._callbacksList[i].header == msg.header) {
                this._callbacksList[i].callback(msg.payload, this._callbacksList[i].args);
            }
        }
    }

    // #endregion

    // #region Basic messages callbacks

    /**
     * Callback for the handshake message.
     * @param {dict} payload - The payload of the message.
     * @param {array} args - The own arguments of the callback.
     * @return {void}
     * @access private
     */
    _onHandshake(payload, args) {
        if (payload['response'] == 'success') {
            this._id = payload['id'];
            console.log("Handshake with id=" + this._id);
        } else {
            throw new Error("Handshake failed");
        }
    }

    /**
     * Callback for the get id message.
     * @param {dict} payload - The payload of the message.
     * @param {array} args - The own arguments of the callback.
     * @return {void}
     * @access private
     */
    _onGetId(payload, args) {
        console.log(`Get id:`);
        this._id = payload['id'];
        console.log(payload);
    }

    /**
     * Callback for the ping message.
     * @param {dict} payload - The payload of the message.
     * @param {array} args - The own arguments of the callback.
     * @return {void}
     * @access private
     */
    _onPing(payload, args) {
        console.log(`Get:`);
        console.log(payload);
    }

    /**
     * Callback for the get client list message.
     * @param {dict} payload - The payload of the message.
     * @param {array} args - The own arguments of the callback.
     * @return {void}
     * @access private
     */
    _onGetClientsList(payload, args) {
        console.log(`Get client list:`);
        console.log(payload);
    }

    // #endregion

    // #endregion
}