/*!
 * @license BSD-3-Clause
 * 
 * Copyright (c) 2024 Universidad Politécnica de Madrid
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the [Nombre del proyecto] nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Rafael Pérez Seguí
 */

/**
 * UAV and Mission Manager prototype, that manage income information from server and call desired callbacks.
 */
class ManagerPrototype extends SmartListCallbacks {

    /**
     * Create a new instance of the class ManagerPrototype.
     * @param {string} infoAdd - Name of the header of the info message that will be received from server when a parameter is add/modified.
     * @param {string} infoSet - Name of the header of the info message that will be received from server when the information is set/reset.
     * @param {string} infoGet - Name of the header of the request message that will be received from server when the information is requested.
     **/
    constructor(colors, infoAdd, infoSet, infoGet) {

        super();

        /**
         * List of colors for each id of the information. Pair [background, border].
         * @type {array}
         * @access private
         */
        this.colors = colors;

        // Callbacks for income information from server
        M.WS.addCallback('info', infoAdd, this._onInfo.bind(this), 'infoAdd');
        M.WS.addCallback('info', infoSet, this._onInfo.bind(this), 'infoSet');
        M.WS.addCallback('request', infoGet, this._onInfoGet.bind(this));
    }

    // #region Public methods

    /**
     * Return a list of two colors for each id of the information.
     * @param {any} id - Id of the information.
     * @returns {list} - List of two colors, one for the background and one for the border.
     * @access public
     */
    getColors(id) {
        return this.colors[super.getList().indexOf(id) % this.colors.length];
    }

    // #endregion

    // #region Private methods

    /**
     * Callback to info message with header infoAdd, that add/updates the information, or
     * with header infoSet, that set the information, and calls the callbacks.
     * @param {dict} payload - Payload of the info message
     * @return {void} 
     * @access private
     */
    _onInfo(payload, type) {

        if (!super.getList().includes(payload['id'])) {
            super.addObject(payload['id'], payload);
        } else {
            if (type[0] == 'infoAdd') {
                super.updateObject(payload['id'], payload, false);
            } else if (type[0] == 'infoSet') {
                super.updateObject(payload['id'], payload, true);
            }
        }
    }

    /**
     * Callback to request message with header infoGet, that get the list of information and calls the callbacks.
     * @param {dict} payload - Payload of the request message
     * @return {void} 
     * @access private
     */
    _onInfoGet(payload) {
        for (let key in payload['list']) {
            this._onInfo(payload['list'][key], 'infoSet');
        }
    }
    // #endregion
}


/**
 * Class that manage draw mission layers, adding callbacks when the mission is added/removed/edited.
 */
class DrawLayers extends SmartListCallbacks {

    /**
     * Create a new instance of the class DrawLayers.
     * @param {string} status - Status of the draw layer to be stored. 
     */
    constructor(status) {
        super();

        // Callbacks for Map information
        M.addMapCallback('layeradd', this._onLayerAdd.bind(this));
        M.addMapCallback('layerremove', this._onLayerRemove.bind(this));

        /**
         * Status of the layer. Use to identify the type of the layer, for example, draw, confirmed, etc.
         * @type {string}
         * @access private
         */
        this._status = status;

        /**
         * Id for each layer created.
         * @type {string} - Id of the layer.
         * @private
         */
        this._id = 0;
    }

    // #region Public methods

    /**
     * Remove the layer from the list of layers by its id.
     * @param {string} id - Id of the layer.
     * @return {void}
     * @access public
     */
    removeLayerById(id) {
        let layer = super.getDictById(id).layer;
        if (layer != null) {
            layer.remove();
        }
    }

    /**
     * Change the item index in the list and in the dictionary by the desired index.
     * @param {string} itemId - Id of the layer.
     * @param {string} newIndex - Desired index in the list.
     * @return {void}
     * @access public
     */
    changeLayersOrder(itemId, newIndex) {

        if (newIndex < 0) {
            newIndex = 0;
        } else if (newIndex > this.getList().length) {
            newIndex = this.getList().length;
        }

        let oldIndex = super.getList().indexOf(itemId);
        let newLayerList = Utils.deepCopy(super.getList());

        newLayerList[newIndex] = itemId;
        newLayerList[oldIndex] = super.getList()[newIndex];

        let newDict = Utils.deepCopy(super.getDict());

        for (let i = super.getList().length; i >= 0; i--) {
            let id = super.getList()[i];
            super.removeById(id);
        }

        for (let i = 0; i < newLayerList.length; i++) {
            let id = newLayerList[i];
            super.addObject(id, newDict[id]);
        }

        this._objectList = newLayerList;
    }

    // #endregion

    // #region Private methods

    /**
     * Process the layer, finding the drawManager attribute and the value of the information.
     * @param {L.Layer} layer - Layer to be process.
     * @return {list} - List with two elements: flag, value. Flag is true if the layer has a drawManager, false otherwise.  Value is a dict with the layer and the drawManager attribute.
     * @access private
     */
    _getDrawManager(layer) {

        let pmLayer = null;


        if (Utils.hasMember(layer, ['pm', '_layer'])) {
            pmLayer = layer.pm._layer;

            let drawManager = {};

            // Extract the drawManager attribute
            // If layer has been created by code
            if (Utils.hasMember(pmLayer, ['options', 'drawManager'])) {
                pmLayer.options.drawManager = Object.assign({}, pmLayer.options.drawManager);
                drawManager = pmLayer.options.drawManager;
                // If layer has been created by user
            } else if (Utils.hasMember(pmLayer, ['pm', 'options', 'drawManager'])) {
                pmLayer.pm.options.drawManager = Object.assign({}, pmLayer.pm.options.drawManager);
                drawManager = pmLayer.pm.options.drawManager;
            }
            // If the layer is a Draw layer, return its value
            if (Utils.hasMember(drawManager, ['options', 'status'])) {
                if (drawManager.options.status == this._status) {
                    let value = {
                        'layer': layer,
                        'drawManager': drawManager,
                    };
                    return [true, value];
                }
            }
        }
        return [false, null];
    }

    /**
     * When a layer is added to the map, it is checked if it has a drawManager attribute and assign a id to it.
     * @param {array} args - Empty array.
     * @param {event} e - Layer created event.
     * @return {void}
     * @access private
     */
    _onLayerAdd(args, e) {
        let info = this._getDrawManager(e.layer);
        let flag = info[0];
        let value = info[1];

        if (flag) {
            value.drawManager.id = this._id;
            value.id = this._id;
            super.addObject(value.drawManager.id, value);
            this._id++;

            // Add callback to the layer change
            let callback = this._onUserLayerChange.bind(this);
            e.layer.on('pm:edit', (e2) => {
                callback(e2);
            });

            // Add callback to the layer remove
            let callback2 = this._onCodeLayerChange.bind(this);
            e.layer.on('move', (e2) => {
                callback2(e2);
            });

            // TODO: Fix this, when the layer is created with option continueDraw = true, 
            // after user draw it, it should enable the layer draw again. Now it is not working.
            // if (Utils.hasMember(value, ['drawManager', 'instance', 'options', 'continueDraw'])) {
            //     console.log("Continue draw")
            //     value.drawManager.instance.userDraw(value.drawManager.options);
            // }
        }
    }

    /**
     * Layer remove callback.
     * @param {array} args - Empty array. 
     * @param {event} e - Layer remove event.
     * @return {void}
     * @access private
     */
    _onLayerRemove(args, e) {
        let event = Object.assign({}, e);
        let info = this._getDrawManager(event.layer);
        let flag = info[0];
        let value = info[1];

        if (flag) {
            super.removeById(value.drawManager.id);
        }
    }

    /**
     * Layer change by user callback.
     * @param {event} e - Layer change event.
     * @return {void}
     * @access private
     */
    _onUserLayerChange(e) {
        let info = this._getDrawManager(e.target);
        let flag = info[0];
        let value = info[1];
        if (flag) {
            super.updateObject(value.drawManager.id, value);
        }
    }

    /**
     * Layer change by code callback.
     * @param {event} e - Layer change event.
     * @return {void}
     * @access private
     */
    _onCodeLayerChange(e) {
        let info = this._getDrawManager(e.target);
        let flag = info[0];
        let value = info[1];
        if (flag) {
            super.updateObject(value.drawManager.id, value);
        }
    }

    // #endregion
}

/**
 * Class that manage UAV draw layers, adding callbacks when the UAV is added/removed/edited.
 */
class UavManager extends ManagerPrototype {

    /**
     * Create a new instance of the class ManagerPrototype.
     * @param {array} colors - List of colors for each id of the information. Pair [background, border].
     * @param {string} infoAdd - Name of the header of the info message that will be received from server when a parameter is add/modified.
     * @param {string} infoSet - Name of the header of the info message that will be received from server when the information is set/reset.
     * @param {string} infoGet - Name of the header of the request message that will be received from server when the information is requested.
     **/
    constructor(colors, infoAdd, infoSet, infoGet) {
        super(colors, infoAdd, infoSet, infoGet);
    }

    // #region Public methods

    /**
     * Add a UAV to the map, to use the system without real UAV connected.
     * @param {string} id - Id of the UAV.
     * @param {string} state - State of the UAV.
     * @param {dict} pose - Pose of the UAV. Dictionary with the following keys: lat, lng, alt, yaw.
     * @return {void}
     * @access public
     */
    addVirtualUav(id, state, pose) {

        let uavInfo = {
            'id': id,
            'state': state,
            'pose': pose
        }

        if (!super.getList().includes(uavInfo.id)) {
            super.addObject(uavInfo.id, uavInfo);
        } else {
            super.updateObject(uavInfo.id, uavInfo, false);
        }
    }

    // #endregion
}

/**
 * Mission Manager that extends the ManagerPrototype to manage confirm and reject mission messages.
 */
class MissionManager extends ManagerPrototype {

    /**
     * Create a new instance of the class MissionManager.
     * @param {array} colors - List of colors for each id of the information. Pair [background, border].
     * @param {string} missionConfirm - Name of the header of the info message that will be received from server when a mission is confirmed/rejected.
     * @param {string} missionAdd - Name of the header of the info message that will be received from server when a parameter of the mission is add/modified.
     * @param {string} missionSet - Name of the header of the info message that will be received from server when a mission is set/reset.
     * @param {string} missionGet - Name of the header of the request message that will be received from server when the mission list is requested.
     */
    constructor(colors, missionAdd, missionSet, missionGet) {
        super(colors, missionAdd, missionSet, missionGet);

        let mission_request_list = ['missionConfirm', 'missionPause',
            'missionResume', 'missionStop', 'missionRemove'];

        /**
         * List of mission requests list of callbacks.
         * @type {dict}
         * @private
         */
        this._missionRequestListCallbacks = {};

        // Create WS callbacks for each mission request type
        for (let i = 0; i < mission_request_list.length; i++) {
            let header = mission_request_list[i];
            M.WS.addCallback('request', header, this._onMissionRequestList.bind(this), header);
        }
    }

    // #region Public methods

    /**
     * Add a function callback when a mission is confirmed by the server.
     * @param {function} callback - Function to be called when the mission is confirmed/rejected.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addMissionConfirmCallback(callback, ...args) {
        this._addMissionCallback('missionConfirm', callback, args);
    }

    /**
     * Add a function callback when a mission is paused by the server.
     * @param {function} callback - Function to be called when the mission is paused.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void}
     * @access public
     */
    addMissionPauseCallback(callback, ...args) {
        this._addMissionCallback('missionPause', callback, args);
    }

    /**
     * Add a function callback when a mission is resumed by the server.
     * @param {function} callback - Function to be called when the mission is resumed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void}
     * @access public
     */
    addMissionResumeCallback(callback, ...args) {
        this._addMissionCallback('missionResume', callback, args);
    }

    /**
     * Add a function callback when a mission is stopped by the server.
     * @param {function} callback - Function to be called when the mission is stopped.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void}
     * @access public
     */
    addMissionStopCallback(callback, ...args) {
        this._addMissionCallback('missionStop', callback, args);
    }

    /**
     * Add a function callback when a mission is removed by the server.
     * @param {function} callback - Function to be called when the mission is removed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void}
     * @access public
     */
    addMissionRemoveCallback(callback, ...args) {
        this._addMissionCallback('missionRemove', callback, args);
    }

    // #endregion

    // #region Private methods

    /**
     * Callback to request message with header missionRequestList, that get the response of the mission request list.
     * @param {string} header - Header of the request message.
     * @param {function} callback - Function to be called when the mission is resumed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void}
     * @access private
     */
    _addMissionCallback(header, callback, ...args) {
        if (header in this._missionRequestListCallbacks) {
            this._missionRequestListCallbacks[header].push([callback, args]);
        } else {
            this._missionRequestListCallbacks[header] = [[callback, args]];
        }
    }


    /**
     * Callback to request message with header in missionRequestList, that get the response of the mission request.
     * @param {dict} payload - payload of the request message
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access private
     */
    _onMissionRequestList(payload, ...args) {
        let header = args[0][0];
        let callCallbacksFlag = false;
        switch (header) {
            case 'missionConfirm':
                if (payload.status == 'confirmed') {
                    callCallbacksFlag = true;
                } else if (payload.status == 'rejected') {
                    console.log('Mission confirm rejected');
                    ConsoleSideBar.addWarning('Mission confirm rejected: ' + payload['extra']);
                }
                break;
            case 'missionPause':
                if (payload.status == 'paused') {
                    callCallbacksFlag = true;
                } else if (payload.status == 'rejected') {
                    ConsoleSideBar.addWarning('Mission pause rejected: ' + payload['extra']);
                }
                break;
            case 'missionResume':
                if (payload.status == 'resumed') {
                    callCallbacksFlag = true;
                } else if (payload.status == 'rejected') {
                    ConsoleSideBar.addWarning('Mission resume rejected: ' + payload['extra']);
                }
                break;
            case 'missionStop':
                if (payload.status == 'stopped') {
                    callCallbacksFlag = true;
                } else if (payload.status == 'rejected') {
                    ConsoleSideBar.addWarning('Mission stop rejected: ' + payload['extra']);
                }
                break;
            case 'missionRemove':
                if (payload.status == 'removed') {
                    callCallbacksFlag = true;
                } else if (payload.status == 'rejected') {
                    ConsoleSideBar.addWarning('Mission remove rejected: ' + payload['extra']);
                }
                break;
            default:
                console.log('Mission request header: ' + header + ' not found');
                break;
        }
        if (callCallbacksFlag) {
            Utils.callCallbacks(this._missionRequestListCallbacks[header], payload);
        }
    }

    // #endregion
}


/**
 * Map manager that create the map, start the communication with the server and manage sidebars.
 */
class MapManager {

    /**
     * Create the instance of the MapManager.
     * @param {array} mapCenter - List of the coordinates [latitude, longitude] of the center of the map.
     * @param {number} zoom - Initial zoom of the map.
     * @param {string} host - Host of the server.
     */
    constructor(
        mapCenter = config.Global.mapCenter,
        zoom = config.Global.zoom,
        host = config.WebSocket.host) {

        /**
         * List of the coordinates [latitude, longitude] of the center of the map.
         * @type {array}
         * @access public
         */
        this.mapCenter = mapCenter;

        /**
         * Smart list with the information recived from server.
         * @type {L.Map} - Map of the library Leaflet (Reference: https://leafletjs.com/).
         * @access public
         */
        this.MAP = new L.Map(
            'mapid',
            {
                center: mapCenter,
                zoom: zoom,
                doubleClickZoom: false
            }
        );

        // Add tile layers to the map by the config file
        let mapLayers = {};
        let mapUrl = config.Global.mapUrl;
        for (let i = 0; i < config.Global.subdomains.length; i++) {
            let subdomainsLabel = config.Global.subdomainsLabels[i];
            let subdomainMaxZoom = config.Global.subdomainsMaxZoom[i];

            mapLayers[subdomainsLabel] = new L.TileLayer(mapUrl, {
                maxZoom: subdomainMaxZoom,
                subdomains: config.Global.subdomains,
                useCache: config.Global.useCache,
                crossOrigin: config.Global.useCache,
                saveToCache: config.Global.saveToCache,
                useOnlyCache: config.Global.useOnlyCache,
                cacheMaxAge: config.Global.cacheMaxAgeInMs,
            });

            if (i == 0) {
                mapLayers[subdomainsLabel].addTo(this.MAP);
            }
        }

        /**
         * Layer control of the map.
         * @type {L.control} - Control of the library Leaflet (Reference: https://leafletjs.com/).
         * @access public
         */
        this.layerControl = L.control.layers(mapLayers, {}, { position: 'topleft', collapsed: false }).addTo(this.MAP);

        // Create sidebars HTML elements
        this._initializeSideBars();

        // Initialize connection to server
        /**
         * Web Socket Manager instance to manage the connection with the server.
         * @type {WebSocketManager}
         * @access public
         */
        this.WS = new WebSocketManager(host);

        // Add callbacks to the WebSocketManager when a basic message of handshake is received
        this.WS.addCallback('basic', 'handshake', this._onHandshake.bind(this));

        // Layers created manager
        this.addMapCallback('pm:create', this._pmOnCreateCallback);

        /**
         * Uav picker callback list.
         * @type {array} - List of four elements: id, callback, othersElements, args. Id is the html element id of the uav picker. Callback is the function to be called when the uav picker is pick. OthersElements is a list of list [name, flag]: name is the name of the element to add to the list of picker elements, and the flag is false when the element must be remove when the uav list is not empty, true otherwise . Args is a list of arguments to be passed to the callback.
         * @private
         */
        this._uavPickerCallbackList = [];
    }

    // #region Public methods

    /**
     * Initialize UAV, mission and layers managers.
     * @return {void}
     * @access public
     */
    initialize() {
        // Manage GPS mode or Local Coordinates mode
        this.USE_LOCAL_COORDINATES = config.Global.useLocalCoordinates;
        this.X_NAME = 'Latitude';
        this.Y_NAME = 'Longitude';
        if (this.USE_LOCAL_COORDINATES) {
            this.UTM = new LocalCoordinates(this.mapCenter);
            this.X_NAME = 'X';
            this.Y_NAME = 'Y';
        } else {
            // Add a layer control to the map, with the latitude and longitude of the mouse
            L.control.coordinates({
                position: "bottomright",
                decimals: 5,
                decimalSeperator: ".",
                labelTemplateLat: "Lat: {y}",
                labelTemplateLng: "Lng: {x}",
                useLatLngOrder: true
            }).addTo(this.MAP);
        }

        this.UAV_MANAGER = new UavManager(config.UAV.colors, 'uavInfo', 'uavInfoSet', 'getUavList');
        this.MISSION_MANAGER = new MissionManager(config.Mission.colors, 'missionInfo', 'missionInfoSet', 'getMissionList');

        this.DRAW_LAYERS = new DrawLayers('draw');
        this.MISSION_LAYERS = new DrawLayers('confirmed');
        this.UAV_LAYERS = new DrawLayers('uav');

        this.UAV_MANAGER.addInfoAddCallback(this._updateUavPickerListCallback.bind(this));
    }

    /**
     * Get all layers of the map and return them in a list.
     * @returns {array} - List of layers.
     * @access public
     */
    getLayers() {
        return L.PM.Utils.findLayers(this.MAP);
    }

    /**
     * Add a function callback to map listener
     * @param {function} callback - Function to be called when the map event happend
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addMapCallback(type, callback, ...args) {
        this.MAP.on(type, function (e) {
            callback(args, e);
        });
    }

    /**
     * Get UAV picker Html.
     * @param {string} type - Picker type: checkbox or radio.
     * @param {string} id - Id of the Html element to add.
     * @param {array} othersElements - OthersElements is a list of list [name, flag]: name is the name of the element to add to the list of picker elements, and the flag is false when the element must be remove when the uav list is not empty, true otherwise.
     * @param {function} callback - Function to be called when the uav picker is pick.
     * @param {array} args - Arguments to be passed to the callback.
     * @return {void}
     * @access public
     */
    getUavPickerDict(type, id, othersElements = [], callback, ...args) {
        let uavList = M.UAV_MANAGER.getList();

        // Add others elements to the list
        if (uavList.length > 0) {
            for (let i = 0; i < othersElements.length; i++) {
                if (!othersElements[i][1]) {
                    othersElements.splice(i, 1);
                }
            }
        }

        this._uavPickerCallbackList.push([id, callback, othersElements, args]);

        // Create the picker with the uav list
        if (othersElements.length > 0) {
            let list = [];
            for (let i = 0; i < othersElements.length; i++) {
                list.push(othersElements[i][0]);
            }
            return HTMLUtils.addDict('checkBoxes', `${id}`, { 'class': 'UavPicker' }, type, list.concat(uavList));
        }
        return HTMLUtils.addDict('checkBoxes', `${id}`, { 'class': 'UavPicker' }, type, uavList);
    }

    /**
     * Add callback to each element of the uav picker.
     * @param {string} id - Html element id of the uav picker.
     * @return {void}
     * @access public
     */
    uavPickerInitiliazeCallback(id) {
        // Add callback to the checkbox
        for (let i = 0; i < this._uavPickerCallbackList.length; i++) {
            let divId = this._uavPickerCallbackList[i][0];
            let callback = this._uavPickerCallbackList[i][1];
            let othersElements = this._uavPickerCallbackList[i][2];
            let userargs = this._uavPickerCallbackList[i][3];

            if (divId == id) {
                // Add callback to the othersElements
                for (let j = 0; j < othersElements.length; j++) {
                    let name = othersElements[j][0];
                    this._uavPickerAddCallback(id, name, callback, userargs);
                }
                // Add callback to the uavList
                let uavList = M.UAV_MANAGER.getList();
                for (let j = 0; j < uavList.length; j++) {
                    let name = uavList[j];
                    this._uavPickerAddCallback(id, name, callback, userargs);
                }
            }
        }
    }

    // #endregion

    // #region Private methods

    /**
     * This callback is called when a layer is created and manage it options.
     * @param {array} args - Empty array. 
     * @param {event} e - Event of the layer creation.
     * @returns {void}
     * @access private
     */
    _pmOnCreateCallback(args, e) {
        // Change layer color if the option color is set
        if (Utils.hasMember(e.layer.pm.options, ['color'])) {
            e.layer.setStyle({ color: e.layer.pm.options.color });
            if (Utils.hasMember(e.layer.options, ['color'])) {
                e.layer.options.color = e.layer.options.color;
            }
        }
    }

    // #region Web Socket Callbacks

    /**
     * Callback to basic message with header handshake, and ask for the mission and UAVs state.
     * @param {dict} payload - payload of the handshake message. Must contain the key 'response' and must be 'success' to continue.
     * @return {void}
     * @access private
     */
    _onHandshake(payload) {
        if (payload.response == 'success') {
            this.WS.sendRequestGetUavList();
            this.WS.sendRequestGetMissionList();
        } else {
            throw new Error('Handshake failed');
        }
    }

    // #endregion

    // #region Side Bars

    /**
     * Initialize left and right side bars, adding them to the map.
     * @return {void}
     * @access private
     */
    _initializeSideBars() {
        this._initializeLefSideBar();
        this._initializeRightSideBar();
    }

    /**
     * Initialize left side bar, adding it to the map.
     * @return {void}
     * @access private
     */
    _initializeLefSideBar() {
        this.sidebar_left = L.control.sidebar({
            autopan: true,             // whether to maintain the centered map point when opening the sidebar
            closeButton: true,         // whether t add a close button to the panes
            container: 'sideBar-left', // the DOM container or #ID of a predefined sidebar container that should be used
            position: 'left',
        }).addTo(this.MAP);
    }

    /**
     * Initialize right side bar, adding it to the map.
     * @return {void}
     * @access private
     */
    _initializeRightSideBar() {
        this.sidebar_right = L.control.sidebar({
            autopan: false,
            closeButton: true,
            container: 'sideBar-right',
            position: 'right',
        }).addTo(this.MAP);
    }

    // #endregion

    // #region Uav Picker

    /**
     * Add callback to the checkbox element
     * @param {string} id - Id of the uav picker html element.
     * @param {string} name - Name of the picker element.
     * @param {function} callback - Function to be called when the uav picker is pick.
     * @param {array} userargs - Arguments to be passed to the callback.
     * @return {void}
     * @access private
    */
    _uavPickerAddCallback(id, name, callback, userargs) {
        if (M.UAV_MANAGER.getList().includes(name)) {
            let label = document.getElementById(id + '-' + name + '-Label');
            label.style.setProperty("background-color", `${M.UAV_MANAGER.getColors(name)[1]}`, "important");
        }

        let checkbox = document.getElementById(id + '-' + name + '-Input');
        checkbox.addEventListener('change', function () {
            let inputId = this.id.split('-');
            let uavName = inputId[inputId.length - 2];
            let value = this.checked;

            callback(uavName, value, userargs);
        });
    }

    /**
     * Update UAV picker when a new UAV is added or removed.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the id of the uav added or removed.
     * @param {HTMLElement} picker - HTML element of the picker.
     * @return {void}
     * @access private
     */
    _updateUavPickerList(myargs, args, picker) {

        // Update the checkbox HTML
        let childId = picker.children[0].id;
        let type = document.getElementById(childId + '-Input').type;

        HTMLUtils.addCheckBox(picker.id, picker.id, type, args[0]);

        // Add callback to the checkbox
        for (let i = 0; i < this._uavPickerCallbackList.length; i++) {
            let divId = this._uavPickerCallbackList[i][0];
            let callback = this._uavPickerCallbackList[i][1];
            let othersElements = this._uavPickerCallbackList[i][2];
            let userargs = this._uavPickerCallbackList[i][3];

            if (divId == picker.id) {

                this._uavPickerAddCallback(picker.id, args[0], callback, userargs);

                // Remove defaults checkboxes
                for (let i = 0; i < othersElements.length; i++) {
                    let name = othersElements[i][0];
                    let keep = othersElements[i][1];
                    if (!keep) {
                        let checkBoxId = picker.id + '-' + name;
                        document.getElementById(checkBoxId).remove();
                        this._uavPickerCallbackList[i][2].splice(i, 1);
                    }
                }
            }
        }
    }

    /**
     * Update UAV picker list callback when a new UAV is added or removed.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - UAV name added or removed.
     * @param {HTMLElement} picker - HTML element of the picker.
     * @return {void}
     * @access private
     */
    _updateUavPickerListCallback(myargs, args) {
        let pickers = document.getElementsByClassName('UavPicker');
        for (let i = 0; i < pickers.length; i++) {
            this._updateUavPickerList(myargs, args, pickers[i]);
        }
    }

    // #endregion

    // #endregion
}
