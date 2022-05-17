/**
 * Class that manage UAV Drawer.
 */
class UavDrawer {
    /**
     * Creates a new UavDrawer instance.
     */
    constructor() {
        // Add UAV Callbacks
        M.UAV_MANAGER.addInfoParamCallback('pose', this._updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('odom', this._updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('desiredPath', this._updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('state', this._updateUavParam.bind(this));

        // Add Map callbacks to update UAV popups.
        M.addMapCallback('pm:drawstart', this._onDrawCallback.bind(this), false);
        M.addMapCallback('pm:drawend', this._onDrawCallback.bind(this), true);

        // Add draw types of mission layers
        this._addDrawTypes();
    }

    /**
     * Creates an instance of each type of UAV layers.
     * @returns {void}
     * @access private
     */
    _addDrawTypes() {
        /**
         * UAV Smart List that store old information about UAVs to know what parameter has been updated.
         * @type {SmartList}
         * @private
         */
        this.UAV_LIST = new SmartList();

        let status = 'uav';

        /**
         * UAV Marker manager.
         * @type {UAVMarker}
         * @access private
         */
        this._uavMarker = new UAVMarker(status);

        /**
         * UAV Odometry manager.
         * @type {Odom}
         * @access private
         */
        this._odom = new Odom(status);

        /**
         * UAV Desired Path manager.
         * @type {Odom}
         * @access private
         */
        this._desiredPath = new DesiredPath(status, undefined, {'opacity': 0.5});        
    }

    /**
     * Check if a layer already exists. If true, remove it to draw a new one updated.
     * @param {string} id - UAV id.
     * @param {string} name - Layer name. For example: 'pose', 'odom', 'desiredPath'.
     * @returns {void}
     * @access private
     */
    _checkLayer(id, name) {
        if (name in this.UAV_LIST.getDictById(id)) {
            this.UAV_LIST.getDictById(id)[name].codeLayerDrawn.remove();
        }
    }

    /**
     * Callback to a new UAV parameter add/remove or change. Manage it and update the UAV layers.
     * @param {string} param - Parameter name.
     * @param {array} value - List with the new value.
     * @param {array} args - List with the UAV id.
     * @returns {void}
     * @access private
     */
    _updateUavParam(param, value, args) {
        let uavId = args[0];

        if (this.UAV_LIST.getList().indexOf(uavId) === -1) {
            this.UAV_LIST.addObject(uavId, {'id': uavId});
        }

        if (param == 'odom' && value.length < 2 ||
            param == 'desiredPath' && value.length < 2) {
            return;
        }

        if (param in this.UAV_LIST.getDictById(uavId)) {
            switch (param) {
                case 'pose':
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeLayerDrawn.setLatLng([value['lat'], value['lng']]);
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeLayerDrawn.options.rotationAngle = this.Utils._angleENU2NEU(value['yaw']);
                    this._updatePopup(uavId);
                    break;
                case 'odom':
                    this.UAV_LIST.getDictById(uavId)['layerOdom'].codeLayerDrawn.setLatLngs(value);
                    break;
                case 'desiredPath':
                    this.UAV_LIST.getDictById(uavId)['layerDesiredPath'].codeLayerDrawn.setLatLngs(value);
                    break;
                case 'state':
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeLayerDrawn.options['state'] = value;
                    break;
                default:
                    break;
            }

        } else {
            let desiredColor = M.UAV_MANAGER.getColors(uavId);
            switch (param) {
                case 'pose':
                    this._checkLayer(uavId, 'layerPose');
                    let colors = M.UAV_MANAGER.getColors(uavId);
                    this.UAV_LIST.getDictById(uavId)['layerPose'] = this._uavMarker;
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeDraw([value['lat'], value['lng']], {'uavList': [uavId]}, {'rotationAngle': this.Utils._angleENU2NEU(value['yaw'])}, desiredColor);
                    this._updatePopup(uavId);
                    break;
                case 'odom':
                    this._checkLayer(uavId, 'layerOdom'); 
                    this.UAV_LIST.getDictById(uavId)['layerOdom'] = this._odom;
                    let odomValue = value;
                    if (value.length < 2) {
                        odomValue = [value, value];
                    }
                    this.UAV_LIST.getDictById(uavId)['layerOdom'].codeDraw(odomValue, {'uavList': [uavId]}, {'color': desiredColor[0]});
                    break;
                case 'desiredPath':
                    this._checkLayer(uavId, 'layerDesiredPath');
                    this.UAV_LIST.getDictById(uavId)['layerDesiredPath'] = this._desiredPath
                    this.UAV_LIST.getDictById(uavId)['layerDesiredPath'].codeDraw(value, {'uavList': [uavId]}, {'color': desiredColor[0]});
                    break;
                case 'state':
                    this._checkLayer(uavId, 'layerPose');
                    let pose = M.UAV_MANAGER.getDictById(uavId)['pose'];
                    this.UAV_LIST.getDictById(uavId)['pose'] = pose;

                    this.UAV_LIST.getDictById(uavId)['layerPose'] = this._uavMarker;
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeDraw([pose['lat'], pose['lng']], {'uavList': [uavId]}, undefined, desiredColor);
                    this.UAV_LIST.getDictById(uavId)['layerPose'].options.drawManager.options['state'] = value;
                default:
                    break;
            }
        }
        this.UAV_LIST.getDictById(uavId)[param] = value;
    }

    /**
     * Update popup content with the UAV information.
     * @param {string} id - UAV id.
     * @returns {void}
     * @access private 
     */
    _updatePopup(id) {
        var uavLayer = this.UAV_LIST.getDictById(id)['layerPose'];
        
        let height = M.UAV_MANAGER.getDictById(id)['pose']['height'];
        // console.log("Update popup: " + height);
        let popupContent = `<p>Height = ${Utils.round(height, 2)} m</p>`;

        if (uavLayer['popup'] == undefined) {
            uavLayer['marker'] = uavLayer.codeLayerDrawn;

            let popup = L.popup({
                closeOnClick: false,
                autoClose: false
            }).setContent(popupContent);

            uavLayer['marker'].bindPopup(popup).openPopup();
            uavLayer['popup'] = uavLayer['marker'].getPopup();
            uavLayer['popupState'] = true;

            var callback = this._popupListenerCallback.bind(this);

            uavLayer['marker'].on('popupopen', function(e) {
                // console.log('popupopen');
                // console.log(id);
                callback(id, true);
            });

            uavLayer['marker'].on('popupclose', function(e) {
                // console.log('popupclose');
                // console.log(id);
                callback(id, false);
            });

        } else {
            uavLayer['popup'].setContent(popupContent).update();

            if (uavLayer['popupState']) {
                uavLayer['marker'].openPopup(); // TODO: fix not open in _onDrawCallback
            } else {
                uavLayer['marker'].closePopup();
            }     
        }
    }

    /**
     * Callback to change the popup state.
     * @param {string} id - UAV id. 
     * @param {string} state - State of the popup.
     * @returns {void}
     * @access private
     */
    _popupListenerCallback(id, state) {
        this.UAV_LIST.getDictById(id)['layerPose']['popupState'] = state;
    }

    /**
     * Change all popup states depending on the draw state.
     * @param {arrray} args - List with the draw state.
     * @param {Event} e - Listener event.
     * @returns {void}
     * @access private 
     */
    _onDrawCallback(args, e) {
        for (let i = 0; i < this.UAV_LIST.getList().length; i++) {
            let id = this.UAV_LIST.getList()[i];
            this.UAV_LIST.getDictById(id)['layerPose']['popupState'] = args[0];
            this._updatePopup(id);
        }
    }
}