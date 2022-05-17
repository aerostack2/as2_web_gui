/**
 * Class that manage Layers Control, to add or remove group of layers by a checkbox.
 */
class LayersControl {
    /**
     * Creates a new LayersControl instance.
     */
    constructor() {
        // Add layers add/remove callbacks
        M.DRAW_LAYERS.addInfoAddCallback(this._addDrawLayerCallback.bind(this));
        M.MISSION_LAYERS.addInfoAddCallback(this._addMissionLayerCallback.bind(this));
        M.UAV_LAYERS.addInfoAddCallback(this._addUavLayerCallback.bind(this));

        /**
         * Control for draw layer.
         * @type {L.Control}
         * @access private
         */
        this._drawControlLayer = L.layerGroup();

        // Add control to map        
        this._drawControlLayer.addTo(M.MAP);
        M.layerControl.addOverlay(this._drawControlLayer, 'Draw Layers');

        /**
         * SmartList that contain control for each UAV.
         * @type {SmartList}
         * @access private
         */
        this._uavControlLayers = new SmartList();

        /**
         * SmartList that contain control for each mission.
         * @type {SmartList}
         * @access private
         */
        this.missionControlLayers = new SmartList();
    }

    /**
     * Add layer to draw control when it is created.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the id of the layer and if the layer is added or removed
     * @returns {void}
     * @access private
     */
    _addDrawLayerCallback(myargs, args) {
        let layerId = args[0];
        
        if (args[1] == 'add') {
            let info = M.DRAW_LAYERS.getDictById(layerId);
            this._drawControlLayer.addLayer(info.layer);
        }
    }

    /**
     * Add layer to mission control when it is created.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the id of the layer and if the layer is added or removed
     * @returns {void}
     * @access private
     */
     _addMissionLayerCallback(myargs, args) {
        let layerId = args[0];
        
        if (args[1] == 'add') {
            let info = M.MISSION_LAYERS.getDictById(layerId);
            let missionId = info.drawManager.options.missionId;
            let dict = this.missionControlLayers.getDictById(missionId);
            if (dict == null) {
                let missionControlLayer = L.layerGroup();
                missionControlLayer.addTo(M.MAP);
                M.layerControl.addOverlay(missionControlLayer, missionId);

                this.missionControlLayers.addObject(missionId, {'controlLayer': missionControlLayer});
            }
            dict = this.missionControlLayers.getDictById(missionId);
            dict.controlLayer.addLayer(info.layer);
        }
    }

    /**
     * Add layer to UAV control when a UAV is added.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the id of the layer and if the layer is added or removed
     * @returns {void}
     * @access private
     */
    _addUavLayerCallback(myargs, args) {
        let layerId = args[0];
        
        if (args[1] == 'add') {
            let info = M.UAV_LAYERS.getDictById(layerId);
            let uavId = info.drawManager.options.uavList[0];
            let dict = this._uavControlLayers.getDictById(uavId);
            if (dict == null) {
                let uavControlLayer = L.layerGroup();
                uavControlLayer.addTo(M.MAP);
                M.layerControl.addOverlay(uavControlLayer, uavId);

                this._uavControlLayers.addObject(uavId, {'controlLayer': uavControlLayer});
            }
            dict = this._uavControlLayers.getDictById(uavId);
            dict.controlLayer.addLayer(info.layer);

        }
    }
} 