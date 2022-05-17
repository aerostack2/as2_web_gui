/**
 * Class for drawing mission on the map when it is recieved from the server.
 */
class MissionDrawer {
    /**
     * Creates a new MissionDrawer instance.
     */
    constructor() {
        // Add callbacks to mission manager
        M.MISSION_MANAGER.addMissionConfirmCallback(this._missionConfirmCallback.bind(this));
        M.MISSION_MANAGER.addInfoAddCallback(this.newMissionCallback.bind(this));

        // Add draw types of mission layers
        this._addDrawTypes();
    }

    /**
     * Creates an instance of each type of mission layers.
     * @returns {void}
     * @access private
     */
    _addDrawTypes() {
        let status = 'confirmed';

        /**
         * Take off manager.
         * @type {TakeOffPoint}
         * @access private
         */
        this._takeOffPoint = new TakeOffPoint(status);

        /**
         * Land point manager.
         * @type {LandPoint}
         * @access private
         */
        this._landPoint = new LandPoint(status);

        /**
         * Waypoint point manager.
         * @type {WayPoint}
         * @access private
         */
        this._wayPoint = new WayPoint(status);

        /**
         * Path manager.
         * @type {Path}
         * @access private
         */
        this._path = new Path(status, undefined, { 'opacity': 0.6, 'weight': 4 });
        
        /**
         * Area manager.
         * @type {Area}
         * @access private
         */
        this._area = new Area(status);
    }

    /**
     * Callback to mission confirm button. Remove draw layers.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {dict} args - Dictionary with key 'oldId' with the id of the mission draw layers to be removed.
     * @returns {void}
     * @access private
     */
    _missionConfirmCallback(myargs, args) {
        let missionId = args['oldId'];
        let layers = M.DRAW_LAYERS.getList();

        for (let i = 0; i < layers.length; i++) {
            let layer = M.DRAW_LAYERS.getDictById(layers[i]);

            if (layer.drawManager.options.missionId == missionId && layer.drawManager.options.status == 'draw') {
                M.DRAW_LAYERS.removeLayerById(id);
            }
        }
    }

    /**
     * Callback to a new mission added. Read the mission and draw 
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {dict} args - Dictionary with key 'oldId' with the id of the mission draw layers to be removed.
     * @returns {void}
     * @access private
     */
    newMissionCallback(myargs, args) {
        let missionId = args[0];
        let missionDict = M.MISSION_MANAGER.getDictById(missionId);

        for (let i = 0; i < missionDict.layers.length; i++) {
            let layer = missionDict.layers[i];
            let uavId = layer['uavList'][0];
            let desiredColor = M.UAV_MANAGER.getColors(uavId);

            switch (layer.name) {
                case 'TakeOffPoint':
                    this._takeOffPoint.codeDraw([layer.values['lat'], layer.values['lng']], {'missionId': missionId}, undefined, desiredColor);
                    break;
                case 'Path':
                    this._path.codeDraw(layer.values, {'missionId': missionId}, {'color': desiredColor[0]});
                    break;
                case 'LandPoint':
                    this._landPoint.codeDraw([layer.values.lat, layer.values.lng], {'missionId': missionId}, undefined, desiredColor);
                    break;
                case 'WayPoint':
                    this._wayPoint.codeDraw([layer.values.lat, layer.values.lng], {'missionId': missionId}, undefined, desiredColor);
                    break;
                case 'Area':
                    this._area.codeDraw(layer.values[0], {'missionId': missionId}, {'opacity': 0.3, 'color': M.MISSION_MANAGER.getColors(missionId)[1]});

                    for (let j = 0; j < layer.uavList.length; j++) {
                        let uavId_aux = layer.uavList[j];
                        this._path.codeDraw(layer.uavPath[uavId_aux], {'missionId': missionId}, undefined, {'color': desiredColor[0]});
                    }

                    break;
                default:
                    throw new Error("Unknown layer name: " + layer.name);
                    break;
            }
        }
        ConsoleSideBar.addMessage("Mission " + missionId + " added");
    }
}