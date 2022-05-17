/**
 * Class that manage Mission Controller Left Side Bar.
 */
class MissionController {
    /**
     * Creates a new MissionController instance.
     */
    constructor() {
        /**
         * Id of the div that contains the mission controller content.
         * @type {string}
         * @access private
         */
        this.htmlId = 'sideBar-left-missionController-content';

        /**
         * Flag to know if the side bar is initialized.
         * @type {boolean}
         * @access private
         */
        this._initialized = false;

        /**
         * Id of the selected mission to control.
         * @type {string}
         * @access private
         */
        this._selectedMissionId = null;

        // Add callback to a new mission added
        M.MISSION_MANAGER.addInfoAddCallback(this._updateMissionListCallback.bind(this));

        // Initialize the mission controller with a file input
        this._initializedLoad();
    }

    // #region Initialize

    /**
     * Initialize the mission controller with a file input, to load missions.
     * @returns {void}
     * @access private
     */
    _initializedLoad() {
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [HTMLUtils.addDict('fileInput', `${this.htmlId}-missionFile`, {}, 'Choose Mission File', '')]);
        Utils.addFileCallback(`${this.htmlId}-missionFile`, this._loadMissionCallback.bind(this));
    }

        /**
     * Callback for load mission input. Load mission from file.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {Event} input - Event of the input file target.
     * @returns {void}
     * @access private
     */
    _loadMissionCallback(args, input) {
        let fileToLoad = input.target.files[0];
        Utils.loadFile(fileToLoad, this._loadMissionFileCallback.bind(this), 'json', fileToLoad.name.split('.')[0]);
    }

    /**
     * Callback for load file. Process the file content and send the mission to the server.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {Event} input - Event of the input file target.
     * @returns {void}
     * @access private
     */
    _loadMissionFileCallback(data, myargs) {

        Utils.resetLoadFileInput(`${this.htmlId}-missionFile`);

        let id = myargs;
        let missionList = M.MISSION_MANAGER.getList();
        let cont = 0;
        while (missionList.includes(id)) {
            id = `${myargs}-${cont}`;
            cont++;
        }

        data.id = id;

        let uavList = M.UAV_MANAGER.getList();
        let missionUavList = data.uavList;

        for (let i = 0; i < missionUavList.length; i++) {
            let uavId = missionUavList[i];
            if (!uavList.includes(uavId)) {
                alert(`UAV: ${uavId} from file ${myargs} is not connected`);
                return;
            }
        }

        M.WS.sendConfirmedMission(data);
    }

    // #endregion

    // #region Callbacks

    /**
     * Check if HTML has been already initialized. If not, create it.
     * @returns {void}
     * @access private
     */
     _checkInitalize() {
        if (!this._initialized) {
            this._addHTML();
            this._initialized = true;
        }
    }

    /**
     * Add HTML content to the Mission Controller side bar. 
     * @returns {void}
     * @access private
     */
    _addHTML() {
        this._selectedMissionId = M.MISSION_MANAGER.getList()[0];

        let missionControllerHtmlList = [];

        // Mission Dropdown list
        missionControllerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-MissionList`, M.MISSION_MANAGER.getList(), M.MISSION_MANAGER.getList()[0]));

        // Buttons for draw mission
        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-primary',}, `Reload missions`));
        // splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-edit`,   {'class': 'btn btn-primary',}, `Edit mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-save`,   {'class': 'btn btn-primary',}, `Save mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-center`, {'class': 'btn btn-primary',}, `Center mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-start`,  {'class': 'btn btn-primary',}, `Start mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-stop`,   {'class': 'btn btn-danger'},   'Stop mission'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-end`,    {'class': 'btn btn-success'},  'End mission'));
        missionControllerHtmlList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, {'class': 'row m-1'}));

        // Global collapse
        let missionControllerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-addToMapCollapse`, {}, 'Controller', true, missionControllerHtmlList);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionControllerCollapse]);

        this._addCallbacks();
    }

    /**
     * Add callbacks to inputs.
     * @returns {void}
     * @access private
     */
    _addCallbacks() {
        Utils.addButtonCallback(`${this.htmlId}-btn-reload`, this._reloadMissionCallback.bind(this), []);
        // Utils.addButtonCallback(`${this.htmlId}-btn-edit`,   this._editMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-save`,   this.saveMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-center`, this._centerMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-start`,  this._startMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-stop`,   this._stopMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-end`,    this._endMissionCallback.bind(this), []);
    }

    // #region Buttons callbacks

    /**
     * Callback to new mission added. Update the mission dropdown menu and callbacks.
     * @param {array} myargs - List of arguments passed to the callback. Empty.
     * @param {dict} args - Dictionary. Empty.
     * @returns {void}
     * @access private
     */
    _updateMissionListCallback(myargs, args) {
        this._checkInitalize();
        HTMLUtils.updateDropDown(`${this.htmlId}-MissionList`, M.MISSION_MANAGER.getList());
        Utils.addButtonsCallback(`${this.htmlId }-MissionList-item`, this._clickMissionListCallback.bind(this));
        this._selectedMissionId = M.MISSION_MANAGER.getList()[0];
    }

    /**
     * Callback for dropdown menu click. Change selected mission.
     * @param {Event} e - Button click event.
     * @param {dict} args - Dictionary. Empty.
     * @returns {void}
     * @access private
     */
    _clickMissionListCallback(e, args) {
        let button = document.getElementById(`${this.htmlId}-MissionList-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
        this._selectedMissionId = e.innerHTML;
    }

    /**
     * Callback for start mission button. Send a start mission message to the server.
     * @param {Event} e - Button click event.
     * @returns {void}
     * @access private
     */
    _startMissionCallback(e) {
        M.WS.sendStartMission(this._selectedMissionId);
    }

    /**
     * Callback for edit mission button. TODO: Not implemented yet.
     * @param {Event} e - Button click event.
     * @returns {void}
     * @access private
     */
    _editMissionCallback(e) {
        console.log(`TODO: Edit mission ${this._selectedMissionId}`);
        // TODO: Implement
    }

    /**
     * Callback for center mission button. Center map to the first waypoint of the mission.
     * @param {Event} e - Button click event.
     * @returns {void}
     * @access private
     */
    _centerMissionCallback(e) {
        let missionDict = M.MISSION_MANAGER.getDictById(this._selectedMissionId);
        let center = missionDict.layers[0].values;

        if (center.lat !== undefined && center.lng !== undefined) {
            M.MAP.flyTo(center, M.MAP.getZoom());
        } else if (center[0].lat !== undefined && center[0].lng !== undefined) {
            M.MAP.flyToBounds(center, M.MAP.getZoom());
        }
    }

    /**
     * Callback for stop mission button. Send a stop mission message to the server.
     * @param {Event} e - Button click event.
     * @returns {void}
     * @access private
     */
    _stopMissionCallback(e) {
        M.WS.sendStopMission(this._selectedMissionId);
    }

    /**
     * Callback for end mission button. Send an end mission message to the server.
     * @param {Event} e - Button click event.
     * @returns {void}
     * @access private
     */
    _endMissionCallback(e) {
        M.WS.sendEndMission(this._selectedMissionId);
    }

    /**
     * Callback for reload mission button. Send an end mission message to the server.
     * @param {Event} e - Button click event.
     * @returns {void}
     * @access private
     */
    _reloadMissionCallback(e) {
        let missionLayers = M.MISSION_LAYERS.getList();
        for (let i = 0; i < missionLayers.length; i++) {
            M.MISSION_LAYERS.removeLayerById(missionLayers[i]);
        }

        M.WS.sendRequestGetMissionList();
    }

    /**
     * Callback for save mission button. Send a save mission message to the server.
     * @param {Event} e - Button click event.
     * @returns {void}
     * @access private
     */
    _saveMissionCallback(e) {
        let missionId = this._selectedMissionId;
        let missionDict = M.MISSION_MANAGER.getDictById(missionId);

        Utils.download(`Mission-${missionId}.txt`, missionDict);
    }

    // #endregion
}