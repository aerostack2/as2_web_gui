/**
 * Class that manage Home Left Side Bar.
 */
class Home {
    /**
     * Creates a new Home instance.
     */
    constructor() {
        /**
         * Id of the div that contains the home content.
         * @type {string}
         * @access private
         */
        this._htmlId = 'sideBar-left-home-content';

        /**
         * Config file of the home.
         * @type {dict}
         * @access private
         */
        let _configFile = config.SideBars.Home;

        /**
         * Default value of Go To loaded from config file.
         * @type {array}
         * @access private
         */
        this._defaultGoTo = _configFile.goTo;

        /**
         * Default value of Go To Zoom loaded from config file.
         * @type {array}
         * @access private
         */
        this._defaultZoom = _configFile.goToZoom;

        /**
         * Default value of virtual UAV name loaded from config file.
         * @type {string}
         * @access private
         */
        this._defaultUAVname = _configFile.UAVname;

        /**
         * Default value of latitude and longitude for the virtual UAV loaded from config file.
         * @type {array}
         * @access private
         */
        this._defaultUAV = _configFile.UAVvalue;

        // Initialize the home
        this._addHTML();
        this._addCallbacks();
    }

    /**
     * Add HTML content to the home side bar.
     * @returns {void}
     * @access private
     */
    _addHTML() {
        let homeHtmlList = [];

        // Go to
        let goToLat = HTMLUtils.addDict('input', `${this._htmlId}-goToLat`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultGoTo[0] }, 'number', 'Latitude');
        let goToLng = HTMLUtils.addDict('input', `${this._htmlId}-goToLng`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultGoTo[1] }, 'number', 'Longitude');
        let goToBtn = HTMLUtils.addDict('button', `${this._htmlId}-goToBtn`, { 'class': 'btn btn-primary' }, 'Go to');
        let goToRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [goToLat, goToLng, goToBtn], { 'class': 'col-md-4' });
        let gotoCollapse = HTMLUtils.addDict('collapse', `${this._htmlId}-gotoCollapse`, {}, 'Go to', true, [goToRow]);

        // Add virtual UAV button
        let virtualUAVName = HTMLUtils.addDict('input', `${this._htmlId}-virtualName`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultUAVname}, 'text', 'Name');
        let virtualUAVLat = HTMLUtils.addDict('input', `${this._htmlId}-virtualLat`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultUAV[0] }, 'number', 'Latitude');
        let virtualUAVLon = HTMLUtils.addDict('input', `${this._htmlId}-virtualLng`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultUAV[1] }, 'number', 'Longitude');
        let virtualUAVPose = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [virtualUAVLat, virtualUAVLon], { 'class': 'col-md-6' });

        let virtualBtnContent = HTMLUtils.addDict('button', `${this._htmlId}-virtualBtn`, { 'class': 'btn btn-primary' }, 'Add virtual UAV');
        let virtualBtn = HTMLUtils.addDict('splitDivs', 'none', {}, [virtualBtnContent], { 'class': 'row m-1' });

        let virtualCollapse = HTMLUtils.addDict('collapse', `${this._htmlId}-virtualCollapse`, {}, 'Add virtual UAV', true, [virtualUAVName, virtualUAVPose, virtualBtn]);

        // Add each element to html list
        homeHtmlList.push(gotoCollapse);
        homeHtmlList.push(virtualCollapse);

        // Add the html list to the side bar content
        HTMLUtils.addToExistingElement(`${this._htmlId}`, homeHtmlList);
    }

    /**
     * Add callbacks to inputs.
     * @returns {void}
     * @access private
     */
    _addCallbacks() {
        // Go To callback
        Utils.addFormCallback(
            `${this._htmlId}-goToBtn`,
            [`${this._htmlId}-goToLat`, `${this._htmlId}-goToLng`],
            ['map_center_lat', 'map_center_lng'],
            this._goToCallback.bind(this),
            this._defaultZoom // zoom
        );

        // Add virtual UAV callback
        Utils.addFormCallback(
            `${this._htmlId}-virtualBtn`,
            [`${this._htmlId}-virtualName`, `${this._htmlId}-virtualLat`, `${this._htmlId}-virtualLng`],
            ['name', 'lat', 'lng'],
            this._addUAVCallback.bind(this)
        );
    }

    /**
     * Go To callback, set the map center to the given coordinates.
     * @param {array} myarg - List with the zoom of the map 
     * @param {dict} input - Dictionary with the coordinates values with keys 'map_center_lat' and 'map_center_lng'.
     * @returns {void}
     * @access private
     */
     _goToCallback(myarg, input) {
        M.MAP.flyTo([input['map_center_lat'], input['map_center_lng']], arg[0]);
    }

    /**
     * Add virtual UAV callback, add a virtual UAV to the map.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {dict} input - Dictionary with keys 'name', 'lat', 'lng' of the virtual UAV to be added.
     * @returns {void}
     * @access private
     */
    _addUAVCallback(myarg, input) {

        let uavInfo = {
            'id': input.name,
            'state': {},
            'pose': {'lat': parseFloat(input.lat), 'lng': parseFloat(input.lng), 'height': 0, 'yaw': 0}
        }

        M.WS.sendUavInfo(uavInfo);
    }
}
