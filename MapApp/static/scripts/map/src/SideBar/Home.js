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
        if (M.USE_LOCAL_COORDINATES) {
            this._defaultGoTo = [0.0, 0.0];
        } else {
            this._defaultGoTo = config.Global.mapCenter;
        }

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
        if (M.USE_LOCAL_COORDINATES) {
            this._defaultUAV = [0.0, 0.0];
        } else {
            this._defaultUAV = config.Global.mapCenter;
        }

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
        let goToX = HTMLUtils.addDict('input', `${this._htmlId}-goToX`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultGoTo[0] }, 'number', M.X_NAME);
        let goToY = HTMLUtils.addDict('input', `${this._htmlId}-goToY`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultGoTo[1] }, 'number', M.Y_NAME);
        let goToBtn = HTMLUtils.addDict('button', `${this._htmlId}-goToBtn`, { 'class': 'btn btn-primary' }, 'Go to');
        let goToRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [goToX, goToY, goToBtn], { 'class': 'col-md-4' });
        let gotoCollapse = HTMLUtils.addDict('collapse', `${this._htmlId}-gotoCollapse`, {}, 'Go to', true, [goToRow]);

        // Add virtual UAV button
        let virtualUAVName = HTMLUtils.addDict('input', `${this._htmlId}-virtualName`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultUAVname}, 'text', 'Name');
        let virtualUAVX = HTMLUtils.addDict('input', `${this._htmlId}-virtualX`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultUAV[0] }, 'number', M.X_NAME);
        let virtualUAVY = HTMLUtils.addDict('input', `${this._htmlId}-virtualY`, { 'class': 'form-control', 'required': 'required', 'value': this._defaultUAV[1] }, 'number', M.Y_NAME);
        let virtualUAVPose = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [virtualUAVX, virtualUAVY], { 'class': 'col-md-6' });

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
            [`${this._htmlId}-goToX`, `${this._htmlId}-goToY`],
            ['map_center_x', 'map_center_y'],
            this._goToCallback.bind(this),
            this._defaultZoom // zoom
        );

        // Add virtual UAV callback
        Utils.addFormCallback(
            `${this._htmlId}-virtualBtn`,
            [`${this._htmlId}-virtualName`, `${this._htmlId}-virtualX`, `${this._htmlId}-virtualY`],
            ['name', 'x', 'y'],
            this._addUAVCallback.bind(this)
        );
    }

    /**
     * Go To callback, set the map center to the given coordinates.
     * @param {array} myarg - List with the zoom of the map 
     * @param {dict} input - Dictionary with the coordinates values with keys 'map_center_x' and 'map_center_y'.
     * @returns {void}
     * @access private
     */
     _goToCallback(myarg, input) {
        let coordinates = [input['map_center_x'], input['map_center_y']];
        if (M.USE_LOCAL_COORDINATES) {
            coordinates = M.UTM.getLatLng(coordinates);
        }
        M.MAP.flyTo(coordinates, arg[0]);
    }

    /**
     * Add virtual UAV callback, add a virtual UAV to the map.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {dict} input - Dictionary with keys 'name', 'x', 'y' of the virtual UAV to be added.
     * @returns {void}
     * @access private
     */
    _addUAVCallback(myarg, input) {

        let uavInfo = {
            'id': input.name,
            'state': {},
            'pose': [parseFloat(input.x), parseFloat(input.y), 0.0, 0.0]
        }

        M.WS.sendUavInfo(uavInfo);
    }
}
