/**
 * Extends the Draw Manager to enable polygon drawing.
 */
class Polygon extends DrawManager {
    /**
     * Creates a new Polygon Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {string} name - Name of the layer, for example: 'Path', 'Odom', 'DesiredPath'.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     */
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'Polygon', name, parameters, options, layerOptions);
    }

    // #region Public Methods

    /**
     * Extends the drawInfoAdd of DrawManager to add polygon vertex coordinates.
     * @param {string} htmlId - Base id of the HTML element to add.
     * @param {dict} info - Dict with the layer and the Draw Manager options.
     * @param {string} name - Name of the layer.
     * @param {array} initialHtml - List with the HTML to add at the beginning of the info.
     * @param {array} endHtml - List with the HTML to add at the end of the info.
     * @param {string} uavPickerType - Type of the UAV picker, for example 'checkbox' or 'radio'.
     * @returns {void}
     * @access public
     */
    drawInfoAdd(htmlId, info, name = info.drawManager.options.name, initialHtml = [], endHtml = undefined, uavPickerType = 'checkbox') {

        let id = htmlId + '-' + info.id;

        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            let coordinates = [values[i].lat, values[i].lng];
            if (M.USE_LOCAL_COORDINATES) {
                coordinates = M.UTM.getLocalUTM(coordinates);
            }

            let xDict = HTMLUtils.addDict('input', `${id}-${i}-x`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(coordinates[0], 6) }, 'number', M.X_NAME);
            let yDict = HTMLUtils.addDict('input', `${id}-${i}-y`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(coordinates[1], 6) }, 'number', M.Y_NAME);
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [xDict, yDict], { 'class': 'col-6' });

            initialHtml.push(row);
        }

        return super.drawInfoAdd(htmlId, info, name, initialHtml, endHtml, uavPickerType);
    }

    // #endregion

    // #region Private Methods

    /**
     * Overload the Draw Manager _addChangeCallback manage change coordinates of the line.
     * @param {string} id - Base id of the HTML element to add.
     * @param {dict} info - Dict with the layer and the Draw Manager options.
     * @returns {void}
     * @access private
     */
    _addChangeCallback(id, info) {
        let htmlId = [];
        let nameId = [];
        let values = info.layer._latlngs;
        for (let i = 0; i < values.length; i++) {
            htmlId.push(`${id}-${i}-x`);
            htmlId.push(`${id}-${i}-y`);

            nameId.push(`${i}-x`);
            nameId.push(`${i}-y`);
        }
        Utils.addFormCallback(`${id}-change`, htmlId, nameId, this._changeCallback.bind(this), info);
    }

    /**
     * Overload the Draw Manager _changeCallback to change the coordinates of the line.
     * @param {array} myargs - List with the info dict, that has the layer and the Draw Manager options.
     * @param {dict} args - Dict with ['${id}-${i}-x', '${id}-${i}-y'] as keys and their values, for each vertex of the polygon.
     * @returns {void}
     * @access private
     */
    _changeCallback(myargs, inputs) {
        let layer = myargs[0].layer;

        let values = [];
        let len = Math.floor(Object.keys(args).length / 2);
        for (let i = 0; i < len; i++) {
            values.push([]);
        }
        for (var key in args) {
            let value = args[key];
            let keyParse = key.split('-');
            let index1 = parseInt(keyParse[0]);
            if (keyParse[1] == 'x') {
                values[index1][0] = value;
            } else if (keyParse[1] == 'y') {
                values[index1][1] = value;
            }
        }
        let coordinates = values;
        if (M.USE_LOCAL_COORDINATES) {
            coordinates = M.UTM.getLatLngs(values);
        }
        layer.setLatLngs(values);
    }

    // #endregion
}

/**
 * Type of Polygon, use to draw the mission area.
 */
class Area extends Polygon {
    constructor(status, options = {}, layerOptions = undefined, parameters = config.Layers.Polygon.Area.parameters) {
        super(status, 'Area', parameters, options, layerOptions);

        /**
         * Config dict of the Area.
         * @type {dict}
         * @access public
         */
        this.configFile = config.Layers.Polygon.Area;
    }
}
