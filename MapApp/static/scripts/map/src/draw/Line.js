/**
 * Extends the Draw Manager to enable polylines drawing.
 */
class Polyline extends DrawManager {
    /**
     * Creates a new Polyline Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {string} name - Name of the layer, for example: 'Path', 'Odom', 'DesiredPath'.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     */
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'Polyline', name, parameters, options, layerOptions);
    }

    // #region Public Methods
    // /**
    //  * Extends the Draw Manager codeDraw to assign color to the line by the UAV id.
    //  * @param {L.latlng} values - Leaflet latitude and longitude of the layer (Reference: https://leafletjs.com/).
    //  * @param {dict} options - Extra options to add to the Draw Manager. Optional.
    //  * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    //  * @param {string} uavId - UAV id. Optional.
    // * @returns {L.Layer} - Instance of the layer created (Reference: https://leafletjs.com/).
    //  * @access public
    //  */
    // codeDraw(values, options = undefined, layerOptions = {}, uavId = undefined) {
    //     if (values.length < 1) {
    //         return;
    //     }
    //     if (uavId !== undefined) {
    //         console.log(M.UAV_MANAGER.getColors(uavId));
    //         layerOptions['color'] = M.UAV_MANAGER.getColors(uavId)[1];
    //     }
    //     return super.codeDraw(values, options, layerOptions);
    // }
    /**
     * Extends the drawInfoAdd of DrawManager to add polyline vertex coordinates.
     * @param {string} htmlId - Base id of the HTML element to add.
     * @param {dict} info - Dict with the layer and the Draw Manager options.
     * @param {string} name - Name of the layer.
     * @param {array} initialHtml - List with the HTML to add at the beginning of the info.
     * @param {array} endHtml - List with the HTML to add at the end of the info.
     * @param {string} uavPickerType - Type of the UAV picker, for example 'checkbox' or 'radio'.
     * @returns {void}
     * @access public
     */
    drawInfoAdd(htmlId, info, name = info.drawManager.options.name, initialHtml = [], endHtml = undefined, uavPickerType = undefined) {

        let id = htmlId + '-' + info.id;

        let values = info.layer._latlngs;
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lat, 6) }, 'number', 'Latitude');
            let lngDict = HTMLUtils.addDict('input', `${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lng, 6) }, 'number', 'Longitude');
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], { 'class': 'col-6' });

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
            htmlId.push(`${id}-${i}-lat`);
            htmlId.push(`${id}-${i}-lng`);

            nameId.push(`${i}-lat`);
            nameId.push(`${i}-lng`);
        }
        Utils.addFormCallback(`${id}-change`, htmlId, nameId, this._changeCallback.bind(this), info);
    }

    /**
     * Overload the Draw Manager _changeCallback to change the coordinates of the line.
     * @param {array} myargs - List with the info dict, that has the layer and the Draw Manager options.
     * @param {dict} args - Dict with ['${id}-${i}-lat', '${id}-${i}-lng'] as keys and their values, for each vertex of the polyline.
     * @returns {void}
     * @access private
     */
    _changeCallback(myargs, args) {
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
            if (keyParse[1] == 'lat') {
                values[index1][0] = value;
            } else if (keyParse[1] == 'lng') {
                values[index1][1] = value;
            }
        }
        layer.setLatLngs(values);
    }

    // #endregion
}

/**
 * Type of Polyline, use to draw the UAV path.
 */
class Path extends Polyline {
    /**
     * Creates a new Path Polyline Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     */
    constructor(status, options = undefined, layerOptions = undefined, parameters = config.Layers.Line.Path.parameters) {
        super(status, 'Path', parameters, options, layerOptions);
    }
}

/**
 * Type of Polyline, use to draw the UAV odom.
 */
class Odom extends Polyline {
    /**
     * Creates a new Path Polyline Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     */
    constructor(status, options = undefined, layerOptions = undefined, parameters = undefined) {
        super(status, 'Odom', parameters, options, layerOptions);
    }
}

/**
 * Type of Polyline, use to draw the UAV desired path.
 */
class DesiredPath extends Polyline {
    /**
     * Creates a new Path Polyline Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     */
    constructor(status, options = undefined, layerOptions = undefined, parameters = undefined) {
        super(status, 'DesiredPath', parameters, options, layerOptions);
    }
}