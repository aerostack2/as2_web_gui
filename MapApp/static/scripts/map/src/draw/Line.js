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
    drawInfoAdd(htmlId, info, name = info.drawManager.options.name, initialHtml = [], endHtml = undefined, uavPickerType = 'radio') {

        let id = htmlId + '-' + info.id;

        let values = info.layer._latlngs;
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
     * @param {dict} args - Dict with ['${id}-${i}-x', '${id}-${i}-y'] as keys and their values, for each vertex of the polyline.
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