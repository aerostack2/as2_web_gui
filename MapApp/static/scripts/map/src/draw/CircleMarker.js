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
 * Extends the Draw Manager to enable circle markers drawing.
 */
class CircleMarker extends DrawManager
{
    /**
     * Creates a new Circle Marker Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {string} layerName - Name of the layer, for example: 'PointOfAction'.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     */
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'CircleMarker', name, parameters, options, layerOptions);
    }

    /**
     * TODO: Manage to draw circle markers by code.
     * @param {L.latlng} values - Leaflet latitude and longitude of the layer (Reference: https://leafletjs.com/).
     * @param {dict} options - Extra options to add to the Draw Manager.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options.
     * @returns {object} - Instance of the layer created.
     * @access public
     */
    codeDraw(values, options = {}, layerOptions = {}) {
        let drawOption = this._mergeOptions(options, layerOptions);
        let draw = null;

        throw new Error("Not implemented");
        
    }
}
