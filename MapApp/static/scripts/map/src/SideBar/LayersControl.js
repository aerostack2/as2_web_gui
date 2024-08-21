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