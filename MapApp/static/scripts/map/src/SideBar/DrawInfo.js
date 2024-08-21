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
 * Class that manage Draw Info Right Side Bar.
 */
class DrawInfo {
    /**
     * Creates a new DrawInfo instance.
     */
    constructor() {
        /**
         * Id of the div that contains the draw info.
         * @type {string}
         * @access private
         */
        this._htmlId = 'sideBar-right-drawInfo-content';

        // Add callbacks
        M.DRAW_LAYERS.addInfoAddCallback(this._addLayerCallback.bind(this));
        M.DRAW_LAYERS.addInfoChangeCallback(this.updateLayerCallback.bind(this));
    }

    /**
     * This method is called when a new layer is added to the map or when a layer is removed from the map. Add the layer info to the side bar.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the layer id and a flag to indicate if the layer is added or removed.
     * @returns {void}
     * @access private
     */
    _addLayerCallback(myargs, args) {

        if (args[1] == 'add') {
            let info = M.DRAW_LAYERS.getDictById(args[0]);
            info.drawManager.instance.drawInfoAdd(this._htmlId, info);
        } else if (args[1] == 'remove') {
            let info = M.DRAW_LAYERS.getDictById(args[0]);
            if (info != null) {
                info.drawManager.instance.drawInfoRemove(this._htmlId + '-' + info.id);
            }
        }
    }

    /**
     * This method is called when a layer is update, changing some of its properties. Update the layer info in the side bar.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the layer id.
     * @returns {void}
     * @access private
     */
    updateLayerCallback(myargs, args) {
        let info = M.DRAW_LAYERS.getDictById(args[0]);
        if (info != null) {
            info.drawManager.instance.drawInfoAdd(this._htmlId, info);
        }
    }
}