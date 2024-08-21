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
 * Draw Controller for the user to draw on the map.
 */
class DrawController {
    /**
     * Disable all draw modes
     * @return {void}
     * @access public
     * @static
     */
    static drawMouse() {
       
        if (M.MAP.pm.globalDrawModeEnabled()) {
            M.MAP.pm.disableDraw();
        } else if (M.MAP.pm.globalEditModeEnabled()) {
            M.MAP.pm.toggleGlobalEditMode();
        } else if (M.MAP.pm.globalRemovalModeEnabled()) {
            M.MAP.pm.toggleGlobalRemovalMode();
        } else if (M.MAP.pm.globalDragModeEnabled()) {
            M.MAP.pm.toggleGlobalDragMode();
        } else if (M.MAP.pm.globalRotateModeEnabled()) {
            M.MAP.pm.toggleGlobalRotateMode();
        }
    }

    /**
     * Enable edit mode
     * @return {void}
     * @access public
     * @static
     */
    static drawEdit() {
        M.MAP.pm.toggleGlobalEditMode();
    }

    /**
     * Enable delete mode
     * @return {void}
     * @access public
     * @static
     */
    static drawDelete() {
        M.MAP.pm.toggleGlobalRemovalMode();
    }

    /**
     * Enable remove mode
     * @return {void}
     * @access public
     * @static
     */
    static drawMove() {
        M.MAP.pm.toggleGlobalDragMode();
    }

    /**
     * Enable rotate mode
     * @return {void}
     * @access public
     * @static
     */
    static drawRotate() {
        M.MAP.pm.toggleGlobalRotateMode();
    }

    /**
     * Remove all draw layers that are not confirmed in a mission
     * @return {void}
     * @access public
     * @static
     */
    static drawRemoveAll() {
        let drawLayers = Utils.deepCopy(M.DRAW_LAYERS.getList());

        for (let i = 0; i < drawLayers.length; i++) {
            M.DRAW_LAYERS.removeLayerById(drawLayers[i]);
        }
    }
}