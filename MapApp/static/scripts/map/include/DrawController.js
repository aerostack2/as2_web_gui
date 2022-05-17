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