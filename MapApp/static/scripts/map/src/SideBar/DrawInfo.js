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