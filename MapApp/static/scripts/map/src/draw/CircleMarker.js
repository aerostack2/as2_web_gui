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
        
        draw.addTo(M.MAP);
        this.codeLayerDrawn = draw;

        if (drawOption.draggable == false) {
            this.codeLayerDrawn.pm.setOptions({ draggable: false });
        }
        return draw;
    }
}
