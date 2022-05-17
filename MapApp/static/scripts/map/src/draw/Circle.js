/**
 * Extends the Draw Manager to enable circle area drawing.
 */
class Circle extends DrawManager {
    /**
     * Creates a new Circle Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {string} layerName - Name of the layer, for example: 'CircularArea'.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     */
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'Circle', name, parameters, options, layerOptions);
    }
}

/**
 * Type of Circle Area, use to draw mission area.
 */
class CircularArea extends Circle {
    /**
     * Creates a new Circle Area Draw Manager.
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {dict} options - Options of the Draw Manager. Optional.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
     * @param {array} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
     */
    constructor(status, options = undefined, layerOptions = undefined, parameters = undefined) {
        super(status, 'CircularArea', parameters, options, layerOptions);
    }
}
