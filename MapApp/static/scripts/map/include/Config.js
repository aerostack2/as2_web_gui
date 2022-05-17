/**
 * Config class that read all config files and store them in a single object.
 */
class Config 
{
    /**
     * Create a new Config object.
     * @param {string} filePath - Path to the config file to load.
     * @param {function} callback - The callback function to be called when all config files are loaded.
     */
    constructor(filePath, callback) {
        /**
         * The base path of all config files.
         * @type {string}
         * @access private
         */
        this._basePath = '/static/scripts/map/Config/';
        
        /**
         * Counter of config files loaded.
         * @type {number}
         * @access private
         */
        this._cont = 0;

        /**
         * Number of config files to be loaded.
         * @type {number}
         * @access private
         */
        this._dataLenght = null;

        /**
         * Counter of config SVG loaded.
         * @type {number}
         * @access private
         */
         this._contSvg = 0;

        /**
         * Number of SVG files to be loaded.
         * @type {number}
         * @access private
         */
        this._dataSVGLenght = null;

        // Load main config file
        Utils.loadLocalFile(this._basePath + filePath, this._intialize.bind(this), 'json', callback);
    }

    /**
     * For each config file in the main file, load it and store it in the Config object.
     * @param {JSON} data - JSON object with all config files, been the key the name of the member in the Config object and the value the path to the config file.
     * @param {function} callback - The callback function to be called when all config files are loaded.
     * @access private
     * @return {void}
     */
    _intialize(data, callback) {
        this._dataLenght = Object.keys(data).length;

        for (let i = 0; i < Object.keys(data).length; i++) {
            let key = Object.keys(data)[i];
            let filePath = data[key];
            if (key == "Markers") {
                Utils.loadLocalFile(this._basePath + filePath, this._onLoadConfigMarkersFile.bind(this), 'json', key, callback[0]);
            } else {
                Utils.loadLocalFile(this._basePath + filePath, this._onLoadConfigFile.bind(this), 'json', key, callback[0]);
            }
        }
    }

    /**
     * Store the config file in the Config object.
     * @param {JSON} data - JSON object with the config file content.
     * @param {array} args - List with the callback function to be called when all config files are loaded.
     */
    _onLoadConfigFile(data, args) {
        this[args[0]] = data;

        this._cont++;
        if (this._cont == this._dataLenght) {
            args[1]();
        }
    }

    _onLoadConfigMarkersFile(data, args) {
        this[args[0]] = data;
        this._dataSVGLenght = Object.keys(data).length;

        for (let i = 0; i < Object.keys(data).length; i++) {
            let key = Object.keys(data)[i];
            let content = data[key];
            
            Utils.loadLocalFile(content.svgUrl, this._onLoadSVGFile.bind(this), 'text', args[0], key, args[1]);
        }
    }

    _onLoadSVGFile(data, args) {
        let markerName = args[0][0];
        let markerType = args[0][1];
        let callback = args[0][2];

        this[markerName][markerType].svg = data;

        this._contSvg++;
        if (this._contSvg == this._dataSVGLenght) {
            this._cont++;
            if (this._cont == this._dataLenght) {
                callback();
            }
        }
    }
}