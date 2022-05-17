/**
 * List of key and its corresponding class, to be used to generate HTML elements
 * @type {array}
 * @access private
 */
blocksClassList = [
    ['button', Button],
    ['checkBox', CheckBox],
    ['checkBoxes', CheckBoxes],
    ['collapse', Collapse],
    ['div', Div],
    ['dropDown', DropDown],
    ['dropDownBtn', DropDownBtn],
    ['dropDownExpand', DropDownExpand],
    ['input', Input],
    ['label', Label],
    ['splitDivs', SplitDivs],
    ['table', Table],
    ['tr', Tr],
    ['fileInput', FileInput],
]

window.onload = function () {

    /**
     * Config object, that contains all the configuration variables.
     * @type {Config}
     * @access private
     */
    config = new Config('config.json', initialize);

    /**
     * Start the map when all config files are loaded.
     * @returns {void}
     */
    function initialize() {

        /**
         * Map Manager global variable to be accessed from other modules
         * @type {MapManager}
         * @global
         * @public
         */
        M = new MapManager(
            config.Global.mapCenter,
            config.Global.mapZoom,
            config.WebSocket.connectionString
        );

        M.initialize();

        /**
         * List of classes to be instantiated before the map is loaded
         */
        var sideBarsClass = [
            new Home(),
            new MissionPlanner(),
            new MissionController(),
            new UavDrawer(),
            new MissionDrawer(),
            new UavInfo(),
            new DrawInfo(),
            new MissionInfo(),
            new ImageOVerlay(),
            new LayersControl(),
        ]

        // Instantiate sidebars elements
        for (let i = 0; i < sideBarsClass.length; i++) {
            sideBarsClass[i];
        }
    }

    window.onerror = function (msg, url, line, col, error) {
        var extra = !col ? '' : '\ncolumn: ' + col;
        extra += !error ? '' : '\nerror: ' + error;
        console.log('Error: ' + msg + '\nurl: ' + url + '\nline: ' + line + extra);
        ConsoleSideBar.addError('Error: ' + msg + '\nurl: ' + url + '\nline: ' + line + extra);
    }
}   

