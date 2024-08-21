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
 * List of key and its corresponding class, to be used to generate HTML elements
 * @type {array}
 * @access private
 */
blocksClassList = [
    ['button', Button],
    ['checkBox', CheckBox],
    ['checkBoxes', CheckBoxes],
    ['collapse', Collapse],
    ['collapse-order', CollapseOrder],
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
    config = new Config(initialize);

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

