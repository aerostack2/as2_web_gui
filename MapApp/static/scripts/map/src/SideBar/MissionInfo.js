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
 * Class that manage Mission Info Right Side Bar.
 */
class MissionInfo {
    /**
     * Creates a new MissionInfo instance.
     */
    constructor() {
        /**
         * Id of the div that contains the Mission Info content.
         * @type {string}
         * @access private
         */
        this._htmlId = 'sideBar-right-missionInfo-content';

        // Add callbacks to mission manager
        M.MISSION_MANAGER.addInfoAddCallback(this._addLayerCallback.bind(this));
        M.MISSION_MANAGER.addInfoChangeCallback(this._updateLayerCallback.bind(this));

        /**
         * Config file of the Mission Info.
         * @type {dict}
         * @access private
         */
        let _configFile = config.SideBars.MissionInfo;

        /**
         * Header of the table.
         * @type {array}
         * @access private
         */
        this._header = _configFile.infoTableHeader;

        /**
         * List of the keys to add to the table.
         * @type {array}
         * @access private
         */
        this._infoTable = _configFile.infoTable;

        /**
         * List of the keys to round.
         * @type {array}
         * @access private
         */
        this._roundTable = _configFile.roundTable;
    }

    /**
     * Callback to a new mission added.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the mission id.
     * @returns {void}
     * @access private
     */
    _addLayerCallback(myargs, args) {

        let missionId = args[0];

        let dict = M.MISSION_MANAGER.getDictById(missionId);
        let list = Utils.generateList(this._infoTable, dict, this._roundTable);

        let missionInfoHtml = HTMLUtils.addDict('table', `${this._htmlId}-mission-${missionId}`, {}, this._header, list);
        let missionCollapseHtml = HTMLUtils.addDict('collapse', `${this._htmlId}-UAV-${missionId}`, {}, `Mission ${missionId}`, true, [missionInfoHtml]);
        HTMLUtils.addToExistingElement(`${this._htmlId}`, [missionCollapseHtml]);

    }

    /**
     * Callback to a mission changed.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the mission id.
     * @returns {void}
     * @access private
     */
    _updateLayerCallback(myargs, args) {

        let missionId = args[0];

        let parent = document.getElementById(`${this._htmlId}-mission-${missionId}-table-body`);
        parent.innerHTML = '';

        let dict = M.MISSION_MANAGER.getDictById(missionId);
        let list = Utils.generateList(this._infoTable, dict, this._roundTable);

        for (let i = 0; i < list.length; i++) {
            let trContent = HTMLUtils.addDict('tr', ``, {}, list[i]);
            HTMLUtils.addHTML(parent, trContent);
        }
    }
}