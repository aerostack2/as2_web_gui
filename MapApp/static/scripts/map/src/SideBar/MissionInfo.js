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