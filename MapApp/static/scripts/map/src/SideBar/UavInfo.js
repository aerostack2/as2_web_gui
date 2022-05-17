/**
 * Class that manage UAV Info Right Side Bar.
 */
class UavInfo
{
    /**
     * Creates a new UavInfo instance.
     */
    constructor() {
        /**
         * Id of the div that contains the UAV Info content.
         * @type {string}
         * @access private
         */
        this._htmlId = 'sideBar-right-UAVInfo-content';

        // Add callbacks to UAV manager
        M.UAV_MANAGER.addInfoAddCallback(this._addUAVCallback.bind(this));
        M.UAV_MANAGER.addInfoChangeCallback(this._updateUAVCallback.bind(this));

        /**
         * Config file of the UAV Info.
         * @type {dict}
         * @access private
         */
         let _configFile = config.SideBars.UavInfo;

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
     * Callback to a new UAV added.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the UAV id.
     * @returns {void}
     * @access private
     */
    _addUAVCallback(myargs, args) {

        let uavId = args[0];

        let dict = M.UAV_MANAGER.getDictById(uavId);
        let list = Utils.generateList(this._infoTable, dict, this._roundTable);

        let uavInfoHtml = HTMLUtils.addDict('table', `${this._htmlId}-mission-${uavId}`, {}, this._header, list);
        let uavCollapseHtml = HTMLUtils.addDict('collapse', `${this._htmlId}-UAV-${uavId}`, {}, `Mission ${uavId}`, true, [uavInfoHtml]);
        HTMLUtils.addToExistingElement(`${this._htmlId}`, [uavCollapseHtml]);
    }

    /**
     * Callback to a UAV changed.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {array} args - List with the UAV id.
     * @returns {void}
     * @access private
     */
    _updateUAVCallback(myargs, args) {

        let uavId = args[0];

        let parent = document.getElementById(`${this._htmlId}-mission-${uavId}-table-body`);
        parent.innerHTML = '';

        let dict = M.UAV_MANAGER.getDictById(uavId);
        let list = Utils.generateList(this._infoTable, dict, this._roundTable);

        for (let i = 0; i < list.length; i++) {
            let trContent = HTMLUtils.addDict('tr', ``, {}, list[i]);
            HTMLUtils.addHTML(parent, trContent);
        }
    }
}
