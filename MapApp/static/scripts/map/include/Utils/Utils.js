/**
 * Class that contains utility static methods.
 */
class Utils {

    static onlyUnique(value, index, self) {
        return self.indexOf(value) === index;
    }

    // #region HTML input callbacks

    /**
     * Dumb function to initialize callbacks
     * @param {Object} input - Functions inputs 
     */
     static _nullFunct(input) {
        ConsoleSideBar.addWarning("Dumb callback function");
        console.log("Dumb callback function");
    };

    /**
     * Call each function in the list with the arguments
     * @param {array} callbackList - list of pair with the function to be called and the arguments to be passed
     * @param {...any} args - Other arguments to be passed to the functions
     * @return {void}
     * @access public
     * @static
     */
    static callCallbacks(callbackList, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            callbackList[i][0](callbackList[i][1], args);
        }
    }

    /**
     * Call each function in the list if the param changed is the desired one.
     * If desired param is '*' this function will be also called.
     * @param {array} callbackList - List of functions to be called 
     * @param {string} param - Name of the param that changed
     * @param {any} value - Value of the param that changed
     * @param {...any} args - Other arguments to be passed to the functions
     * @return {void}
     * @access public
     * @static
     */
    static callCallbackByParam(callbackList, param, value, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            if (callbackList[i][0] == param || callbackList[i][0] == '*') {
                callbackList[i][1](param, value, args);
            }
        }
    }

    /**
     * Add a callback function to a toggle button
     * @param {string} button_id - Id of the button
     * @param {function} callback - Callback when press button. Args will be passed to callback.
     * @param {...any} args - Other arguments to be passed to the functions
     * @return {void}
     * @access public
     * @static
     */
    static addButtonCallback(button_id, callback = Utils._nullFunct, ...args) {
        // get button element
        const btn = document.getElementById(button_id);

        // add an event listener to the button
        if (btn != null) {
            btn.addEventListener('click', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // call the button callback
                callback(args);
            });
        } else {
            ConsoleSideBar.addWarning("Utils.addButtonCallback - button not found");
            console.log("Warning: Utils.addButtonCallback - button not found");
        }
    }

    /**
     * Add a callback function to a list of buttons by their class
     * @param {string} button_class - Class of the buttons 
     * @param {function} callback - Callback when press button. Event.target and args will be passed to callback.
     * @param  {...any} args - Arguments list pass to callback
     * @return {void}
     * @access public
     * @static
     */
    static addButtonsCallback(button_class, callback = Utils._nullFunct, ...args) {
        const btns = document.getElementsByClassName(button_class);

        // add an event listener to the button
        for (let i = 0; i < btns.length; i++) {
            btns[i].addEventListener('click', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // call the button callback
                callback(e.target, args);
            });
        }
    }

    /**
     * Add a listener for number inputs by a button
     * @param {string} button_id - Id of the button
     * @param {string} inputs_id - List of form inputs id
     * @param {string} names_id - Name of elements in the list
     * @param {function} callback - Callback when press button. Args and Inputs will be passed to callback.
     * @param {...any} args - Other arguments to be passed to the functions
     * @return {void}
     * @access public
     * @static
     */
    static addFormCallback(button_id, inputs_id, names_id, callback = Utils._nullFunct, ...args) {

        // get button element
        const btn = document.getElementById(button_id);

        // check if inputs and names has the same length
        if (inputs_id.length != names_id.length) {
            ConsoleSideBar.addError("Utils.addFormCallback - inputs and names must have the same length");
            throw "Input id and names id lengths are not equals";
        }

        if (btn != null) {
            // add a button listener
            btn.addEventListener('click', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // for each input elemenet, get the value
                let inputs = [];
                for (let i = 0; i < inputs_id.length; i++) {
                    let element = document.getElementById(inputs_id[i]);
                    let value = element.value;
                    if (element.type == 'number') {
                        value = parseFloat(value);
                    }
                    inputs[names_id[i]] = value
                }

                // call the callback
                callback(args, inputs);
            });
        } else {
            ConsoleSideBar.addWarning("Utils.addFormCallback - button not found");
            console.log("Warning: Utils.addFormCallback - button not found");
        }
    }

    // #endregion

    // #region Files management

    /**
     * Add a File input listener
     * @param {string} button_id - Id of the button that load the file
     * @param {function} callback - Callback when file is added. Args and event will be passed to callback.
     * @param  {...any} args - Arguments list pass to callback
     * @return {void}
     * @access public
     * @static
     */
    static addFileCallback(button_id, callback = Utils._nullFunct, ...args) {
        const btn = document.getElementById(button_id);

        if (btn != null) {
            btn.addEventListener('change', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // call the button callback
                callback(args, e);
            });
        } else {
            ConsoleSideBar.addWarning("Utils.addFileCallback - button not found");
            console.log("Warning: Utils.addFileCallback - button not found");
        }
    }

    /**
     * Read a file and return its content as text
     * @param {string} path - Path of the file
     * @return {string} - File content
     * @access public
     * @static
     */
    static readFileText(path) {
        let output = null;
        fetch(path)
            .then(response => response.text())
            .then(data => {
                output = data;
            });
        return output;
    }

    /**
     * Read a file and return its content as blob
     * @param {string} path - Path of the file
     * @return {blob} - Blob of the file
     * @access public
     * @static
     */
    static readFileImage(path) {
        let output = null;
        fetch(path)
            .then(response => response.blob())
            .then(data => {
                output = data;
            });
        return output;
    }

    /**
     * Read a file and return its content as JSON
     * @param {string} path - Path of the file
     * @return {JSON} - JSON object
     * @access public
     * @static
     */
    static readFileJson(path) {
        let output = null;
        fetch(path)
            .then(response => response.json())
            .then(data => {
                output = data;
            });
        return output;
    }

    /**
     * Read a local file and return its blob by the callback function
     * @param {string} path - Path of the file
     * @param {function} callback - Callback when file is loaded. Blob and args will be passed to callback.
     * @param  {...any} args - Arguments list pass to callback
     * @return {void}
     * @access public
     * @static
     */
     static getLocalFile(path, callback, ...args) {
        fetch(path)
            .then(response => response.blob())
            .then(data => {
                callback(data, args);
            });
    }

    /**
     * Read a local file and return its content as text by the callback function
     * @param {string} path - Path of the file
     * @param {function} callback - Callback when file is loaded. File content and args will be passed to callback.
     * @param  {...any} args - Arguments list pass to callback
     * @return {void}
     * @access public
     * @static
     */
    static loadLocalFile(path, callback, type = 'text', ...args) {
        fetch(path)
            .then(response => response.blob())
            .then(data => {
                Utils.loadFile(data, callback, type, args);
            });
    }

    /**
     * Load a file and return its content as type by the callback function
     * @param {string} path - Path of the file
     * @param {function} callback - Callback when file is loaded. File content and args will be passed to callback.
     * @param {string} type - Type of the file read. Can be 'text', 'json', 'arraybuffer' or 'binary
     * @param  {...any} args - Arguments list pass to callback
     * @return {void}
     * @access public
     * @static
     */
    static loadFile(file, callback, type = "text", ...args) {

        var fileReader = new FileReader();
        fileReader.onload = function (fileLoadedEvent) {
            var dataFromFileLoaded = fileLoadedEvent.target.result;
            switch (type) {
                case 'json':
                    callback(JSON.parse(dataFromFileLoaded), ...args);
                    break;
                default:
                    callback(dataFromFileLoaded, args);
                    break;
            }

        };

        switch (type) {
            case 'text':
                fileReader.readAsText(file, "UTF-8");
                break;
            case 'json':
                fileReader.readAsText(file, "UTF-8");
                break;
            case 'arraybuffer':
                fileReader.readAsArrayBuffer(file);
                break;
            case 'binary':
                fileReader.readAsBinaryString(file);
                break;
            default:
                throw `Error: Utils.loadFile - type ${type} not supported`;
        }
    }

    /**
     * Reset the load file input to empty
     * @param {string} input_id - Id of the file html input
     * @return {void}
     * @access public
     * @static
     */
    static resetLoadFileInput(input_id) {
        let inputId = input_id + '-input';
        document.getElementById(inputId).value = "";
    }

    /**
     * Download a file with the input JSON content
     * @param {string} filename - Name of the file
     * @param {JSON} json - Text, dict or JSON to be written in the file
     * @access public
     * @static
     */
    static download(filename, json) {
        var element = document.createElement('a');
        element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(JSON.stringify(json)));
        element.setAttribute('download', filename);

        element.style.display = 'none';
        document.body.appendChild(element);
        element.click();

        document.body.removeChild(element);
    }

    /**
     * Download a Image with the input Blob content
     * @param {string} filename - Name of the file
     * @param {href} imageHref - URL of the image
     * @return {void}
     * @access public
     * @static
     */
    static downloadImage(filename, imageHref) {
        var element = document.createElement('a');
        element.setAttribute('href', imageHref);
        element.setAttribute('download', filename);

        element.style.display = 'none';
        document.body.appendChild(element);

        element.click();

        document.body.removeChild(element);
    }

    // #endregion

    /**
     * Round a number to a given decimal
     * @param {number} value - Number to be rounded
     * @param {number} decimals - Number of decimal to be rounded
     * @returns {number} - Rounded number
     * @access public
     * @static
     */
    static round(value, decimals) {
        return Number(Math.round((value + 0.000001) + 'e' + decimals) + 'e-' + decimals); // Number.EPSILON
    }

    /**
     * Compute the distance between two points
     * @param {number} x1 - X coordinate of the first point 
     * @param {number} y1 - Y coordinate of the first point
     * @param {number} x2 - X coordinate of the second point
     * @param {number} y2 - Y coordinate of the second point
     * @returns {number} - Distance between the two points
     * @access public
     * @static
     */
    static distance(x1, y1, x2, y2) {

        // Euclidean distance
        let x = x2 - x1;
        let y = y2 - y1;
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Convert a radian angle in ENU to a degree angle in NEU.
     * @param {number} angle - Angle in radian
     * @access public
     * @static
     */
    static angleENU2NEU(angle) {
        angle = angle * 180 / Math.PI ;

        // reduce the angle  
        angle =  angle % 360; 

        // force it to be the positive remainder, so that 0 <= angle < 360  
        angle = - (angle + 360) % 360;  
        
        return angle;
    }

    /**
     * Create a deep copy of an object
     * @param {Object} object - Object to be copied
     * @returns {Object} - Copy of the object
     * @access public
     * @static
     */
    static deepCopy(object) {
        if (Array.isArray(object)) {
            return [...object];
        } else if (typeof object === 'object') {
            return Object.assign(Object.create(Object.getPrototypeOf(object)), object);
        }
    }

    /**
     * Create a deep copy of two dictionaries
     * @param {dict} d1 - First dictionary to be copied. Duplicate keys will be overwritten by the second dictionary
     * @param {dict} d2 - Second dictionary to be copied
     * @returns {dict} - Copy of the dictionaries
     * @access public
     * @static
     */
    static deepCopyMergeDict(d1, d2) {
        return Object.assign({}, d1, d2);
    }

    /**
     * Check if an object has members given by a list of keys. If yes, return True. If not, return False.
     * @param {dict} dict - Dictionary to be checked
     * @param {array} memberList - List of keys to be checked
     * @returns {boolean} - True if all keys are in the dictionary
     * @access public
     * @static
     */
     static hasMember(dict, memberList) {
        let currentDict = dict;
        for (let i = 0; i < memberList.length; i++) {
            if (currentDict == undefined) {
                return false;
            }

            if (currentDict[memberList[i]] == undefined) {
                return false;
            }
            currentDict = currentDict[memberList[i]];
        }
        return true;
    }


    /**
     * Check if an object has members given by a list of keys. If yes, return it value. If not, return undefined.
     * @param {dict} dict - Dictionary to be accessed
     * @param {array} memberList - List of keys to be accessed
     * @returns {Object} - Value of the last key in the list. If not found, return undefined
     * @access public
     * @static
     */
    static getMember(dict, memberList) {
        let value = undefined;
        let currentDict = dict;
        for (let i = 0; i < memberList.length; i++) {
            if (currentDict[memberList[i]] != undefined) {
                value = currentDict[memberList[i]];
                currentDict = currentDict[memberList[i]];
            } else {
                return undefined;
            }
        }
        return value;
    }

    /**
     * Generate a list by the given list of keys and the given dictionary
     * @param {array} infoTable - List of list for each row. Each element is a list of keys to be accessed
     * @param {dict} dict - Dictionary to be accessed
     * @param {array} roundTable - List of keys and decimals to be rounded
     * @returns {list} - List of values
     * @access public
     * @static
     */
    static generateList(infoTable, dict, roundTable = []) {
        let infoList = [];
        for (let i = 0; i < infoTable.length; i++) {
            let row = infoTable[i];
            let infoRow = [];
            for (let j = 0; j < row.length; j++) {
                let element = row[j];
                let infoElement = '';
                if (Array.isArray(element)) {

                    if (Utils.hasMember(dict, element)) {
                        infoElement = Utils.getMember(dict, element);
                    } else {
                        infoElement = 'N/A';
                    }

                    for (let k = 0; k < roundTable.length; k++) {
                        let roundRow = roundTable[k];
                        if (roundRow[0] == element) {
                            infoElement = Utils.round(infoElement, roundRow[1]);
                        }
                    }

                } else if (typeof element === 'string' || element instanceof String) {
                    infoElement = element;
                }
                infoRow.push(infoElement);
            }
            infoList.push(infoRow);
        }
        return infoList;
    }
}

/**
 * Class that own a list of id and a dictionary of values for each id
 */
class SmartList {
    /**
     * Create a new instance of the class SmartList
     */
    constructor() {
        /**
         * List of id
         * @type {array}
         * @access private
         */
        this._objectList = [];

        /**
         * Dictionary of values for each id (key: id, value: value)
         * @type {dict}
         * @access private
         */
        this._objectDict = {};
    }

    /**
     * Add an id and its value
     * @param {string} id - Id to be added
     * @param {dict} object - Value to be added
     * @returns {void}
     * @access public
     */
    addObject(id, object) {
        this._objectList.push(id);
        this._objectDict[id] = object;
    }

    /**
     * Change the value of an id
     * @param {string} id - Id to be changed
     * @param {dict} objectInfo - Value to be added
     * @returns {void}
     * @access public
     */
    updateObject(id, objectInfo) {
        this._objectDict[id] = Object.assign({}, this._objectDict[id], objectInfo);
    }

    /**
     * Return the list of id
     * @returns {array} - List of id
     * @access public
     */
    getList() {
        return this._objectList;
    }

    /**
     * Return the dictionary of values for each id
     * @returns {dict} - Dictionary of values for each id
     * @access public
     */
    getDict() {
        return this._objectDict;
    }

    /**
     * Get the value of an id
     * @param {string} id - Id to be added
     * @returns {void}
     * @access public
     */
    getDictById(id) {
        if (this.getList().indexOf(id) !== -1) {
            return this._objectDict[id];
        } else {
            return null;
        }
    }

    /**
     * Remove an id and its value
     * @param {string} id - Id to be removed
     * @returns {void}
     * @access public
     */
    removeObject(id) {
        let index = this._objectList.indexOf(id);
        if (index > -1) {
            this._objectList.splice(index, 1);
        }
        delete this._objectDict[id];
    }
}


/**
 * UAV and Mission Manager prototype, that manage income information from server and call desired callbacks.
 */
class SmartListCallbacks extends SmartList {

    /**
     * Create a new instance of the class ManagerPrototype.
     * @param {string} infoAdd - Name of the header of the info message that will be received from server when a parameter is add/modified.
     * @param {string} infoSet - Name of the header of the info message that will be received from server when the information is set/reset.
     * @param {string} infoGet - Name of the header of the request message that will be received from server when the information is requested.
     **/
    constructor() {

        super();

        /**
         * List of callbacks when a parameter is modified
         * @type {array}
         * @access private
         */
        this._infoChangeCallbacks = [];

        /**
         * List of callbacks when a object is added or removed
         * @type {array}
         * @access private
         */
        this._infoAddCallbacks = [];

        /**
         * List of callbacks when the a parameter is changed
         * @type {array}
         * @access private
         */
        this._paramChangeCallbacks = [];
    }

    // #region Public methods

    /**
     * Add a function callback when any parameter is changed.
     * @param {function} callback - Function to be called when the parameter is changed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoChangeCallback(callback, ...args) {
        this._infoChangeCallbacks.push([callback, args]);
    }

    /**
     * Add a function callback when new information is added.
     * @param {function} callback - Function to be called when the information is added.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoAddCallback(callback, ...args) {
        this._infoAddCallbacks.push([callback, args]);
    }

    /**
     * Add a function callback when the desired parameter is changed.
     * @param {string} param - Parameter to be watched.
     * @param {function} callback - Function to be called when the parameter is changed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoParamCallback(param, callback, ...args) {
        this._paramChangeCallbacks.push([param, callback, args]);
    }

    /**
     * Remove the information with the given id.
     * @param {any} id - Id of the information.
     * @return {void}
     * @access public 
     */
    removeById(id) {
        Utils.callCallbacks(this._infoAddCallbacks, id, 'remove');
        super.removeObject(id);
    }

    /**
     * Add an id and its value
     * @param {string} id - Id to be added
     * @param {dict} object - Value to be added
     * @returns {void}
     * @access public
     */
    addObject(id, objectInfo) {
        super.addObject(id, objectInfo);
        Utils.callCallbacks(this._infoAddCallbacks, id, 'add');

        for (let key in objectInfo) {
            Utils.callCallbackByParam(this._paramChangeCallbacks, key, objectInfo[key], id);
        }
    }

    /**
     * Change the value of an id
     * @param {string} id - Id to be changed
     * @param {dict} objectInfo - Value to be added
     * @param {boolean} override - If true, the value will be overrided. If false, the value will be added to the current value.
     * @returns {void}
     * @access public
     */
    updateObject(id, objectInfo, override = false) {

        let oldInfo = Object.assign({}, this.getDictById(id));
        let newInfo = Object.assign({}, objectInfo);

        if (override) {
            super.addObject(id, objectInfo);
            Utils.callCallbacks(this._infoChangeCallbacks, id);
        } else {
            super.updateObject(id, objectInfo);
            Utils.callCallbacks(this._infoChangeCallbacks, id);
        }



        for (let key in newInfo) {
            if (key in oldInfo) {
                if (oldInfo[key] !== newInfo[key]) {
                    Utils.callCallbackByParam(this._paramChangeCallbacks, key, newInfo[key], id);
                }
            } else {
                Utils.callCallbackByParam(this._paramChangeCallbacks, key, newInfo[key], id);
            }
        }
    }
    // #endregion
}
