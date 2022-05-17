/**
 * Id of each message in the console.
 * @type {number}
 * @access private
 */
let _cont = 0;

/**
 * Class that manage Console Left Side Bar, adding messages, warnings and errors to the side bar.
 */
class ConsoleSideBar {
    /**
     * Add a message to the console.
     * @param {string} message - Message to be added.
     * @returns {void}
     * @access public
     * @static 
     */
    static addMessage(message) {
        let time = new Date();

        let htmlId = 'sideBar-left-console-content';
        let console = document.getElementById(htmlId);
        let newMessage = document.createElement('p');
        newMessage.innerHTML = _cont + ' - ' + time.toLocaleTimeString() + ': ' + message;
        newMessage.style.color = 'white';
        newMessage.setAttribute('class', 'my-1');
        console.appendChild(newMessage);
        _cont++;
    }

    /**
     * Add a warning message to the console.
     * @param {string} message - Message to be added.
     * @returns {void}
     * @access public
     * @static 
     */
    static addWarning(message) {
        let time = new Date();
        let htmlId = 'sideBar-left-console-content';
        let console = document.getElementById(htmlId);
        let newMessage = document.createElement('p');
        newMessage.innerHTML = _cont + ' - ' + time.toLocaleTimeString() + ': ' + message;
        newMessage.style.color = 'yellow';
        newMessage.setAttribute('class', 'my-1');
        console.appendChild(newMessage);
        _cont++;
    }

    /**
     * Add an error message to the console.
     * @param {string} message - Message to be added.
     * @returns {void}
     * @access public
     * @static 
     */
    static addError(message) {
        let time = new Date();
        let htmlId = 'sideBar-left-console-content';
        let console = document.getElementById(htmlId);
        let newMessage = document.createElement('p');
        newMessage.innerHTML = _cont + ' - ' + time.toLocaleTimeString() + ': ' + message;
        newMessage.style.color = 'red';
        newMessage.setAttribute('class', 'my-1');
        console.appendChild(newMessage);
        _cont++;
    }
}