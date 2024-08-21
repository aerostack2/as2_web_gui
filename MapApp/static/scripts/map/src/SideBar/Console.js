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