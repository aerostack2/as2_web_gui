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

class CheckBox extends HTMLUtils {
    static addTypeDict(type, id = 'none', attributes = {}, ...args) {
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'type': args[0],
                'text': args[1],
            }
        );
    }

    static addTypeHTML(content) {

        let id = content.id;

        let div = document.createElement('div');
        let input = document.createElement('input');
        let label = document.createElement('label');

        input.setAttribute('type', content['type']);

        if (content['type'] == 'radio') {
            let name = id .replace('-' + content.text,'');
            input.setAttribute('name', name);
        }

        input.setAttribute('class', 'form-check-input');
        input.setAttribute('id', `${id}-Input`);

        label.setAttribute('class', 'badge bg-primary text-wrap fs-6');
        label.setAttribute('id', `${id}-Label`);
        label.setAttribute('for', `${id}-Input`);
        label.setAttribute('style', 'display:inline-block; width:90%');

        label.innerHTML = content.text;

        div.setAttribute('class', 'form-check');
        div.appendChild(input);
        div.appendChild(label);
        return div;
    }
}

class CheckBoxes extends HTMLUtils {
    static addTypeDict(type, id = 'none', attributes = {}, ...args) {
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'type': args[0],
                'list': args[1],
            }
        );
    }

    static addTypeHTML(content) {

        let divGlobal = document.createElement('div');

        for (let i = 0; i < content.list.length; i++) {
            super.addHTML(divGlobal, HTMLUtils.addDict('checkBox', `${content.id}-${content.list[i]}`, {}, content.type, content.list[i]));
        }

        return divGlobal;
    }
}