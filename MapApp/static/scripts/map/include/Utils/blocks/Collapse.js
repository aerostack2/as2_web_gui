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

class Collapse extends HTMLUtils 
{
    static addTypeDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'text': args[0],
                'show': args[1],
                'list': args[2],
            }
        );
    }  

    static addTypeHTML(content) {

        let div = document.createElement('div');
        let btn = document.createElement('button');
        let div_collapse = document.createElement('div');

        btn.setAttribute('class', 'btn btn-secondary btn-collap-toggle');
        btn.setAttribute('type', 'button');
        btn.setAttribute('data-bs-toggle', 'collapse');
        btn.setAttribute('data-bs-target', '#' + content.id + '-collapsable');
        btn.innerHTML = `${content.text} <i class="icon-collap-toggle fas fa-plus"></i>`;

        if (content.show) {
            div_collapse.setAttribute('class', 'collapse multi-collapse show');
        } else {
            div_collapse.setAttribute('class', 'collapse multi-collapse');
        }
        div_collapse.setAttribute('id', content.id + '-collapsable');
        
        for (let i=0; i<content.list.length; i++) {
            super.addHTML(div_collapse, content.list[i]);
        }

        div.setAttribute('class', 'd-grid gap-2 m-2');
        div.appendChild(btn);
        div.appendChild(div_collapse);

        return div;
    }
}