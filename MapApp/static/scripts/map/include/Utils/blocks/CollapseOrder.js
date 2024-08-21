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

class CollapseOrder extends HTMLUtils 
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
        div.setAttribute('class', 'd-grid gap-2 m-2');

        let divTittle = document.createElement('div');
        divTittle.setAttribute('class', 'row justify-content-md-center align-middle');

        let btnName = document.createElement('button');
        btnName.setAttribute('id', content.id + '-btn');
        btnName.setAttribute('class', 'btn btn-secondary btn-collap-toggle col-md-9');
        btnName.setAttribute('type', 'button');
        btnName.setAttribute('data-bs-toggle', 'collapse');
        btnName.setAttribute('data-bs-target', '#' + content.id + '-collapsable');
        btnName.innerHTML = `${content.text} <i class="icon-collap-toggle fas fa-plus"></i>`;

        let btnUp = document.createElement('button');
        btnUp.setAttribute('class', 'btn btn-secondary col-md-1');
        btnUp.setAttribute('id', content.id + '-up');
        btnUp.innerHTML = `<i class="fa-solid fa-arrow-up"></i>`;

        let btnDown = document.createElement('button');
        btnDown.setAttribute('class', 'btn btn-secondary col-md-1');
        btnDown.setAttribute('id', content.id + '-down');
        btnDown.innerHTML = `<i class="fa-solid fa-arrow-down"></i>`;

        divTittle.appendChild(btnName);
        divTittle.appendChild(btnUp);
        divTittle.appendChild(btnDown);


        let divCollapse = document.createElement('div');
        if (content.show) {
            divCollapse.setAttribute('class', 'collapse multi-collapse show');
        } else {
            divCollapse.setAttribute('class', 'collapse multi-collapse');
        }
        divCollapse.setAttribute('id', content.id + '-collapsable');
        
        for (let i=0; i<content.list.length; i++) {
            super.addHTML(divCollapse, content.list[i]);
        }

        div.appendChild(divTittle);
        div.appendChild(divCollapse);

        return div;
    }
}