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