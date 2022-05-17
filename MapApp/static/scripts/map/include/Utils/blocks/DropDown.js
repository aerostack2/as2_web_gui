class DropDown extends HTMLUtils 
{
    static addTypeDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'btn': args[0],
                'expand': args[1],
            }
        );
    }  

    static addTypeHTML(content) {

        let div = document.createElement('div');
        div.setAttribute('class', 'row');
        
        super.addHTML(div, content.btn);

        let UL = document.createElement('ul');
        UL.setAttribute('class', 'dropdown-menu');
        UL.setAttribute('id', content.id + '-menu');

        super.addHTML(UL, content.expand);
        div.appendChild(UL);
        
        return div;
    }
}

class DropDownBtn extends HTMLUtils 
{
    static addTypeDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'text': args[0],
            }
        );
    }  

    static addTypeHTML(content) {


        let button = document.createElement('button');
        button.setAttribute('type', 'button');
        button.innerHTML = content.text;
        // <i class="fa-solid fa-square-caret-down"></i>

        button.setAttribute('class', 'dropdown-toggle');
        button.setAttribute('data-bs-toggle', 'dropdown');
        button.setAttribute('aria-expanded', 'false');

        return button;
    }
}

class DropDownExpand extends HTMLUtils 
{
    static addTypeDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'list': args[0],
            }
        );
    }  

    static addTypeHTML(content) {

        let div = document.createElement('div');
        div.setAttribute('class', 'row');
        
        for (let i = 0; i < content.list.length; i++) {
            let Li = document.createElement('li');
            content.list[i].attributes['class'] += 'dropdown-item';
            super.addHTML(Li, content.list[i]);
            div.appendChild(Li);
        }

        return div;
    }
}