class SplitDivs extends HTMLUtils 
{
    static addTypeDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'list': args[0],
                'attributes': args[1],
            }
        );
    }   

    static addTypeHTML(content) {
        let div = document.createElement('div');
        for (let i = 0; i < content.list.length; i++) {
            let subDiv = document.createElement('div');
            for (let key in content.attributes) {
                subDiv.setAttribute(key, content.attributes[key]);
            }
            super.addHTML(subDiv, content.list[i]);
            div.appendChild(subDiv);
        }
        return div;
    }
}