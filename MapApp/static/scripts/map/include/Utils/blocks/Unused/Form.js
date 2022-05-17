class Form extends HTMLUtils 
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
        for (let i = 0; i < content.list.length; i++) {
            super.addHTML(div, content.list[i]);
        }
        return div;
    }
}