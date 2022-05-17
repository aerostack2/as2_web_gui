class Li extends HTMLUtils 
{
    static addTypeDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'element': args[0],
            }
        );
    } 

    static addTypeHTML(content) {
        let label = document.createElement('LI');
        super.addHTML(label, content.element);
        return label;
    }
}