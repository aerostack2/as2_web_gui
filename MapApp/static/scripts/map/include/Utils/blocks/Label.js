
class Label extends HTMLUtils 
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
        let label = document.createElement('label');
        label.innerHTML = content.text;
        return label;
    }
}