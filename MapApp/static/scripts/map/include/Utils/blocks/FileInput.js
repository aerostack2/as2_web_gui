class FileInput extends HTMLUtils 
{
    static addTypeDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'text': args[0],
                'path': args[1],
            }
        );
    }   

    static addTypeHTML(content) {

        let div = document.createElement('div');
        div.setAttribute('class', 'mb-3');

        let label = document.createElement('LABEL');
        label.setAttribute('for', content.id + '-input');
        label.setAttribute('id', content.id + '-label');
        label.innerHTML = content.text;

        let input = document.createElement('INPUT');
        input.setAttribute('type', 'file');
        input.setAttribute('id', content.id + '-input');
        input.setAttribute('class', 'form-control form-control-sm');
        input.setAttribute('accept', content.path);

        div.appendChild(label);
        div.appendChild(input);

        return div;
    }
}