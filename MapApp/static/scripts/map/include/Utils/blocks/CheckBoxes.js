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