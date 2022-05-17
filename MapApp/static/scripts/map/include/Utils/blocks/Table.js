class Table extends HTMLUtils {
    static addTypeDict(type, id = 'none', attributes = {}, ...args) {
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'header': args[0],
                'list': args[1],
            }
        );
    }

    static addTypeHTML(content) {

        let table = document.createElement('table');
        table.setAttribute('class', 'table d-grid gap-2 m');
        table.setAttribute('id', `${content['id']}-table`);

        let head = document.createElement('thead');
        head.setAttribute('id', `${content['id']}-table-head`);

        let numberOfColumns = content['header'].length;

        for (let i = 0; i < numberOfColumns; i++) {
            let th = document.createElement('th');
            th.innerHTML = content['header'][i];
            th.setAttribute('scope', 'col');
            head.appendChild(th);
        }

        table.appendChild(head);

        let tbody = document.createElement('tbody');
        tbody.setAttribute('id', `${content['id']}-table-body`);

        for (let i = 0; i < content.list.length; i++) {
            let trContent = super.addDict('tr', `${content['id']}-table-row-${content.list[i][0]}`, {}, content.list[i]);
            super.addHTML(tbody, trContent);
        }

        table.appendChild(tbody);

        return table;
    }
}


class Tr extends HTMLUtils {
    static addTypeDict(type, id = 'none', attributes = {}, ...args) {
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'list': args[0],
            }
        );
    }

    static addTr(tr, val) {
        let td = document.createElement('td');
        if (typeof val === 'object' && !Array.isArray(val)) {
            Tr.addTr(tr, val);
        }
        td.innerHTML = val;
        tr.appendChild(td);
        return td;
    }

    static addTd(array) {
        for (let i = 0; i < array.length; i++) {
            if (!Array.isArray(array[i])) {
                let td = document.createElement('td');
                td.innerHTML = array[i];
                array[i] = td;
            } else if (Array.isArray(array[i])) {
                array[i] = addTd(array[i]);
            }
        }
        return array;
    }
    

    static addTypeHTML(content) {
        let tr = document.createElement('tr');
        tr.setAttribute('id', content.id);

        let tdList = Tr.addTd(content.list);
        for (let i = 0; i < content.list.length; i++) {
            tr.appendChild(tdList[i]);
        }

        return tr;
    }
}