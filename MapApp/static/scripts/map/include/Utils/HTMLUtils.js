// List class of [type, fileName]
var blocksClassList = []

class HTMLUtils {

    static setDict(type, id = 'none', attributes = {}, content) {
        return {
            'type': type,
            'id': id,
            'attributes': attributes,
            'content': content,
        }
    }

    static addTypeDict(type, id = 'none', attributes = {}, ...args) {
        throw new Error('Not implemented');
    }

    static addTypeHTML(content) {
        throw new Error('Not implemented');
    }

    static _addAttribute(element, id, attributes) {
        if (id != 'none') {
            element.setAttribute('id', id);
        }
        for (let key in attributes) {
            if (element.getAttribute(key) == null) {
                element.setAttribute(key, attributes[key]);
            } else {
                element.setAttribute(key, element.getAttribute(key) + ' ' + attributes[key]);
            }
        }
        return element;
    }

    static addDict(type, id = 'none', attributes = {}, ...args) {
        for (let i = 0; i < blocksClassList.length; i++) {
            if (blocksClassList[i][0] == type) {
                return blocksClassList[i][1].addTypeDict(type, id, attributes, ...args);
            }
        }
    }

    static addHTML(parent, childDict) {
        let child = null;
        let flag = true;

        if (childDict == undefined) {
            return;
        }

        if (Object.keys(childDict).length === 0) {
            return;
        }

        if (Array.isArray(childDict)) {
            for (let j = 0; j < childDict.length; j++) {
                HTMLUtils.addHTML(parent, childDict[j])
            }
            return;
        }

        for (let i = 0; i < blocksClassList.length; i++) {
            if (blocksClassList[i][0] == childDict.type) {
                child = blocksClassList[i][1].addTypeHTML(childDict.content);
                HTMLUtils._addAttribute(child, childDict.id, childDict.attributes);
                parent.appendChild(child);
                flag = false;
                break; // If types are unique, break after first match to optimize performance
            }
        }

        if (flag) {
            console.log("Unknown type of HTML block");
            console.log(childDict)
            console.log(typeof childDict)
            console.log(Array.isArray(childDict))
            throw new Error('Unknown type of HTML block');
        }
    }

    static dictToHTML(dict) {
        let child = null;

        for (let i = 0; i < blocksClassList.length; i++) {
            if (blocksClassList[i][0] == dict.type) {
                child = blocksClassList[i][1].addTypeHTML(dict.content);
                HTMLUtils._addAttribute(child, dict.id, dict.attributes);
                return child;
            }
        }

        console.log("Unknown type of HTML block");
        console.log(dict)
        throw new Error('Unknown type of HTML block');
    }

    static addToExistingElement(id, blockList) {
        let parent = document.getElementById(id);

        for (let i = 0; i < blockList.length; i++) {
            HTMLUtils.addHTML(parent, blockList[i]);
        }
    }

    static initDropDown(id, list, defaultValue) {
        let dropDownBtn = HTMLUtils.addDict('dropDownBtn', `${id}-DropDown-Btn`, { 'class': 'btn btn-info' }, defaultValue);

        let dropDownExpandList = [];
        for (let i = 0; i < list.length; i++) {
            dropDownExpandList.push(HTMLUtils.addDict('button', `${id}-DropDown-Expand-${list[i]}`, { 'class': `btn btn-secondary ${id}-item w-75`, 'style': "background: #dae8fc" }, list[i]));
        }

        let dropDownExpand = HTMLUtils.addDict('dropDownExpand', `${id}-DropDown-Expand`, {}, dropDownExpandList);
        return HTMLUtils.addDict('dropDown', `${id}-DropDown`, { 'class': 'row m-1 gap-2' }, dropDownBtn, dropDownExpand);
    }

    static updateDropDown(id, list) {
        let expand = document.getElementById(`${id}-DropDown-menu`);
        expand.innerHTML = '';

        let dropDownExpandList = [];
        for (let i = 0; i < list.length; i++) {
            dropDownExpandList.push(HTMLUtils.addDict('button', `${id}-DropDown-Expand-${list[i]}`, { 'class': `btn btn-secondary ${id}-item w-75`, 'style': "background: #dae8fc" }, list[i]));
        }

        let dropDownExpand = HTMLUtils.addDict('dropDownExpand', `${id}-DropDown-Expand`, {}, dropDownExpandList);
        HTMLUtils.addToExistingElement(`${id}-DropDown-menu`, [dropDownExpand]);
    }

    static updateCheckBoxes(idCheckBoxes, type, list, attributes = {}) {
        console.log("updateCheckBoxes")
        console.log(list)
        let checkBoxes = document.getElementById(`${idCheckBoxes}`);
        checkBoxes.innerHTML = '';
        for (let i = 0; i < list.length; i++) {
            let checkBox = HTMLUtils.addDict('checkBox', `${idCheckBoxes}-${list[i]}`, attributes, type, list[i]);
            HTMLUtils.addToExistingElement(`${idCheckBoxes}`, [checkBox]);
        }
    }

    static addCheckBox(idParent, idContent, type, name, attributes = {}) {
        let parentElement = document.getElementById(idParent);
        HTMLUtils.addHTML(parentElement, HTMLUtils.addDict('checkBox', `${idContent}-${name}`, attributes, type, name));
    }
}
