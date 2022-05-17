/**
 * Class that manage Image Overlay Left Side Bar.
 */
class ImageOVerlay {
    /**
     * Creates a new ImageOVerlay instance.
     */
    constructor() {
        /**
         * Id of the div that contains the image overlay content.
         * @type {string}
         * @access private
         */
        this._htmlId = 'sideBar-left-imageOverlay-content';

        /**
         * Config file of the image overlay.
         * @type {dict}
         * @access private
         */
         let _configFile = config.SideBars.ImageOverlay;

        /**
         * Image URL to be added on the map.
         * @type {string} - Href of the image.
         * @access private
         */
        this._imgUrl = _configFile.url;

        /**
         * Default opacity of the image overlay.
         * @type {number}
         * @access private
         */
        this._opacity = _configFile.opacity;

        // Set image draggable icons
        /**
         * URL of the three icon that delimit the image to be draggeable.
         * @type {string}
         * @access private
         */
         let iconUrl = _configFile.icon;
        let imageOverlay = L.icon({ iconUrl: iconUrl, iconSize: [30, 30] });

        /**
         * List of points that define the image overlay. The first point is the top left corner, the second point is the top right corner, and the third point is the bottom left corner.
         * @type {list}
         * @access private
         */
        this._point_list = [];
        this._point_list.push(L.latLng(_configFile.topLeftCorner));
        this._point_list.push(L.latLng(_configFile.topRightCorner));
        this._point_list.push(L.latLng(_configFile.bottomLeftCorner));

        this.markerTL = L.marker(this._point_list[0], { draggable: true, pmIgnore: true, icon: imageOverlay })
        this.markerTR = L.marker(this._point_list[1], { draggable: true, pmIgnore: true, icon: imageOverlay })
        this.markerBL = L.marker(this._point_list[2], { draggable: true, pmIgnore: true, icon: imageOverlay })

        /**
         * Instance of the image overlay.
         * @type {L.ImageOverlay}
         * @access private
         */
        this._overlay = null;

        // Add html content
        this._addHTML();

        // Add callbacks
        this._addCallbacks();
    }

    /**
     * Add HTML content to the image overlay side bar.
     * @returns {void}
     * @access private
     */
    _addHTML() {
        let htmlList = [];

        // File input
        let fileInput = HTMLUtils.addDict('input', `${this._htmlId}-imageFilenput`, { 'class': 'form-control', 'required': 'required' }, 'url', `Image URL`);
        let fileBtn = HTMLUtils.addDict('button', `${this._htmlId}-imageFileBtn`, { 'class': 'btn btn-primary' }, 'Add URL');
        let fileRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [fileInput, fileBtn], { 'class': 'col-md-6' });

        // Top left corner input
        let topLeftLat = HTMLUtils.addDict('input', `${this._htmlId}-topLeftLat`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[0].lat }, 'text', 'Latitude');
        let topLeftLng = HTMLUtils.addDict('input', `${this._htmlId}-topLeftLng`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[0].lng }, 'text', 'Latitude');
        let row1 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [topLeftLat, topLeftLng], { 'class': 'col-6' });

        // Top right corner input
        let toprightLat = HTMLUtils.addDict('input', `${this._htmlId}-toprightLat`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[1].lat }, 'text', 'Latitude');
        let toprightLng = HTMLUtils.addDict('input', `${this._htmlId}-toprightLng`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[1].lng }, 'text', 'Latitude');
        let row2 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [toprightLat, toprightLng], { 'class': 'col-6' });

        // Bottom left corner input
        let bottomLeftLat = HTMLUtils.addDict('input', `${this._htmlId}-bottomLeftLat`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[2].lat }, 'text', 'Latitude');
        let bottomLeftLng = HTMLUtils.addDict('input', `${this._htmlId}-bottomLeftLng`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[2].lng }, 'text', 'Latitude');
        let row3 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [bottomLeftLat, bottomLeftLng], { 'class': 'col-6' });

        // Add and remove overlay buttons
        let addBtn = HTMLUtils.addDict('button', `${this._htmlId}-addOverlayBtn`, { 'class': 'btn btn-primary' }, 'Add Image');
        let removeBtn = HTMLUtils.addDict('button', `${this._htmlId}-removeOverlayBtn`, { 'class': 'btn btn-danger' }, 'Remove Image');
        let addBtnDiv = HTMLUtils.addDict('div', `none`, {}, [addBtn]);
        let removeBtniv = HTMLUtils.addDict('div', `none`, {}, [removeBtn]);
        let addRemoveBtn = HTMLUtils.addDict('div', `none`, { 'class': 'btn-group d-flex justify-content-evenly', 'role': 'group' }, [addBtnDiv, removeBtniv]);

        // Image overlay opacity input
        let opacityInput = HTMLUtils.addDict('input', `${this._htmlId}-opacityInput`, { 'class': 'form-control', 'required': 'required',  'value': this._opacity }, 'text', `Opacity`);
        let opacityBtn = HTMLUtils.addDict('button', `${this._htmlId}-opacityBtn`, { 'class': 'btn btn-primary' }, 'Set opacity (m)');
        let opacityRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [opacityInput, opacityBtn], { 'class': 'col-md-6' });

        // Add each element to html list
        htmlList.push(fileRow);
        htmlList.push(row1);
        htmlList.push(row2);
        htmlList.push(row3);
        htmlList.push(addRemoveBtn);
        htmlList.push(opacityRow);

        // Add the html list to the side bar content
        HTMLUtils.addToExistingElement(`${this._htmlId}`, htmlList);
    }

    /**
     * Add callbacks to inputs.
     * @returns {void}
     * @access private
     */
    _addCallbacks() {

        Utils.addFormCallback(`${this._htmlId}-imageFileBtn`, [`${this._htmlId}-imageFilenput`], ['imgURL'], this._addImageUrlCallback.bind(this));
        Utils.addFormCallback(`${this._htmlId}-opacityBtn`, [`${this._htmlId}-opacityInput`], ['opacity'], this._setOpacity.bind(this));
        Utils.addButtonCallback(`${this._htmlId}-removeOverlayBtn`, this._removeCallback.bind(this));
        Utils.addFormCallback(
            `${this._htmlId}-addOverlayBtn`,
            [
                `${this._htmlId}-topLeftLat`,    `${this._htmlId}-topLeftLng`,
                `${this._htmlId}-toprightLat`,   `${this._htmlId}-toprightLng`,
                `${this._htmlId}-bottomLeftLat`, `${this._htmlId}-bottomLeftLng`
            ],
            [
                `topLeftLat`,    `topLeftLng`,
                `toprightLat`,   `toprightLng`,
                `bottomLeftLat`, `bottomLeftLng`
            ],
            this._addImageOverlayCallback.bind(this)
        );        
    }

    /**
     * Callback to image URL input.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {dict} input - Dict with key 'imgURL' and the url value.
     * @returns {void}
     * @access private
     */
    _addImageUrlCallback(myargs, inputs) {
        this._imgUrl = inputs.imgURL;
        ConsoleSideBar.addMessage(`Added image url: ${this._imgUrl}`);
    }

    /**
     * Callback to the add image button.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {dict} input - Dict with keys 'topLeftLat' and 'topLeftLng', 'toprightLat' and 'toprightLng', and 'bottomLeftLat' and 'bottomLeftLng', with the image cornet coordinates.
     * @returns {void}
     * @access private
     */
    _addImageOverlayCallback(myargs, inputs) {

        if (this._imgUrl == null) {
            alert("Please select an image");
            ConsoleSideBar.addMessage("Please select an image");
            return;
        }

        this.markerTL.addTo(M.MAP);
        this.markerTR.addTo(M.MAP);
        this.markerBL.addTo(M.MAP);

        this.markerTL.setLatLng(L.latLng(inputs['topLeftLat'], inputs['topLeftLng']));
        this.markerTR.setLatLng(L.latLng(inputs['toprightLat'], inputs['toprightLng']));
        this.markerBL.setLatLng(L.latLng(inputs['bottomLeftLat'], inputs['bottomLeftLng']));

        if (this._overlay == null) {
            this._overlay = L.imageOverlay.rotated(this._imgUrl, this._point_list[0], this._point_list[1], this._point_list[2], {
                opacity: this._opacity,
                interactive: true,
            });
        } else {
            this._overlay.setUrl(this._imgUrl);
            this._overlay.setBounds(this._point_list);
            this._overlay._setOpacity(this._opacity);
        }

        this.markerTL.on('drag dragend', this._repositionImage.bind(this));
        this.markerTR.on('drag dragend', this._repositionImage.bind(this));
        this.markerBL.on('drag dragend', this._repositionImage.bind(this));

        // Add the image overlay to the map
        M.MAP.addLayer(this._overlay, { pmIgnore: true });
    }

    /**
     * Remove button callback that remove the image overlay from the map.
     * @returns {void}
     * @access private
     */
    _removeCallback() {
        if (this._overlay != null) {
            M.MAP.removeLayer(this._overlay);
            this.markerTL.remove();
            this.markerTR.remove();
            this.markerBL.remove();
            this._overlay = null;
        }
    }

    /**
     * Move the image overlay when the markers are moved.
     * @returns {void}
     * @access private
     */
    _repositionImage() {
        this._overlay.reposition(
            this.markerTL.getLatLng(), 
            this.markerTR.getLatLng(), 
            this.markerBL.getLatLng()
        );
    }

    /**
     * callback to the change the image overlay opacity input.
     * @param {array} myargs - List of arguments passed to the callback.
     * @param {dict} inputs - Dictionary with key 'opacity' and the opacity value.
     * @returns {void}
     * @access private
     */
    _setOpacity(myargs, inputs) {

        this._opacity = inputs['opacity'];
        if (this._overlay != null) {
            this._overlay._setOpacity(this._opacity);
        }
    }

}