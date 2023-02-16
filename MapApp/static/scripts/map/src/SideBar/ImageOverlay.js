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

        if (M.USE_LOCAL_COORDINATES) {
            console.log("Converting to local coordinates")
            console.log(this._point_list)
            this._point_list = M.UTM.getLatLngs(this._point_list);
        }

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
        let topLeftX = HTMLUtils.addDict('input', `${this._htmlId}-topLeftX`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[0].lat }, 'text', M.X_NAME);
        let topLeftY = HTMLUtils.addDict('input', `${this._htmlId}-topLeftY`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[0].lng }, 'text', M.Y_NAME);
        let row1 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [topLeftX, topLeftY], { 'class': 'col-6' });

        // Top right corner input
        let toprightX = HTMLUtils.addDict('input', `${this._htmlId}-toprightX`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[1].lat }, 'text', M.X_NAME);
        let toprightY = HTMLUtils.addDict('input', `${this._htmlId}-toprightY`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[1].lng }, 'text', M.Y_NAME);
        let row2 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [toprightX, toprightY], { 'class': 'col-6' });

        // Bottom left corner input
        let bottomLeftX = HTMLUtils.addDict('input', `${this._htmlId}-bottomLeftX`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[2].lat }, 'text', M.X_NAME);
        let bottomLefY = HTMLUtils.addDict('input', `${this._htmlId}-bottomLeftY`, { 'class': 'form-control', 'required': 'required', 'value': this._point_list[2].lng }, 'text', M.Y_NAME);
        let row3 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [bottomLeftX, bottomLefY], { 'class': 'col-6' });

        // Add and remove overlay buttons
        let addBtn = HTMLUtils.addDict('button', `${this._htmlId}-addOverlayBtn`, { 'class': 'btn btn-primary' }, 'Add Image');
        let removeBtn = HTMLUtils.addDict('button', `${this._htmlId}-removeOverlayBtn`, { 'class': 'btn btn-danger' }, 'Remove Image');
        let addBtnDiv = HTMLUtils.addDict('div', `none`, {}, [addBtn]);
        let removeBtniv = HTMLUtils.addDict('div', `none`, {}, [removeBtn]);
        let addRemoveBtn = HTMLUtils.addDict('div', `none`, { 'class': 'btn-group d-flex justify-content-evenly', 'role': 'group' }, [addBtnDiv, removeBtniv]);

        // Image overlay opacity input
        let opacityInput = HTMLUtils.addDict('input', `${this._htmlId}-opacityInput`, { 'class': 'form-control', 'required': 'required', 'value': this._opacity }, 'text', `Opacity`);
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
                `${this._htmlId}-topLeftX`, `${this._htmlId}-topLeftY`,
                `${this._htmlId}-toprightX`, `${this._htmlId}-toprightY`,
                `${this._htmlId}-bottomLeftX`, `${this._htmlId}-bottomLeftY`
            ],
            [
                `topLeft${M.X_NAME}`, `topLeft${M.Y_NAME}`,
                `topright${M.X_NAME}`, `topright${M.Y_NAME}`,
                `bottomLeft${M.X_NAME}`, `bottomLeft${M.Y_NAME}`
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

        let markerTL_coords = [inputs[`topLeft${M.X_NAME}`], inputs[`topLeft${M.Y_NAME}`]];
        let markerTR_coords = [inputs[`topright${M.X_NAME}`], inputs[`topright${M.Y_NAME}`]];
        let markerBL_coords = [inputs[`bottomLeft${M.X_NAME}`], inputs[`bottomLeft${M.Y_NAME}`]];

        if (M.USE_LOCAL_COORDINATES) {
            markerTL_coords = M.UTM.getLatLng(markerTL_coords);
            markerTR_coords = M.UTM.getLatLng(markerTR_coords);
            markerBL_coords = M.UTM.getLatLng(markerBL_coords);
        }

        this.markerTL.setLatLng(L.latLng(inputs[`topLeft${M.X_NAME}`], inputs[`topLeft${M.Y_NAME}`]));
        this.markerTR.setLatLng(L.latLng(inputs[`topright${M.X_NAME}`], inputs[`topright${M.Y_NAME}`]));
        this.markerBL.setLatLng(L.latLng(inputs[`bottomLeft${M.X_NAME}`], inputs[`bottomLeft${M.Y_NAME}`]));

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