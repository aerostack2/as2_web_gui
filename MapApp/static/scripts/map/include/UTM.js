/*!
 * @license BSD-3-Clause
 * 
 * Copyright (c) 2024 Universidad Politécnica de Madrid
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the [Nombre del proyecto] nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Rafael Pérez Seguí
 */

/**
 * It's has three types of coordinates:
 * 1. Global latitude and longitude:
 *   latlngs = {
 *    lat: latitude,
 *    lng: longitude
 *   }
 * 2. Global UTM:
 *  utm = {
 *   easting: easting,
 *   northing: northing,
 *   zoneNum: zoneNum,
 *   zoneLetter: zoneLetter
 *  }
 * 3. Local UTM:
 * utmLocal = {
 *  x: easting relative to the origin,
 *  y: northings relative to the origin,
 *  zoneNum: zoneNum,
 *  zoneLetter: zoneLetter
 * }
 */

class LocalCoordinates {
    constructor(mapCenter) {
        console.log('LocalCoordinates constructor');
        this.utm = new UTMLatLng();
        this.precision = 3;
        this.setOrigin(mapCenter);
    }

    setOrigin(mapCenter) {

        this.originMarker = L.marker(mapCenter, {}).addTo(M.MAP);

        this.originLatLng = this.originMarker._latlng;
        this.originLat = parseFloat(this.originLatLng.lat);
        this.originLng = parseFloat(this.originLatLng.lng);

        this.originUTM = this.originMarker._latlng.utm();
        this.originEasting = parseFloat(this.originUTM.x);
        this.originNorthing = parseFloat(this.originUTM.y);
        this.originZone = this.originUTM.zone;
        this.originBand = this.originUTM.band;
    }

    getLocalUTMCoordinates(latLng) {
        let utm = latLng.utm();
        let x = parseFloat(utm.x) - this.originEasting;
        let y = parseFloat(utm.y) - this.originNorthing;
        let zone = utm.zone;
        let band = utm.band;

        return {
            x: x,
            y: y,
            zone: zone,
            band: band
        };
    }
    
    getLocalUTM(latLng) {
        if (Array.isArray(latLng)) {
            latLng = L.latLng(latLng[0], latLng[1]);
        }
        let utm = latLng.utm();
        let x = parseFloat(utm.x) - this.originEasting;
        let y = parseFloat(utm.y) - this.originNorthing;
        return [x, y]
    }

    getLatLng(local_coordinates, zone = this.originZone, band = this.originBand) {
        let x = parseFloat(local_coordinates[0]) + this.originEasting;
        let y = parseFloat(local_coordinates[1]) + this.originNorthing;

        let utm = L.utm({x: x, y: y, zone: zone, band: band});
        return utm.latLng();
    }

    getLocalUTMs(latLngs) {
        
        let localList = [];
        for (let i = 0; i < latLngs.length; i++) {
            localList.push(this.getLocalUTM(latLngs[i]));
        }
        return localList;
    }

    getLatLngs(utmList, zone = this.originZone, band = this.originBand) {
        let latLngList = [];
        for (let i = 0; i < utmList.length; i++) {
            let utm = utmList[i];
            if (Utils.hasMember(utm, 'lat') && Utils.hasMember(utm, 'lng')) {
                utm = [];
                utm.push(utmList[i].lat);
                utm.push(utmList[i].lng);
            }
            let x = utm[0] + this.originEasting;
            let y = utm[1] + this.originNorthing;
            let latLng = L.utm({x: x, y: y, zone: zone, band: band});
            latLngList.push(latLng.latLng());
        }
        return latLngList;
    }
}