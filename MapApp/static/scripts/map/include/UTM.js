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

// class LocalCoordinates {
//     constructor(mapCenter) {
//         console.log('LocalCoordinates constructor');
//         this.utm = new UTMLatLng();
//         this.precision = 3;
//         this.setOrigin(mapCenter);
//     }

//     setOrigin(mapCenter) {

//         this.originMarker = L.marker(mapCenter, {}).addTo(M.MAP);
//         let origin = this.originMarker._latlng;

//         this.LatLonOrigin = origin;
//         this.latOrigin = parseFloat(origin.lat);
//         this.lngOrigin = parseFloat(origin.lng);

//         this.UTMOrigin = this.utm.convertLatLngToUtm(this.latOrigin, this.lngOrigin, this.precision);
//         this.xOrigin = parseFloat(this.UTMOrigin.Easting);
//         this.yOrigin = parseFloat(this.UTMOrigin.Northing);
//         this.zoneNumOrigin = this.UTMOrigin.ZoneNumer;
//         this.zoneLetterOrigin = this.UTMOrigin.ZoneLetter;
//     }

//     getLatLon(easting, northing, zoneNum, zoneLetter) {
//         return this.utm.convertUtmToLatLng(easting, northing, zoneNum, zoneLetter);
//     }

//     getUTM(latitude, longitude) {
//         return this.utm.convertLatLngToUtm(latitude, longitude, this.precision);
//     }

//     localUTM2LatLong(xLocal, yLocal) {
//         let x = xLocal + this.xOrigin;
//         let y = yLocal + this.yOrigin;
        
//         return this.getLatLon(x, y, this.zoneNumOrigi, this.zoneLetterOrigin);
//     }

//     latLong2LocalUTM(lat, lng) {
//         let globalUTM = this.getUTM(lat, lng);

//         let x = parseFloat(globalUTM.Easting) - this.xOrigin;
//         let y = parseFloat(globalUTM.Northing) - this.yOrigin;

//         return {
//             x: x,
//             y: y
//         };
//     }
// }


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