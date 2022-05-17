import utm

def UTM_to_GPS(east, north, zone_number, zone_letter):
    """
    Transform from UTM coordinates to GPS coordinates.
    Args:
        east (double): Easting of the point in UTM coordinates.
        north (double): Northing of the point in UTM coordinates.
        zone_number (int): Zone number of the UTM coordinates.
        zone_letter (str): Zone letter of the UTM coordinates.
    Returns:
        double, double: Latitude and longitude of the point in GPS coordinates.
    """

    return utm.to_latlon(east, north, zone_number, zone_letter)


def GPS_to_UTM(lat, lon):
    """
    Transform from GPS coordinates to UTM coordinates.
    Args:
        lat (double): Latitude of the point in decimal degrees.
        lon (double): Longitude of the point in decimal degrees.
    Returns:
        double, double, int, string: Easting and Northing of the point in UTM coordinates, zone number and zone letter.
    """
    return utm.from_latlon(lat, lon)


def change_system_reference(coordinates_list, reference):
    """
    Change the coordinates to the new reference system.
    Args:
        coordinates_list (tuple): List of tuples with the coordinates with the form (x, y).
        reference (tuple): Tuple with the new reference system with the form (x, y).
    Returns:
        list: List of tuples with the coordinates in the new reference system.
    """

    # converted list
    converted_list = []

    # then cast your geographic coordinate pair to the projected system
    for coordinates in coordinates_list:
        x = coordinates[1] - reference[0]
        y = coordinates[2] - reference[1]
        converted_list.append([coordinates[0], x, y])

    return converted_list