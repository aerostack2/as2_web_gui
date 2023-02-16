from pymap3d import geodetic2enu, enu2geodetic

def test(gps_origin, enu_position):
    gps_position = enu2geodetic(
        gps_origin[0],
        gps_origin[1],
        0.0,
        enu_position[0],
        enu_position[1],
        0.0)

    print("GPS origin: ")
    print(gps_origin)

    print("GPS position: ")
    print(gps_position)

    enu_position2 = geodetic2enu(
        gps_origin[0],
        gps_origin[1],
        0.0,
        gps_position[0],
        gps_position[1],
        0.0)

    print("ENU position: ")
    print(enu_position2)

if __name__ == "__main__":

    # gps_origin = [40.158157, -3.381046]
    gps_origin = [-3.381046, 40.158157]
    enu_position = [1.0, 1.0]

    test(gps_origin, enu_position)