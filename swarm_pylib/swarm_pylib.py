from swarm_pylib.back_and_force import BackAndForce
from swarm_pylib.binpat import Binpat
import utm

class Swarm():

    @staticmethod
    def swarm_planning_gps(
            uavs_state: dict,
            area_values: list,
            path_algorithm: str,
            distribution_algorithm: str,
            street_spacing: float,
            waypoints_spacing: float,
            theta: float = None,
            uavs_weight_list: list = None,
            filter_path_waypoints: bool = False) -> list:
        """ Get UAVs path for a given area to be covered, in GPS coordinates
        Args:
            uav_state (dict): Dictionary with keys as uav names and values as another
                dictionary with keys as 'initial_position' and 'last_position' and 
                values as a list with latitute, longitude and height
            area_values (list): List of vertexes with latitute, longitude and height values 
                of the area
            path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
            distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
            street_spacing (float): Space between streets
            waypoints_spacing (float): Space between waypoints
            theta (float): Angle of the area. Default 'None' is automatic
            uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
                weights for all UAVs
            filter_waypoints (bool): Filter waypoints. Default 'False' is no filter
        Returns:
            uavs_path (list): List of lists with latitute, longitude and height values of the 
                path for each UAV
        """

        # Process the data input
        initial_position_list, last_position_list, uavs_weight_list = Swarm.data_input_process(
            uavs_state,
            uavs_weight_list)

        origin = area_values[0]

        # Convert the area to ENU coordinates
        area_enu = []
        for vertex in area_values:
            area_enu.append(Swarm.convert_gps2enu(
                origin,
                [vertex[0], vertex[1]],
                vertex[2]))

        # Convert the initial and last positions to ENU coordinates
        initial_position_list_enu = []
        last_position_list_enu = []
        for uav in initial_position_list:
            initial_position_list_enu.append(Swarm.convert_gps2enu(
                origin,
                [uav[0], uav[1]],
                uav[2]))
        for uav in last_position_list:
            last_position_list_enu.append(Swarm.convert_gps2enu(
                origin,
                [uav[0], uav[1]],
                uav[2]))

        # Compute the path
        uavs_path_enu = Swarm.compute_swarms_algorithm(
            initial_position_list_enu,
            last_position_list_enu,
            area_enu,
            path_algorithm,
            distribution_algorithm,
            street_spacing,
            waypoints_spacing,
            theta,
            uavs_weight_list,
            filter_path_waypoints)

        # Convert the path to GPS coordinates
        uavs_path = []
        for uav_path_enu in uavs_path_enu:
            uav_path = []
            for vertex in uav_path_enu:
                uav_path.append(Swarm.convert_enu2gps(
                    origin,
                    [vertex[0], vertex[1]],
                    vertex[2]))
            uavs_path.append(uav_path)
        return uavs_path

    @staticmethod
    def convert_enu2gps(gps_origin, enu_position, height):
        """ Convert ENU position to GPS position
        Args:
            gps_origin (list): List with latitude and longitude values of the GPS origin
            enu_position (list): List with x and y values of the ENU position
            height (float): Height of the position
        Returns:
            gps_position (list): List with latitude, longitude and height values of the GPS position
        """
        # gps_position = enu2geodetic(
        #     gps_origin[0],
        #     gps_origin[1],
        #     0.0,
        #     enu_position[0],
        #     enu_position[1],
        #     0.0)
        # return [gps_position[0], gps_position[1], height]

        utm_reference = utm.from_latlon(gps_origin[0], gps_origin[1])

        latitude, longitude = utm.to_latlon(
            enu_position[0] + utm_reference[0],
            enu_position[1] + utm_reference[1],
            utm_reference[2],
            utm_reference[3])

        return [latitude, longitude, height]

    @staticmethod
    def convert_gps2enu(gps_origin, gps_position, height):
        """ Convert GPS position to ENU position
        Args:
            gps_origin (list): List with latitude and longitude values of the GPS origin
            gps_position (list): List with latitude and longitude values of the GPS position
            height (float): Height of the position
        Returns:
            enu_position (list): List with x and y values of the ENU position
        """
        # enu_position = geodetic2enu(
        #     gps_origin[0],
        #     gps_origin[1],
        #     0.0,
        #     gps_position[0],
        #     gps_position[1],
        #     0.0)
        # return [enu_position[0], enu_position[1], height]

        utm_reference = utm.from_latlon(gps_origin[0], gps_origin[1])
        utm_position = utm.from_latlon(gps_position[0], gps_position[1])

        return [utm_position[0] - utm_reference[0], utm_position[1] - utm_reference[1], height]

    @staticmethod
    def swarm_planning(
            uavs_state: dict,
            area_values: list,
            path_algorithm: str,
            distribution_algorithm: str,
            street_spacing: float,
            waypoints_spacing: float,
            theta: float = None,
            uavs_weight_list: list = None,
            filter_path_waypoints: bool = False) -> list:
        """ Get UAVs path for a given area to be covered
        Args:
            uav_state (dict): Dictionary with keys as uav names and values as another
                dictionary with keys as 'initial_position' and 'last_position' and 
                values as a list with x, y and height
            area_values (list): List of vertexes with x, y and height values of the area
            path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
            distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
            street_spacing (float): Space between streets
            waypoints_spacing (float): Space between waypoints
            theta (float): Angle of the area. Default 'None' is automatic
            uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
                weights for all UAVs
            filter_waypoints (bool): Filter waypoints. Default 'False' is no filter
        Returns:
            uavs_path (list): List of lists with x, y and height values of the path for each UAV
        """

        # Process the data input
        initial_position_list, last_position_list, uavs_weight_list = Swarm.data_input_process(
            uavs_state,
            uavs_weight_list)
        
        return Swarm.compute_swarms_algorithm(
            initial_position_list,
            last_position_list,
            area_values,
            path_algorithm,
            distribution_algorithm,
            street_spacing,
            waypoints_spacing,
            theta,
            uavs_weight_list,
            filter_path_waypoints)

    @staticmethod
    def compute_swarms_algorithm(
            initial_position_list: list,
            last_position_list: list,
            area_values: list,
            path_algorithm: str,
            distribution_algorithm: str,
            street_spacing: float,
            waypoints_spacing: float,
            theta: float = None,
            uavs_weight_list: list = None,
            filter_path_waypoints: bool = False) -> list:
        """ Get UAVs path for a given area to be covered
        Args:
            initial_position_list (list): List of initial positions of the UAVs
            last_position_list (list): List of last positions of the UAVs
            area_values (list): List of vertexes with x, y and height values of the area
            path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
            distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
            street_spacing (float): Space between streets
            waypoints_spacing (float): Space between waypoints
            theta (float): Angle of the area. Default 'None' is automatic
            uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
                weights for all UAVs
            filter_waypoints (bool): Filter waypoints. Default 'False' is no filter
        Returns:
            uavs_path (list): List of lists with x, y and height values of the path for each UAV
        """

        # Algorithm to transform the area into a grid of waypoints with a sequence of streets
        path_waypoints = Swarm.generate_path(
            path_algorithm,
            initial_position_list,
            last_position_list,
            area_values,
            street_spacing,
            waypoints_spacing,
            theta)

        # Algorithm to distribute the waypoints of the area to each UAV
        uavs_path = Swarm.distribute_path(
            distribution_algorithm,
            initial_position_list,
            last_position_list,
            path_waypoints,
            uavs_weight_list)

        if filter_path_waypoints:
            uavs_path = Swarm.filter_waypoints(uavs_path)

        # Add first and last waypoints to the path
        uavs_path = Swarm.add_first_last_waypoints(
            initial_position_list,
            last_position_list,
            uavs_path)
        return uavs_path

    @staticmethod
    def data_input_process(
            uavs_state,
            weight_list):
        """ Process the input data
        Args:
            uav_state (dict): Dictionary with keys as uav names and values as another
                dictionary with keys as 'initial_position' and 'last_position' and
                values as a list with x, y and height
            uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
                weights for all UAVs
        Returns:
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last
                position of each UAV
            uavs_weight_list (list): List of weights for each UAV
        """
        # Transform the uav_state dictionary into a list of initial and last positions
        initial_position_list = []
        for uav in uavs_state:
            initial_position_list.append(uavs_state[uav]['initial_position'])

        last_position_list = []
        for uav in uavs_state:
            last_position_list.append(uavs_state[uav]['last_position'])

        # If no weight input, generate one. Create a vector of weights for each UAV.
        # It's value is 1 divided by the number of UAVs
        if weight_list is None:
            n_uavs = len(uavs_state.keys()) if len(uavs_state.keys()) > 0 else 1
            weight_list = [1/len(uavs_state.keys())] * n_uavs

        return initial_position_list, last_position_list, weight_list


    @staticmethod
    def generate_path(
            path_algorithm,
            initial_position_list,
            last_position_list,
            area_values,
            street_spacing,
            waypoints_spacing,
            theta=None):
        """ Algorithm to transform the area into a grid of waypoints with a sequence of streets
        Args:
            path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last
                position of each UAV
            area_values (list): List of vertexes with x, y and height values of the area
            street_spacing (float): Space between streets
            waypoints_spacing (float): Space between waypoints
            theta (float): Angle of the area (in degrees). Default 'None' is automatic selected
        Returns:
            path_waypoints (list): List of lists with x, y and height values of the path to
                cover the area
        """

        if path_algorithm == 'back_and_force':
            path_waypoints = BackAndForce.generate_path(
                initial_position_list,
                last_position_list,
                area_values,
                street_spacing,
                waypoints_spacing,
                theta)
        else:
            raise ValueError(f'Path algorithm {path_algorithm} not implemented')

        return path_waypoints

    @staticmethod
    def distribute_path(
            distribution_algorithm,
            initial_position_list,
            last_position_list,
            path_waypoints,
            uavs_weight_list=None):
        """ Algorithm to distribute the waypoints of the area to each UAV
        Args:
            distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last
                position of each UAV
            path_waypoints (list): List of lists with x, y and height values of the path to
                cover the area
            uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal weights
                for all UAVs
        Returns:
            uavs_path (list): List of lists with x, y and height values of the path for each UAV
        """

        if distribution_algorithm == 'binpat':
            uavs_path = Binpat.distribute_area(
                initial_position_list,
                last_position_list,
                path_waypoints,
                uavs_weight_list)
        else:
            raise ValueError(f'Path algorithm {distribution_algorithm} not implemented')

        return uavs_path

    @staticmethod
    def filter_waypoints(uavs_path):
        """ Filter the waypoints to remove the ones that are in the same line of the previous and
            next waypoints, checking only x and y values
        Args:
            uavs_path (list): List of lists with x, y and height values of the path for each UAV
        Returns:
            uavs_path_filtered (list): List of lists with x, y and height values of the path to cover the area
        """
        uavs_path_filtered = []
        for uav_path in uavs_path:
            uav_path_filtered = []
            for i in range(len(uav_path)):
                if i == 0 or i == len(uav_path) - 1:
                    uav_path_filtered.append(uav_path[i])
                else:
                    if not Swarm.check_colllinear_points(
                            uav_path[i-1],
                            uav_path[i],
                            uav_path[i+1]):
                        uav_path_filtered.append(uav_path[i])
            uavs_path_filtered.append(uav_path_filtered)
        return uavs_path_filtered

    @staticmethod
    def check_colllinear_points(
            point_ini: list,
            point_mid: list,
            point_end: list):
        """ Check if the middle point is in the same line of the initial and end points
        Args:
            point_ini (list): List with x, y and height values of the initial point
            point_mid (list): List with x, y and height values of the middle point
            point_end (list): List with x, y and height values of the end point
        Returns:
            bool: True if the middle point is in the same line of the initial and end points
        """ 
        triangle_area = point_ini[0] * (point_mid[1] - point_end[1]) + \
                        point_mid[0] * (point_end[1] - point_ini[1]) + \
                        point_end[0] * (point_ini[1] - point_mid[1])

        if abs(triangle_area) < 0.00001:
            return True
        else:
            return False
        
        # Another approach using slopes
        # slope_ini_mid = (point_mid[1] - point_ini[1]) / (point_mid[0] - point_ini[0])
        # slope_mid_end = (point_end[1] - point_mid[1]) / (point_end[0] - point_mid[0])
        # if abs(slope_ini_mid - slope_mid_end) < 0.00001:
        #     return True
        # else:
        #     return False

    @staticmethod
    def add_first_last_waypoints(
            initial_position_list: list,
            last_position_list: list,
            uavs_path: list):
        """ Add the initial and last waypoints to the path of each UAV
        Args:
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last
                position of each UAV
            uavs_path (list): List of lists with x, y and height values of the path for each UAV
        Returns:
            uavs_path (list): List of lists with x, y and height values of the path for each UAV 
            with the initial and last waypoints added
        """
        for i in range(len(uavs_path)):
            uavs_path[i].insert(0, initial_position_list[i])
            uavs_path[i].append(last_position_list[i])
        return uavs_path


def _test_swarm_planning_gps_input_local(
        uavs_state: dict,
        area_values: list,
        path_algorithm: str,
        distribution_algorithm: str,
        street_spacing: float,
        waypoints_spacing: float,
        theta: float = None,
        uavs_weight_list: list = None,
        filter_path_waypoints: bool = False,
        plot: bool = False) -> list:
    """ Get UAVs path for a given area to be covered
    Args:
        uav_state (dict): Dictionary with keys as uav names and values as another
                        dictionary with keys as 'initial_position' and 'last_position'
                        and values as a list with x, y and height
        area_values (list): List of vertexes with x, y and height values of the area
        path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
        distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
        street_spacing (float): Space between streets
        waypoints_spacing (float): Space between waypoints
        theta (float): Angle of the area. Default 'None' is automatic
        uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
            weights for all UAVs
        filter_waypoints (bool): Filter waypoints. Default 'False' is no filter
        plot (bool): Plot the area and the path. Default 'False' is no plot
    Returns:
        uavs_path (list): List of lists with x, y and height values of the path for each UAV
    """

    ## INPUT FROM LOCAL TO GPS

    # Process the data input
    initial_position_list, last_position_list, uavs_weight_list = Swarm.data_input_process(
        uavs_state,
        uavs_weight_list)

    gps_origin = [40.158157, -3.381046]
    # Convert area from ENU to GPS
    area_values_gps = []
    for vertex in area_values:
        area_values_gps.append(Swarm.convert_enu2gps(gps_origin, vertex[:2], vertex[2]))

    # Convert UAVs state from ENU to GPS
    initial_position_list_gps = []
    last_position_list_gps = []
    for uav in initial_position_list:
        initial_position_list_gps.append(Swarm.convert_enu2gps(gps_origin, uav[:2], uav[2]))
    for uav in last_position_list:
        last_position_list_gps.append(Swarm.convert_enu2gps(gps_origin, uav[:2], uav[2]))

    ## COMPUTE TEST IN GPS

    # Get UAVs path
    uavs_path = _test_swarm_planning_gps(
        initial_position_list_gps,
        last_position_list_gps,
        area_values_gps,
        path_algorithm,
        distribution_algorithm,
        street_spacing,
        waypoints_spacing,
        theta,
        uavs_weight_list,
        filter_path_waypoints,
        plot)

    print("UAVs path in GPS")
    print(uavs_path)

    ## OUTPUT FROM GPS TO LOCAL

    # Convert UAVs path from GPS to ENU
    uavs_enu_path = []
    for uav in uavs_path:
        uav_path = []
        for vertex in uav:
            uav_path.append(Swarm.convert_gps2enu(gps_origin, vertex[:2], vertex[2]))
        uavs_enu_path.append(uav_path)

    if plot:
        import matplotlib.pyplot as plt

        ## PLOT LOCAL
        plt.figure()

        # Plot the area
        plt.plot([area_values[0][0], area_values[1][0], area_values[2][0], area_values[3][0], area_values[0][0]],
                [area_values[0][1], area_values[1][1], area_values[2][1], area_values[3][1], area_values[0][1]],
                'k-')

        # Plot initial and final positions
        for i, uav_pos in enumerate(initial_position_list):
            plt.plot(uav_pos[0], uav_pos[1], 'ro')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        for i, uav_pos in enumerate(last_position_list):
            plt.plot(uav_pos[0], uav_pos[1], 'go')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        # Plot UAVs paths
        for i, uav_path in enumerate(uavs_enu_path):
            # Plot path lines
            x = [waypoint[0] for waypoint in uav_path]
            y = [waypoint[1] for waypoint in uav_path]
            plt.plot(x, y, '-')

            # Plot waypoints
            for j, waypoint in enumerate(uav_path):
                if j == 0:
                    color = 'go'
                elif j == len(uav_path) - 1:
                    color = 'ro'
                else:
                    color = 'bo'
                plt.plot(waypoint[0], waypoint[1], color)

        plt.show()

        ## PLOT GPS
        plt.figure()

        # Plot the area
        plt.plot([area_values_gps[0][0], area_values_gps[1][0], area_values_gps[2][0], area_values_gps[3][0], area_values_gps[0][0]],
                [area_values_gps[0][1], area_values_gps[1][1], area_values_gps[2][1], area_values_gps[3][1], area_values_gps[0][1]],
                'k-')

        # Plot initial and final positions
        for i, uav_pos in enumerate(initial_position_list_gps):
            plt.plot(uav_pos[0], uav_pos[1], 'ro')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        for i, uav_pos in enumerate(last_position_list_gps):
            plt.plot(uav_pos[0], uav_pos[1], 'go')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        # Plot UAVs paths
        for i, uav_path in enumerate(uavs_path):
            # Plot path lines
            x = [waypoint[0] for waypoint in uav_path]
            y = [waypoint[1] for waypoint in uav_path]
            plt.plot(x, y, '-')

            # Plot waypoints
            for j, waypoint in enumerate(uav_path):
                if j == 0:
                    color = 'go'
                elif j == len(uav_path) - 1:
                    color = 'ro'
                else:
                    color = 'bo'
                plt.plot(waypoint[0], waypoint[1], color)

        plt.show()
    return uavs_path


def _test_swarm_planning_gps(
        initial_position_list: list,
        last_position_list: list,
        area_values: list,
        path_algorithm: str,
        distribution_algorithm: str,
        street_spacing: float,
        waypoints_spacing: float,
        theta: float = None,
        uavs_weight_list: list = None,
        filter_path_waypoints: bool = False,
        plot: bool = False) -> list:
    """ Get UAVs path for a given area to be covered, in GPS coordinates
    Args:
        initial_position_list (list): List of initial positions of the UAVs
        last_position_list (list): List of last positions of the UAVs
        area_values (list): List of vertexes with latitute, longitude and height values 
            of the area
        path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
        distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
        street_spacing (float): Space between streets
        waypoints_spacing (float): Space between waypoints
        theta (float): Angle of the area. Default 'None' is automatic
        uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
            weights for all UAVs
        filter_waypoints (bool): Filter waypoints. Default 'False' is no filter
        plot (bool): Plot the area and the path. Default 'False' is no plot
    Returns:
        uavs_path (list): List of lists with latitute, longitude and height values of the 
            path for each UAV
    """

    origin = area_values[0]

    # Convert the area to ENU coordinates
    area_enu = []
    for vertex in area_values:
        area_enu.append(Swarm.convert_gps2enu(
            origin,
            [vertex[0], vertex[1]],
            vertex[2]))

    # Convert the initial and last positions to ENU coordinates
    initial_position_list_enu = []
    last_position_list_enu = []
    for uav in initial_position_list:
        initial_position_list_enu.append(Swarm.convert_gps2enu(
            origin,
            [uav[0], uav[1]],
            uav[2]))
    for uav in last_position_list:
        last_position_list_enu.append(Swarm.convert_gps2enu(
            origin,
            [uav[0], uav[1]],
            uav[2]))

    # Compute the path
    uavs_path_enu = _test_compute_swarms_algorithm(
        initial_position_list_enu,
        last_position_list_enu,
        area_enu,
        path_algorithm,
        distribution_algorithm,
        street_spacing,
        waypoints_spacing,
        theta,
        uavs_weight_list,
        filter_path_waypoints,
        plot)

    # Convert the path to GPS coordinates
    uavs_path = []
    for uav_path_enu in uavs_path_enu:
        uav_path = []
        for vertex in uav_path_enu:
            uav_path.append(Swarm.convert_enu2gps(
                origin,
                [vertex[0], vertex[1]],
                vertex[2]))
        uavs_path.append(uav_path)
    return uavs_path


def _test_swarm_planning(
        uavs_state,
        area_values,
        path_algorithm,
        distribution_algorithm,
        street_spacing,
        waypoints_spacing,
        theta=None,
        uavs_weight_list=None,
        filter_path_waypoints=False,
        plot: bool = False) -> list:
    """ Get UAVs path for a given area to be covered
    Args:
        uav_state (dict): Dictionary with keys as uav names and values as another
                        dictionary with keys as 'initial_position' and 'last_position'
                        and values as a list with x, y and height
        area_values (list): List of vertexes with x, y and height values of the area
        path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
        distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
        street_spacing (float): Space between streets
        waypoints_spacing (float): Space between waypoints
        theta (float): Angle of the area. Default 'None' is automatic
        uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
            weights for all UAVs
        filter_waypoints (bool): Filter waypoints. Default 'False' is no filter
        plot (bool): Plot the area and the path. Default 'False' is no plot
    Returns:
        uavs_path (list): List of lists with x, y and height values of the path for each UAV
    """

    # Process the data input
    initial_position_list, last_position_list, uavs_weight_list = Swarm.data_input_process(
        uavs_state,
        uavs_weight_list)

    # Compute the path
    return _test_compute_swarms_algorithm(
        initial_position_list,
        last_position_list,
        area_values,
        path_algorithm,
        distribution_algorithm,
        street_spacing,
        waypoints_spacing,
        theta,
        uavs_weight_list,
        filter_path_waypoints,
        plot)


def _test_compute_swarms_algorithm(
        initial_position_list: list,
        last_position_list: list,
        area_values: list,
        path_algorithm: str,
        distribution_algorithm: str,
        street_spacing: float,
        waypoints_spacing: float,
        theta: float = None,
        uavs_weight_list: list = None,
        filter_path_waypoints: bool = False,
        plot: bool = False) -> list:
    """ Get UAVs path for a given area to be covered
    Args:
        initial_position_list (list): List of initial positions of the UAVs
        last_position_list (list): List of last positions of the UAVs
        area_values (list): List of vertexes with x, y and height values of the area
        path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
        distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
        street_spacing (float): Space between streets
        waypoints_spacing (float): Space between waypoints
        theta (float): Angle of the area. Default 'None' is automatic
        uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal
            weights for all UAVs
        filter_waypoints (bool): Filter waypoints. Default 'False' is no filter
        plot (bool): Plot the area and the path. Default 'False' is no plot
    Returns:
        uavs_path (list): List of lists with x, y and height values of the path for each UAV
    """

    # Generate the path
    path_waypoints = _test_generate_path(
        path_algorithm,
        initial_position_list,
        last_position_list,
        area_values,
        street_spacing,
        waypoints_spacing,
        theta)

    # Distribute the path to each UAV
    uavs_path = _test_distribute_path(
        distribution_algorithm,
        initial_position_list,
        last_position_list,
        path_waypoints,
        uavs_weight_list)

    # Filter the waypoints
    if filter_path_waypoints:
        uavs_path = Swarm.filter_waypoints(uavs_path)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot the area
        plt.plot([area_values[0][0], area_values[1][0], area_values[2][0], area_values[3][0], area_values[0][0]],
                [area_values[0][1], area_values[1][1], area_values[2][1], area_values[3][1], area_values[0][1]],
                'k-')

        # Plot initial and final positions
        for i, uav_pos in enumerate(initial_position_list):
            plt.plot(uav_pos[0], uav_pos[1], 'ro')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        for i, uav_pos in enumerate(last_position_list):
            plt.plot(uav_pos[0], uav_pos[1], 'go')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        # Plot the path lines
        for i in range(len(path_waypoints) - 1):
            plt.plot([path_waypoints[i][0], path_waypoints[i + 1][0]],
                    [path_waypoints[i][1], path_waypoints[i + 1][1]],
                    'r')

        # Plot path waypoints
        for point in path_waypoints:
            plt.plot(point[0], point[1], 'yo')

        # Plot UAVs paths
        for i, uav_path in enumerate(uavs_path):
            # Plot path lines
            x = [waypoint[0] for waypoint in uav_path]
            y = [waypoint[1] for waypoint in uav_path]
            plt.plot(x, y, '-')

            # Plot waypoints
            for j, waypoint in enumerate(uav_path):
                if j == 0:
                    color = 'go'
                elif j == len(uav_path) - 1:
                    color = 'ro'
                else:
                    color = 'bo'
                plt.plot(waypoint[0], waypoint[1], color)

        plt.show()

    return uavs_path


def _test_generate_path(
        path_algorithm,
        initial_position_list,
        last_position_list,
        area_values,
        street_spacing,
        waypoints_spacing,
        theta=None,
        plot: bool = False):
    """ Algorithm to transform the area into a grid of waypoints with a sequence of streets
    Args:
        path_algorithm (str): Path algorithm to use. Options are: 'back_and_force'
        initial_position_list (list): List of lists with x, y and height values of the initial
            position of each UAV
        last_position_list (list): List of lists with x, y and height values of the last
            position of each UAV
        area_values (list): List of vertexes with x, y and height values of the area
        street_spacing (float): Space between streets
        waypoints_spacing (float): Space between waypoints
        theta (float): Angle of the area (in degrees). Default 'None' is automatic selected
        plot (bool): Plot the area and the path. Default 'False' is no plot
    Returns:
        path_waypoints (list): List of lists with x, y and height values of the path to
            cover the area
    """

    # Algorithm to transform the area into a grid of waypoints with a sequence of streets
    path_waypoints = Swarm.generate_path(
        path_algorithm,
        initial_position_list,
        last_position_list,
        area_values,
        street_spacing,
        waypoints_spacing,
        theta)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot the area
        plt.plot([area_values[0][0], area_values[1][0], area_values[2][0], area_values[3][0], area_values[0][0]],
                [area_values[0][1], area_values[1][1], area_values[2][1], area_values[3][1], area_values[0][1]],
                'k-')

        # Plot initial and final positions
        for i, uav_pos in enumerate(initial_position_list):
            plt.plot(uav_pos[0], uav_pos[1], 'ro')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        for i, uav_pos in enumerate(last_position_list):
            plt.plot(uav_pos[0], uav_pos[1], 'go')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))

        # Plot the path lines
        for i in range(len(path_waypoints) - 1):
            plt.plot([path_waypoints[i][0], path_waypoints[i + 1][0]],
                    [path_waypoints[i][1], path_waypoints[i + 1][1]],
                    'r')

        # Plot path waypoints
        for point in path_waypoints:
            plt.plot(point[0], point[1], 'yo')

        plt.show()

    return path_waypoints


def _test_distribute_path(
        distribution_algorithm,
        initial_position_list,
        last_position_list,
        path_waypoints,
        uavs_weight_list=None,
        plot: bool = False):
    """ Algorithm to distribute the waypoints of the area to each UAV
    Args:
        distribution_algorithm (str): Distribution algorithm to use. Options are: 'binpat'
        initial_position_list (list): List of lists with x, y and height values of the initial
            position of each UAV
        last_position_list (list): List of lists with x, y and height values of the last
            position of each UAV
        path_waypoints (list): List of lists with x, y and height values of the path to
            cover the area
        uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal weights
            for all UAVs
        plot (bool): Plot the area and the path. Default 'False' is no plot
    Returns:
        uavs_path (list): List of lists with x, y and height values of the path for each UAV
    """

    uavs_path = Swarm.distribute_path(
        distribution_algorithm,
        initial_position_list,
        last_position_list,
        path_waypoints,
        uavs_weight_list)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot path waypoints
        for point in path_waypoints:
            plt.plot(point[0], point[1], 'yo')

        # Plot UAVs paths
        for i, uav_path in enumerate(uavs_path):
            # Plot path lines
            x = [waypoint[0] for waypoint in uav_path]
            y = [waypoint[1] for waypoint in uav_path]
            plt.plot(x, y, '-')

            # Plot waypoints
            for j, waypoint in enumerate(uav_path):
                if j == 0:
                    color = 'go'
                elif j == len(uav_path) - 1:
                    color = 'ro'
                else:
                    color = 'bo'
                plt.plot(waypoint[0], waypoint[1], color)

        plt.show()

    return uavs_path


if __name__ == "__main__":

    position = [-1.0, 6.0, 10.0]
    uavs_state = {
        "drone_1": {
            "initial_position": position,
            "last_position": position,
        },
        "drone_2": {
            "initial_position": position,
            "last_position": position,
        }
    }

    area_height = 10.0
    area_values = [
        [  0.0,  0.0, area_height],
        [100.0,  0.0, area_height],
        [100.0, 50.0, area_height],
        [  0.0, 50.0, area_height]]

    path_algorithm = 'back_and_force'
    distribution_algorithm = 'binpat'

    street_spacing = 10.0
    waypoints_spacing = 4.0

    theta = None
    uavs_weight_list = None
    filter_path_waypoints = True

    path_waypoints = _test_swarm_planning_gps_input_local(
        uavs_state,
        area_values,
        path_algorithm,
        distribution_algorithm,
        street_spacing,
        waypoints_spacing,
        theta,
        uavs_weight_list,
        filter_path_waypoints,
        True)
