""" Back and force algorithm """
from swarm_pylib.swarm_algorithms_templates import PathAlgorithm
import numpy as np
import math
from typing import List, Tuple
from matplotlib import path
from numpy import linalg as LA


class BackAndForce(PathAlgorithm):
    """ Back and force algorithm """

    @staticmethod
    def generate_path(
            initial_position_list: list,
            last_position_list: list,
            area_values: list,
            street_spacing: float,
            waypoints_spacing: float,
            theta: float = None) -> list:
        """ Algorithm to transform the area into a grid of waypoints with a sequence of streets
        Args:
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
        path_waypoints = BackAndForce.compute_back_and_force(
            initial_position_list,
            area_values,
            street_spacing,
            waypoints_spacing,
            theta)
        return path_waypoints

    @staticmethod
    def compute_back_and_force(
            initial_position_list,
            area_values,
            street_spacing,
            waypoints_spacing,
            theta=None):
        """ Algorithm to transform the area into a grid of waypoints with a sequence of streets
        Args:
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            area_values (list): List of vertexes with x, y and height values of the area
            street_spacing (float): Space between streets
            waypoints_spacing (float): Space between waypoints
            theta (float): Angle of the area (in degrees). Default 'None' is automatic selected
        Returns:
            path_waypoints (list): List of lists with x, y and height values of the path to
                cover the area
        """

        area_h, area_np, init_position_list_np = BackAndForce.data_input_process(
            area_values,
            initial_position_list)

        # Get area with desired or optimal angle
        area, angle = BackAndForce.process_area_values(area_np, theta)

        # Get waypoints of the area contour for street generation
        streets_lines = BackAndForce.get_area_contour_waypoints(
            area, street_spacing)

        # Generate streets path
        contour_path = BackAndForce.generate_contour_path(
            init_position_list_np,
            streets_lines)

        # Divide path into waypoints
        path_waypoints = BackAndForce.divide_path_into_waypoints(
            contour_path,
            waypoints_spacing)

        # Rotate path to the original angle
        path_original_angle = BackAndForce.rotate_path_to_original_angle(
            path_waypoints,
            angle)

        # Add height to the path
        path_waypoints_height = BackAndForce.add_height_to_path(
            path_original_angle,
            area_h)

        return path_waypoints_height

    @staticmethod
    def data_input_process(
            area_values: list,
            initial_position_list: list) -> np.array:
        """ Process the input data to be used in the algorithm
        Args:
            area_values (list): List of vertexes with x, y and height values of the area
        Returns:
            area_height (float): Height of the area
            area_values (np.array): List of lists with x and y values of the path to
                cover the area
            initial_position_list (np.array): x and y values of the initial position of each UAV
        """

        # Remove height from area values
        position_list, height_list = PathAlgorithm._extract_height_from_list(area_values)
        area_np = np.array(position_list)

        # Area height
        area_height = height_list[0]

        # Get initial position list without height
        init_pos_list, _ = PathAlgorithm._extract_height_from_list(initial_position_list)
        init_pos_list = np.array(init_pos_list)

        return area_height, area_np, init_pos_list

    @staticmethod
    def process_area_values(
            area_values: np.array,
            theta: float) -> np.array:
        """ Process area values
        Args:
            area_values (np.array): List of vertexes with x and y values of the area
            theta (float): Angle of the area in degrees. Default 'None' is automatic
        Returns:
            area (np.array): Array with x and y values of the area
        """

        # If theta is None, get the angle that maximize the area width
        if theta is None:
            theta = BackAndForce.get_optimal_angle(area_values)
        # Use the input theta
        else:
            theta = float(theta) * math.pi / 180

        # Rotate area and get the width
        rotation_matrix = np.array([
            [math.cos(theta), -math.sin(theta)],
            [math.sin(theta), math.cos(theta)]])

        area_rotated = np.dot(rotation_matrix, np.transpose(area_values))

        return area_rotated, theta

    @staticmethod
    def get_optimal_angle(area_values: np.array) -> float:
        """ Get optimal angle for area streets
        Args:
            area_values np.array: List of vertexes with x and y  values of the area
        Returns:
            theta (float): Angle of the area in degrees
        """
        # Get initial area width
        area_width = max(area_values[:, 0]) - min(area_values[:, 0])

        for i in range(-90, 90):
            theta_candidate = i * math.pi / 180

            # Rotate area and get the width
            rotation_matrix = np.array([
                [math.cos(theta_candidate), -math.sin(theta_candidate)],
                [math.sin(theta_candidate), math.cos(theta_candidate)]])

            area_rotated = np.dot(rotation_matrix, np.transpose(area_values))

            if max(area_rotated[0, :]) - min(area_rotated[0, :]) > area_width:
                theta = theta_candidate
                area_width = max(area_rotated[0, :]) - min(area_rotated[0, :])

        return theta

    @staticmethod
    def get_area_contour_waypoints(
            area_values: np.array,
            street_spacing: float) -> np.array:
        """ Generate area streets intersections
        Args:
            area_values (np.array): Array with x and y values of the area
            street_spacing (float): Street spacing
        Returns:
            streets_lines (np.array): Array with x and y values of the area streets intersections
                sort by y coordinate
        """
        # Get area height
        area_contour_height = max(area_values[1, :]) - min(area_values[1, :])

        # Get number of streets
        if street_spacing == 0:
            raise ValueError('Street spacing must be greater than 0')
        n_streets = int(math.ceil(area_contour_height / street_spacing))

        # Get distance between streets
        distance_between_streets = area_contour_height / n_streets

        # # Generate lines for each street in the area,
        # # parallel to the x axis and with separation of distance_between_streets,
        # # and start and end points of each street in the area contour
        streets_intersection_coordinates = []

        # Get a list of the y coordinates of each street, with separation of distance_between_streets
        y_coordinates = np.arange(min(area_values[1, :]), max(
            area_values[1, :]), distance_between_streets)

        # If the last street is not in the area contour, add it
        if y_coordinates[-1]+distance_between_streets == max(area_values[1, :]):
            y_coordinates = np.append(y_coordinates, max(area_values[1, :]))

        # Get the y min and max coordinates of each line segment of the area contour, iterating over the area values
        for i in range(len(area_values[0, :])):
            # Get the segment coordinates
            x1 = area_values[0, i]
            y1 = area_values[1, i]
            x2 = area_values[0, (i + 1) % len(area_values[0, :])]
            y2 = area_values[1, (i + 1) % len(area_values[0, :])]

            # Get the y min and max coordinates of the segment
            y_min_segment = min(y1, y2)
            y_max_segment = max(y1, y2)

            # Compute the point of intersection between the segment and each street

            # If the segment is parallel to the x axis, skip it, there is no intersection
            if y1 == y2:
                continue

            y_coordinates_between_segment = y_coordinates[(
                y_coordinates >= y_min_segment) & (y_coordinates <= y_max_segment)]

            # If the segment is vertical, the intersection are all the y coordinates between the segment
            if x1 == x2:
                for j in range(len(y_coordinates_between_segment)):
                    streets_intersection_coordinates.append(
                        [x1, float(y_coordinates_between_segment[j])])
                continue

            # If the segment is not vertical, compute the intersection
            for j in range(len(y_coordinates_between_segment)):
                y_street = y_coordinates_between_segment[j]
                x_street = x1 + (x2 - x1) * (y_street - y1) / (y2 - y1)
                streets_intersection_coordinates.append([x_street, y_street])

        # Sort points by y coordinate
        streets_intersection_coordinates = sorted(
            streets_intersection_coordinates, key=lambda x: x[1])

        # Create a np array with the coordinates
        streets_intersection_coordinates = np.array(
            streets_intersection_coordinates)

        # Sort points by y coordinates.
        # If two points have the same y coordinate, sort them by x coordinate
        # If there are more than two points with the same y coordinate,
        # keep the one with the min x coordinate and the one with the max x coordinate
        streets_lines = []
        for i in range(len(y_coordinates)):
            intersections_i = streets_intersection_coordinates[
                streets_intersection_coordinates[:, 1] == y_coordinates[i]]

            # Get intersection with min x coordinate in intersections_i
            x_min = min(intersections_i[:, 0])
            # Get intersection with max x coordinate in intersections_i
            x_max = max(intersections_i[:, 0])

            streets_lines.append([x_min, y_coordinates[i]])
            streets_lines.append([x_max, y_coordinates[i]])

        streets_lines = np.array(streets_lines)
        return streets_lines

    @staticmethod
    def generate_contour_path(
            init_position_list: np.array,
            streets_lines: np.array) -> np.array:
        """ Generate grid path
        Args:
            init_position_list (np.array): List of initial positions of the UAVs
            streets_lines (np.array): Array with x and y values of the area streets intersections
                sort by y coordinate
        Returns:
            contour_path (np.array): List of x and y values of vertexes to cover the area grid
        """
        initial_vertex_idx = BackAndForce.select_initial_vertex(
            streets_lines, init_position_list)

        contour_path = BackAndForce.generate_vertexes_path(
            streets_lines, initial_vertex_idx)

        return contour_path

    @staticmethod
    def select_initial_vertex(
            streets_lines: np.array,
            initial_position_list: np.array) -> np.array:
        """ Select initial vertex of the grid path
        Args:
            streets_lines (np.array): Array with x and y values of the area streets intersections
                sort by y coordinate
            initial_position_list (np.array): List of initial positions of the UAVs
        Returns:
            initial_vertex_index (str): Initial vertex index of street_lines
        """
        # Candidates to be the first and last streets are the extremes of the area contour
        first_street = streets_lines[streets_lines[:, 1] == min(
            streets_lines[:, 1]), :]
        last_street = streets_lines[streets_lines[:, 1] == max(
            streets_lines[:, 1]), :]
        candidates = np.concatenate((first_street, last_street), axis=0)

        # Select the candidate that is closer to the initial position of one of the UAVs
        cost = None
        selected_candidate_idx = None
        selected_uav_idx = None
        for uav_idx, uav_pos in enumerate(initial_position_list):
            for candidate_idx, candidate in enumerate(candidates):
                cost_candidate = np.linalg.norm(uav_pos - candidate)
                if cost is None or \
                   selected_uav_idx is None or \
                   cost_candidate < cost:
                    cost = cost_candidate
                    selected_uav_idx = uav_idx
                    selected_candidate_idx = candidate_idx

        # Get the index in streets_lines
        if selected_candidate_idx == 2:
            selected_candidate_idx = len(streets_lines)-2
        elif selected_candidate_idx == 3:
            selected_candidate_idx = len(streets_lines)-1

        return selected_candidate_idx

    @staticmethod
    def generate_vertexes_path(
            streets_lines: np.array,
            initial_vertex_idx: int) -> np.array:
        """ Generate path to cover the area grid
        Args:
            streets_lines (np.array): Array with x and y values of the area streets intersections
                sort by y coordinate
            initial_vertex_idx (int): Initial vertex index of street_lines
        Returns:
            contour_path (np.array): List of x and y values of vertexes to cover the area grid
        """
        # If initial_vertex_idx is 0 or 1, the path is generated from the first street to the last street
        # Else, the path is generated from the last street to the first street

        n_vertex = len(streets_lines)
        # Odd street lines vertex
        left_vertex = streets_lines[::2, :]
        # Even street lines vertex
        right_vertex = streets_lines[1::2, :]
        if initial_vertex_idx == n_vertex-1 or \
           initial_vertex_idx == n_vertex-2:

            # Sort vertex from last to first
            left_vertex = left_vertex[::-1]
            right_vertex = right_vertex[::-1]
        
        # Path is a numpy array with shape (n_vertex, 2)
        contour_path = np.zeros((n_vertex, 2))

        # Generate path to visit all streets
        # If initial_vertex_idx is 0 or n_vertex-2
        # The path will be:
        # - For odd street lines: from left to right
        # - For even street lines: from right to left
        # If initial_vertex_idx is 1 or n_vertex-1
        # The path will be:
        # - For odd street lines: from right to left
        # - For even street lines: from left to right

        if initial_vertex_idx == 0 or initial_vertex_idx == n_vertex-2:
            for i in range(len(left_vertex)):
                if i % 2 == 0:
                    contour_path[i*2] = left_vertex[i]
                    contour_path[i*2+1] = right_vertex[i]
                else:
                    contour_path[i*2] = right_vertex[i]
                    contour_path[i*2+1] = left_vertex[i]
        else:
            for i in range(len(left_vertex)):
                if i % 2 == 0:
                    contour_path[i*2] = right_vertex[i]
                    contour_path[i*2+1] = left_vertex[i]
                else:
                    contour_path[i*2] = left_vertex[i]
                    contour_path[i*2+1] = right_vertex[i]
                
        return contour_path

    @staticmethod
    def divide_path_into_waypoints(
            contour_path: np.array,
            waypoints_spacing: float) -> np.array:
        """ Algorithm to divide the path into waypoints
        Args:
            contour_path (np.array): List of x and y values of vertexes to cover the area grid
            waypoints_spacing (float): Distance between waypoints
        Returns:
            path_waypoints (np.array): List of lists with x and y values of the path to
                cover the area
        """
        # For each line (pair of vertexes) of the path, compute the distance between them
        # If the distance is greater than the waypoints spacing, divide the line into
        # waypoints_spacing segments
        # Always vertexes are added to the path_waypoints list
        path_waypoints = []
        for i in range(len(contour_path)-1):
            # Compute distance between vertexes
            distance = np.linalg.norm(contour_path[i+1] - contour_path[i])
            if distance > waypoints_spacing:
                # Compute number of waypoints to add
                n_waypoints = math.ceil(distance / waypoints_spacing)
                # Compute vector between vertexes
                vector = contour_path[i+1] - contour_path[i]
                # Compute vector between waypoints
                vector_waypoints = vector / n_waypoints
                # Add waypoints
                for j in range(n_waypoints):
                    path_waypoints.append(contour_path[i] + vector_waypoints * j)
            else:
                path_waypoints.append(contour_path[i])
        path_waypoints.append(contour_path[-1])
        return np.array(path_waypoints)

    @staticmethod
    def rotate_path_to_original_angle(
            path_waypoints: np.array,
            theta: float) -> list:
        """ Rotate path to original angle
        Args:
            path_waypoints (np.array): List of lists with x and y values of the path to
                cover the area
            theta (float): Angle of the area in degrees
        Returns:
            path_waypoints (list): List of lists with x and y values of the path to
                cover the area
        """
        # Inverse rotate path to original angle
        rotation_matrix = np.array([
            [ math.cos(theta), math.sin(theta)],
            [-math.sin(theta), math.cos(theta)]])

        # Rotate path
        path_original_angle = np.dot(rotation_matrix, np.transpose(path_waypoints))
        return path_original_angle.transpose().tolist()

    @staticmethod
    def add_height_to_path(
            path_waypoints: list,
            height: float) -> list:
        """ Add height to path
        Args:
            path_waypoints (list): List of lists with x and y values of the path to
                cover the area
            height (float): Height of the area
        Returns:
            path_waypoints (list): List of lists with x, y and height values of the path to
                cover the area
        """
        # Add height to path
        path_waypoints_height = PathAlgorithm._add_height_to_position_list(
            path_waypoints,
            height)
        return path_waypoints_height

def _test_get_optimal_angle(area_values: list) -> float:
    """ Test angle compute
    Args:
        area_values (list): List of vertexes with x and y values of the area
    Returns:
        theta (float): Angle of the area in degrees
    """
    theta = BackAndForce.get_optimal_angle(area_values)

    print("Optimal angle: " + str(theta * 180 / math.pi) + " degrees")
    return theta


def _test_process_area_values(
        area_values: np.array,
        theta: float,
        plot: bool = False) -> np.array:
    """ Process area values
    Args:
        area_values (np.array): List of vertexes with x and y values of the area
        theta (float): Angle of the area in degrees. Default 'None' is automatic
        plot (bool): Plot the area and the rotated area. Default 'False'
    Returns:
        area (np.array): Array with x and y values of the area
    """
    area_rotated, angle = BackAndForce.process_area_values(area_values, theta)

    print("Angle: " + str(angle * 180 / math.pi) + " degrees")

    print("Area rotated vortexes: ")
    for vortex in area_rotated.T:
        print(vortex)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot the area
        area_np = np.array(area_values)
        for i in range(0, len(area_np)):
            plt.plot([area_np[i][0], area_np[(i + 1) % len(area_np)][0]],
                    [area_np[i][1], area_np[(i + 1) % len(area_np)][1]], 'b-')
        
        # Plot the area rotated
        for i in range(0, len(area_rotated[0, :])):
            plt.plot([area_rotated[0, i], area_rotated[0, (i + 1) % len(area_rotated[0, :])]],
                    [area_rotated[1, i], area_rotated[1, (i + 1) % len(area_rotated[0, :])]], 'k-')

        plt.show()

    return area_rotated, angle


def _test_get_area_contour_waypoints(
        area_rotated: np.array,
        street_spacing: float,
        plot: bool = False) -> np.array:
    """ Test generate area streets
    Args:
        area_rotated (np.array): Array with x and y values of the area
        street_spacing (float): Distance between streets
        plot (bool): Plot streets intersections with the area. Default 'False'
    Returns:
        streets_lines (np.array): Array with x and y values of the area streets intersections
            sort by y coordinate
    """
    streets_lines = BackAndForce.get_area_contour_waypoints(
        area_rotated, street_spacing)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot the streets intersections with the area
        for point in streets_lines:
            plt.plot(point[0], point[1], 'ro')

        plt.show()

    return streets_lines

def _test_generate_contour_path(
        init_position_list: np.array,
        streets_lines: np.array,
        plot: bool = False) -> np.array:
    """ Generate grid path
    Args:
        init_position_list (np.array): List of initial positions of the UAVs
        streets_lines (np.array): Array with x and y values of the area streets intersections
            sort by y coordinate
        plot (bool): Plot path contour waypoints. Default 'False'
    Returns:
        contour_path (np.array): List of x and y values of vertexes to cover the area grid
    """
    path_waypoints = BackAndForce.generate_contour_path(
        init_position_list, streets_lines)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot the path lines
        for i in range(len(path_waypoints) - 1):
            plt.plot([path_waypoints[i][0], path_waypoints[i + 1][0]],
                    [path_waypoints[i][1], path_waypoints[i + 1][1]],
                    'r')

        # Mark the initial and last positions of the path
        plt.plot(path_waypoints[0][0], path_waypoints[0][1], 'g*')
        plt.plot(path_waypoints[-1][0], path_waypoints[-1][1], 'b*')

        plt.show()

    return path_waypoints

def _test_divide_path_into_waypoints(
        contour_path: np.array,
        waypoints_spacing: float,
        plot: bool = False) -> np.array:
    """ Algorithm to divide the path into waypoints
    Args:
        contour_path (np.array): List of x and y values of vertexes to cover the area grid
        waypoints_spacing (float): Distance between waypoints
        plot (bool): Plot waypoints. Default 'False'
    Returns:
        path_waypoints (np.array): List of lists with x and y values of the path to
            cover the area
    """
    path_waypoints = BackAndForce.divide_path_into_waypoints(contour_path, waypoints_spacing)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot waypoints
        for point in path_waypoints:
            plt.plot(point[0], point[1], 'yo')

        plt.show()

    return path_waypoints

def _test_rotate_path_to_original_angle(
        path_waypoints: np.array,
        angle: float,
        plot: bool = False) -> np.array:
    """ Algorithm to rotate the path to the original angle
    Args:
        path_waypoints (np.array): List of lists with x and y values of the path to
            cover the area
        angle (float): Angle of the area (in radians)
        plot (bool): Plot the path rotated. Default 'False'
    Returns:
        path_rotated (np.array): List of lists with x and y values of the path to
            cover the area rotated
    """
    path_rotated = BackAndForce.rotate_path_to_original_angle(path_waypoints, angle)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot the path rotated
        for i in range(len(path_rotated) - 1):
            plt.plot([path_rotated[i][0], path_rotated[i + 1][0]],
                    [path_rotated[i][1], path_rotated[i + 1][1]],
                    'r')
        plt.show()

    return path_rotated

def _test_generate_path(
        initial_position_list,
        last_position_list,
        area_values,
        street_spacing,
        waypoints_spacing,
        theta=None,
        plot: bool = False):
    """ Algorithm to transform the area into a grid of waypoints with a sequence of streets
    Args:
        initial_position_list (list): List of lists with x, y and height values of the initial
            position of each UAV
        last_position_list (list): List of lists with x, y and height values of the last
            position of each UAV
        area_values (list): List of vertexes with x, y and height values of the area
        street_spacing (float): Space between streets
        waypoints_spacing (float): Space between waypoints
        theta (float): Angle of the area (in degrees). Default 'None' is automatic selected
        plot (bool): Plot the area, streets intersections, path and waypoints. Default 'False'
    Returns:
        path_waypoints (list): List of lists with x, y and height values of the path to
            cover the area
    """
    area_h, area_np, init_position_list_np = BackAndForce.data_input_process(
        area_values,
        initial_position_list)

    # Get area with desired or optimal angle
    area_rotated, angle = _test_process_area_values(area_np, theta, False)

    # Get waypoints of the area contour for street generation
    streets_lines = _test_get_area_contour_waypoints(area_rotated, street_spacing, False)

    # Generate streets path
    contour_path = _test_generate_contour_path(init_position_list_np, streets_lines, False)

    # Divide path into waypoints
    path_waypoints = _test_divide_path_into_waypoints(
        contour_path,
        waypoints_spacing,
        False)

    # Rotate path to the original angle
    path_original_angle = _test_rotate_path_to_original_angle(
        path_waypoints,
        angle,
        False)

    # Add height to the path
    path_waypoints_height = BackAndForce.add_height_to_path(
        path_original_angle,
        area_h)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot the area
        area_np = np.array(area_values)
        for i in range(0, len(area_np)):
            plt.plot([area_np[i][0], area_np[(i + 1) % len(area_np)][0]],
                    [area_np[i][1], area_np[(i + 1) % len(area_np)][1]], 'b-')
        
        # Plot the area rotated
        for i in range(0, len(area_rotated[0, :])):
            plt.plot([area_rotated[0, i], area_rotated[0, (i + 1) % len(area_rotated[0, :])]],
                    [area_rotated[1, i], area_rotated[1, (i + 1) % len(area_rotated[0, :])]], 'k-')

        # Plot the initial and last positions points for each UAV
        for i in range(len(initial_position_list)):
            plt.plot(initial_position_list[i][0], initial_position_list[i][1], '*')
            plt.plot(last_position_list[i][0], last_position_list[i][1], '*')

        # Plot the streets intersections with the area
        for point in streets_lines:
            plt.plot(point[0], point[1], 'ro')

        # Plot the path lines
        for i in range(len(path_waypoints) - 1):
            plt.plot([path_waypoints[i][0], path_waypoints[i + 1][0]],
                    [path_waypoints[i][1], path_waypoints[i + 1][1]],
                    'r')

        # Mark the initial and last positions of the path
        plt.plot(path_waypoints[0][0], path_waypoints[0][1], 'g*')
        plt.plot(path_waypoints[-1][0], path_waypoints[-1][1], 'b*')

        # Plot waypoints
        for point in path_waypoints:
            plt.plot(point[0], point[1], 'yo')

        plt.figure()

        # Plot the area
        area_np = np.array(area_values)
        for i in range(0, len(area_np)):
            plt.plot([area_np[i][0], area_np[(i + 1) % len(area_np)][0]],
                    [area_np[i][1], area_np[(i + 1) % len(area_np)][1]], 'b-')

        # Plot the path rotated
        for i in range(len(path_original_angle) - 1):
            plt.plot([path_original_angle[i][0], path_original_angle[i + 1][0]],
                    [path_original_angle[i][1], path_original_angle[i + 1][1]],
                    'r')

        # Plot each path waypoint
        for point in path_original_angle:
            plt.plot(point[0], point[1], 'yo')
        
        # Plot the initial and last positions points for each UAV
        for i in range(len(initial_position_list)):
            plt.plot(initial_position_list[i][0], initial_position_list[i][1], '*')
            plt.plot(last_position_list[i][0], last_position_list[i][1], '*')

        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

    return path_waypoints_height


if __name__ == '__main__':
    position = [0.0, 0.0, 10.0]

    initial_position_list = [
        position,
        position]

    last_position_list = initial_position_list

    area_height = 10.0
    area_values = [
        [  0.0,  0.0, area_height],
        [100.0,  0.0, area_height],
        [100.0, 50.0, area_height],
        [  0.0, 50.0, area_height]]

    street_spacing = 25.0
    waypoints_spacing = 20.0

    theta = 0.0

    path_waypoints_height = _test_generate_path(
            initial_position_list,
            last_position_list,
            area_values,
            street_spacing,
            waypoints_spacing,
            theta,
            True)
