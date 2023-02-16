from swarm_pylib.swarm_algorithms_templates import DistributionAlgorithm
import numpy as np
from numpy import linalg as LA
from scipy.optimize import linear_sum_assignment


class Binpat(DistributionAlgorithm):

    @staticmethod
    def distribute_area(
            initial_position_list: list,
            last_position_list: list,
            path_waypoints: list,
            uavs_weight_list: list = None) -> list:
        """ Algorithm to distribute the waypoints of the area to each UAV
        Args:
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
        area_waypoints_uavs = Binpat.old_method(
            initial_position_list, last_position_list, path_waypoints, uavs_weight_list)
        return area_waypoints_uavs

    @staticmethod
    def old_method(
            initial_position_list_h: list,
            last_position_list_h: list,
            path_waypoints_h: list,
            uavs_weight_list: list = None) -> list:
        """ Algorithm to distribute the waypoints of the area to each UAV
        Args:
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

        path_height, path_waypoints, \
        initial_positions_height, initial_position_list, \
        last_positions_height, last_position_list = Binpat.data_input_process(
            path_waypoints_h, initial_position_list_h, last_position_list_h)

        assets, track_number, track_firsts, track_lasts, UAV_altitudes, final_cost = Binpat.binpat(
            initial_position_list, last_position_list, path_waypoints, uavs_weight_list, path_height)

        waypoints = Binpat.generate_waypoints(
            initial_position_list, path_waypoints, assets, track_number, track_firsts, track_lasts)

        # # Add height to waypoints
        uavs_path = []
        for i in range(len(waypoints)):
            positions_with_height = DistributionAlgorithm._add_height_to_position_list(
                waypoints[i], path_height)
            uavs_path.append(positions_with_height)
        return uavs_path

    @staticmethod
    def data_input_process(
            path_waypoints_h: list,
            initial_position_list_h: list,
            last_position_list_h: list) -> np.array:
        """ Process the input data to be used in the algorithm
        Args:
            path_waypoints (list): List of lists with x, y and height values of the path to
                cover the area
            initial_position_list (list): List of lists with x, y and height values of the initial
                position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last
                position of each UAV
        Returns:
            path_height (float): Height of the path
            path_waypoints (np.array): Numpy array with x and y values of waypoints
            initial_positions_height (list): List of heights of the initial positions
            initial_position_list (np.array): x and y values of the initial position of each UAV
            last_positions_height (list): List of heights of the last positions
            last_position_list (np.array): x and y values of the last position of each UAV
        """

        # Remove height from path waypoints
        position_list, height_list = DistributionAlgorithm._extract_height_from_list(
            path_waypoints_h)
        path_waypoints = np.array(position_list)
        path_height = height_list[0]

        # Remove height from initial position list
        position_list, height_list = DistributionAlgorithm._extract_height_from_list(
            initial_position_list_h)
        initial_position_list = np.array(position_list)
        initial_positions_height = height_list

        # Remove height from last position list
        position_list, height_list = DistributionAlgorithm._extract_height_from_list(
            last_position_list_h)
        last_position_list = np.array(position_list)
        last_positions_height = height_list

        return path_height, path_waypoints, initial_positions_height, initial_position_list, last_positions_height, last_position_list

    @staticmethod
    def binpat(
            initial_position_list,
            last_position_list,
            waypoints_grid,
            uavs_weight_list,
            mission_height):
        """ Distribute waypoints using binpat algorithm
        Args:
            initial_position_list (list): List of lists with x, y and height values of the initial position of each UAV
            last_position_list (list): List of lists with x, y and height values of the last position of each UAV
            waypoints_grid (list): List of lists with x, y and height values of the waypoints of the area
            uavs_weight_list (list): List of weights for each UAV.
        Returns:
            area_waypoints_uavs (list): List of lists with x, y and height values of the waypoints for each UAV
            uavs_cost (list): List of costs for each UAV
        """

        # Number of tracks between waypoints
        number_tracks_between_waypoints = len(waypoints_grid)-1

        # Number of UAVs
        number_uavs = len(initial_position_list)

        # Initialize matrices
        distribution_matrix = np.zeros((number_tracks_between_waypoints))
        uavs_weights = np.zeros((number_uavs))

        # Distribution matrix
        for i in range(number_tracks_between_waypoints):
            distribution_matrix[i] = LA.norm(
                waypoints_grid[i, :]-waypoints_grid[i+1, :])

        # UAVs weights
        for i in range(number_uavs):
            uavs_weights[i] = uavs_weight_list[i]*sum(distribution_matrix)

        # Compute bin pack algorithm
        tracks_assets, tracks_distribution_matrix = Binpat.bin_pack_algorithm(
            distribution_matrix,
            uavs_weights)

        tracks_firsts = np.zeros((number_uavs))
        tracks_lasts = np.zeros((number_uavs))
        tracks_individual = np.zeros((number_uavs))

        for i in range(0, number_uavs):
            iterator = filter(lambda x: (x >= 0), tracks_assets[i, :])
            filtered = list(iterator)
            tracks_individual[i] = len(filtered)  # Numero de pistas
            if tracks_individual[i] > 0:
                tracks_firsts[i] = min(filtered)
                tracks_lasts[i] = max(filtered)

        # Short distribution matrix, getting indexes of tracks
        cost_matrix = np.zeros((number_uavs, number_uavs))

        for i in range(0, number_uavs):

            for j in range(0, number_uavs):

                arrive_cost = abs(LA.norm(
                    waypoints_grid[int(tracks_firsts[j]), :]-initial_position_list[i, :]))  # Initial position
                return_cost = abs(
                    LA.norm(waypoints_grid[int(tracks_lasts[j]), :]-last_position_list[i, :]))  # Last position
                mission_cost = abs(tracks_distribution_matrix[j])
                # up_down_delay = abs(
                #     2*altitude_matrix[i]+2*(altitude_matrix[i]-mission_altitude))
                # print (mission_cost)
                # +up_down_delay /UAV_weights[i]
                cost_matrix[i, j] = (arrive_cost+return_cost+mission_cost)

        row, asset_col = linear_sum_assignment(cost_matrix)
        first_cost = cost_matrix[row, asset_col]

        altitude_matrix = Binpat.assign_uav_altitudes(first_cost, mission_height)
        cost = np.zeros_like(first_cost)
        for i in range(0, len(cost)):
            cost[i] = first_cost[i] + \
                abs(2*altitude_matrix[i]+2*(altitude_matrix[i]-mission_height))

        return asset_col, tracks_individual, tracks_firsts, tracks_lasts, altitude_matrix, cost

    @staticmethod
    def bin_pack_algorithm(
            distribution_matrix,
            uavs_weights):
        track_number = len(distribution_matrix)
        bin_number = len(uavs_weights)

        bins = np.zeros((bin_number, track_number))
        index_mat = np.ones((bin_number, track_number))*-1
        weight_sum_mat = np.zeros((bin_number))
        # track_mean=V_total/len(weights)
        count = 0
        bin_count = 0
        weight_sum = 0
        track_cont = 0
        V_bin_max = uavs_weights[count]
        previous_add_result = V_bin_max
        # previous_add_result=V_bin_max
        for item, weight in enumerate(distribution_matrix):
            # previous_add_result=bin_weights[count]
            # Aqui sumamos el peso al anadir una nueva pista a la lista del UAV
            new_weight_sum = weight+weight_sum
            # El valor que me falta o sobra al anadir la pista
            current_add_result = abs(V_bin_max-new_weight_sum)

            # Evaluar si al anadir la nueva pista estamos mas lejos que al anadir la anterior
            if current_add_result > previous_add_result:
                count = count+1
                if count > bin_number-1:
                    count = bin_number-1
                V_bin_max = uavs_weights[count]
                # print("V_bin_max for: %d" % (V_bin_max))
                new_weight_sum = weight  # actualizar el valor de new weight sum
                # bin_count=bin_count+1 #saltar al siguiente UAV

            bins[count, item] = weight
            index_mat[count, item] = item
            weight_sum_mat[count] = sum(bins[count, :])

            weight_sum = new_weight_sum
            previous_add_result = abs(V_bin_max-new_weight_sum)

        return index_mat, weight_sum_mat

    @staticmethod
    def assign_uav_altitudes(cost_matrix, mission_altitude):
        aux_cost_matrix = np.zeros(len(cost_matrix))
        altitude_matrix = np.zeros(len(aux_cost_matrix))

        for i in range(0, len(cost_matrix)):
            aux_cost_matrix[i] = cost_matrix[i]

        for i in range(0, len(aux_cost_matrix)):
            index = np.argmax(aux_cost_matrix)
            current_altitude = mission_altitude+5+(5*i)  # Heigh of each UAV
            altitude_matrix[index] = current_altitude
            aux_cost_matrix[index] = -1
            # print(cost_matrix)
        # print(altitude_matrix)
        return altitude_matrix

    @staticmethod
    def generate_waypoints(UAV, V, assets, track_number_matrix, track_first, track_last):
        UAVnumber = len(UAV)
        waypoint_number = len(V)
        waypoint_mat = np.ones((UAVnumber, waypoint_number, 2))*float("NaN")
        waypoint_mat_final = np.ones(
            (UAVnumber, waypoint_number, 2))*float("NaN")
        # First waypoint is UAV initial position (altitude change)

        # print(V)
        # print (V_points)

        count = 1

        new_track = 0
        aux_start = 0

        for i, track in enumerate(track_number_matrix):

            aux_it = int(new_track+track+1)

            for j in range(int(aux_start), aux_it):
                waypoint_mat[i, int(j-aux_start), :] = V[j, :]

            new_track = new_track+track
            aux_start = new_track+1

        # print(waypoint_mat)

        for i in range(0, UAVnumber):
            waypoint_mat_final[i, :, :] = waypoint_mat[assets[i], :, :]

        # Filter NaN values
        waypoint_mat_final_filtered = []
        for i in range(0, UAVnumber):
            waypoint_mat_final_filtered.append(
                waypoint_mat_final[i, ~np.isnan(waypoint_mat_final[i, :, 0]), :])

        return waypoint_mat_final_filtered


def _test_distribute_area(
        initial_position_list: list,
        last_position_list: list,
        path_waypoints: list,
        uavs_weight_list: list = None,
        plot: bool = False) -> list:
    """ Algorithm to distribute the waypoints of the area to each UAV
    Args:
        initial_position_list (list): List of lists with x, y and height values of the initial
            position of each UAV
        last_position_list (list): List of lists with x, y and height values of the last
            position of each UAV
        path_waypoints (list): List of lists with x, y and height values of the path to
            cover the area
        uavs_weight_list (list): List of weights for each UAV. Default 'None' is equal weights
            for all UAVs
        plot (bool): If True, the path of each UAV is plotted. Default 'False'
    Returns:
        uavs_path (list): List of lists with x, y and height values of the path for each UAV
    """
    uavs_path = Binpat.distribute_area(
        initial_position_list,
        last_position_list,
        path_waypoints,
        uavs_weight_list)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure()

        # Plot initial and final positions
        for i, uav_pos in enumerate(initial_position_list):
            plt.plot(uav_pos[0], uav_pos[1], 'ro')
            plt.text(uav_pos[0], uav_pos[1], "UAV %d" % (i))
        
        # Plot uav waypoints
        for uav_path in uavs_path:
            for waypoint in uav_path:
                plt.plot(waypoint[0], waypoint[1], 'bo')
        
        # Plot path line
        for uav_path in uavs_path:
            x = [waypoint[0] for waypoint in uav_path]
            y = [waypoint[1] for waypoint in uav_path]
            plt.plot(x, y, '-')

        plt.legend()
        plt.show()

    return uavs_path


if __name__ == "__main__":
    position = [0.0, 0.0, 10.0]

    initial_position_list = [
        position,
        position]

    last_position_list = initial_position_list

    uavs_weight_list = [0.5, 0.5]

    path_waypoints = [[0.0, 5.0, 10.0], [2.0, 5.0, 10.0], [4.0, 5.0, 10.0], [6.0, 5.0, 10.0], [8.0, 5.0, 10.0], [10.0, 5.0, 10.0], [10.0, 3.75, 10.0], [10.0, 2.5, 10.0], [8.0, 2.5, 10.0], [
        6.0, 2.5, 10.0], [4.0, 2.5, 10.0], [2.0, 2.5, 10.0], [0.0, 2.5, 10.0], [0.0, 1.25, 10.0], [0.0, 0.0, 10.0], [2.0, 0.0, 10.0], [4.0, 0.0, 10.0], [6.0, 0.0, 10.0], [8.0, 0.0, 10.0]]

    _test_distribute_area(
        initial_position_list,
        last_position_list,
        path_waypoints,
        uavs_weight_list,
        True)
