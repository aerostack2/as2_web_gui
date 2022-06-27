import numpy as np
from matplotlib import path
import math
from numpy import linalg as LA
import scipy
from scipy.optimize import linear_sum_assignment


def compute_area(
        UAV_initial_position,
        UAV_last_position,
        uav_weight,
        area,
        altitude,
        street_spacing,
        wpt_separation,
        path_algorithm='Back and force',
        distribution_algorithm=""):

    if path_algorithm == 'Back and force':
        V, wpt_grid = back_and_forth(
            area, street_spacing, wpt_separation, UAV_initial_position)
    else:
        raise Exception("Unknown path_algorithm")

    import time
    t = time.time()
    if distribution_algorithm == "binpat":
        # Assign tracks
        assets, track_number, track_firsts, track_lasts, UAV_altitudes, final_cost = binpat(
            wpt_grid, UAV_initial_position, UAV_last_position, uav_weight, altitude)  # Track asset

    elif distribution_algorithm == "powell_binpat":
        assets, track_number, track_firsts, track_lasts, UAV_altitudes, final_cost = powell_binpat(
            wpt_grid, UAV_initial_position, UAV_last_position, uav_weight, altitude)

    else:
        raise Exception("Unknown distribution_algorithm")

    elapsed = time.time() - t
    print('Elapsed time: %f' % (elapsed))

    # Generate relative waypoints
    waypoints = generate_waypoints3(
        UAV_initial_position, wpt_grid, assets, track_number, track_firsts, track_lasts)  # Relative waypoints

    return waypoints, wpt_grid


# region Back and forth

def split(start, end, segments):
    x_delta = (end[0] - start[0]) / float(segments)
    y_delta = (end[1] - start[1]) / float(segments)
    ini = np.zeros((1, 2))
    ini[0, :] = [start[0], start[1]]
    fin = np.zeros((1, 2))
    fin[0, :] = [end[0], end[1]]
    points = np.zeros((segments, 2))
    #points = []

    for i in range(0, segments):
        points[i, :] = [start[0] + i * x_delta, start[1] + i * y_delta]

    points = np.delete(points, 0, axis=0)

    # print(start)
    points_b = np.concatenate((ini, points), axis=0)
    points_c = np.concatenate((points_b, fin), axis=0)
    return points_c


def back_and_forth(points, spacing, separation, UAV):

    # Track generation
    x = points[:, 0]
    y = points[:, 1]

    areaWidth = max(x)-min(x)
    thetamin = 0

    # Get optimal direction
    for i in range(1, 360):
        theta = i*2*math.pi/360
        R = np.array([[math.cos(theta), -math.sin(theta)],
                     [math.sin(theta), math.cos(theta)]])
        aux = R.dot(np.transpose(points))
        if (max(aux[0, :])-min(aux[0, :])) < areaWidth:
            areaWidth = max(aux[0, :])-min(aux[0, :])
            thetamin = theta

    R = np.array([[math.cos(thetamin), -math.sin(thetamin)],
                 [math.sin(thetamin), math.cos(thetamin)]])
    aux = R.dot(np.transpose(points))
    x = np.transpose(aux[0, :])
    y = np.transpose(aux[1, :])

    areaWidth = max(x)-min(x)
    areaLength = max(y)-min(y)
    numberOfLanes = math.ceil(areaWidth/spacing)

    laneDist = areaWidth/(int(numberOfLanes))
    lanemin = np.zeros((int(numberOfLanes), 2))
    lanemax = np.zeros((int(numberOfLanes), 2))

    poly_points = np.zeros((x.size, 2))

    for i, val in enumerate(x):
        poly_points[i] = (x[i], y[i])

    p = path.Path(poly_points)
    # p = poly_points

    for i in range(1, int(numberOfLanes+1)):
        xi = min(x)+laneDist*i-laneDist/2
        delta = 0.1
        k = 0
        miny = min(y) + k*delta
        while not(p.contains_points([(xi, miny)])):
            miny = min(y)+k*delta
            k = k+1

        k = 0
        maxy = max(y) - k*delta
        while not(p.contains_points([(xi, maxy)])):
            maxy = max(y)-k*delta
            k = k+1

        lanemin[i-1] = (xi, miny)
        lanemax[i-1] = (xi, maxy)

    lmin = np.transpose(np.transpose(R).dot(np.transpose(lanemin)))
    lmax = np.transpose(np.transpose(R).dot(np.transpose(lanemax)))

    V = np.zeros((int(numberOfLanes*2), 2))
    for i in range(0, int(numberOfLanes*2)):

        if(i % 2) == 0:
            V[i, :] = lmax[int(i/2), :]
        else:
            V[i, :] = lmin[int((i-1)/2), :]

    # Intermediate waypoints

    # first--decide the best initial point to generate tracks based on less V[0,:] or V[1,:] distance
    ini_mat = np.zeros((2, len(UAV)))
    for i in range(0, len(UAV)):
        ini_mat[0, i] = LA.norm(V[0, :]-UAV[i, :])
        # TODO: Ver si afecta a last point
        ini_mat[1, i] = LA.norm(V[1, :]-UAV[i, :])

    result_min = np.where(ini_mat == np.amin(
        ini_mat))  # find the minimum distance
    prev_res = result_min[0][0]
    dist = LA.norm(V[0, :]-V[1, :])  # first track distance
    split_val = int(dist/separation)
    if split_val < 1:
        split_val = 1
    if(prev_res == 0):  # if minimum distance is for V0 start from V0
        prev_array = array = split(V[0, :], V[1, :], split_val)
    else:
        prev_array = array = split(V[1, :], V[0, :], split_val)
    # plt.scatter(V[:,0],V[:,1])

    for i in range(1, int(len(V)/2)):
        dist = LA.norm(V[(2*i), :]-V[(2*i)+1, :])
        split_val = int(dist/separation)
        if split_val < 1:
            split_val = 1  # al menos dos waypoints en una calle
        if (prev_res == 0):
            current_array = split(
                V[(2*i)+1, :], V[(2*i), :], split_val)  # now change
            prev_res = 1
        else:
            current_array = split(V[(2*i), :], V[(2*i)+1, :], split_val)
            prev_res = 0
        final = np.concatenate((prev_array, current_array), axis=0)
        prev_array = final

    # print(list_wpt)

    return V, final

# endregion

# region Binpat


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


def bin_pack_alg2(track_weights, bin_weights):
    track_number = len(track_weights)
    bin_number = len(bin_weights)

    bins = np.zeros((bin_number, track_number))
    index_mat = np.ones((bin_number, track_number))*-1
    weight_sum_mat = np.zeros((bin_number))
    # track_mean=V_total/len(weights)
    count = 0
    bin_count = 0
    weight_sum = 0
    track_cont = 0
    V_bin_max = bin_weights[count]
    previous_add_result = V_bin_max
    # previous_add_result=V_bin_max
    for item, weight in enumerate(track_weights):
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
            V_bin_max = bin_weights[count]
            #print("V_bin_max for: %d" % (V_bin_max))
            new_weight_sum = weight  # actualizar el valor de new weight sum
            # bin_count=bin_count+1 #saltar al siguiente UAV

        bins[count, item] = weight
        index_mat[count, item] = item
        weight_sum_mat[count] = sum(bins[count, :])

        weight_sum = new_weight_sum
        previous_add_result = abs(V_bin_max-new_weight_sum)

    return index_mat, weight_sum_mat


def track_asset4(tracks, UAV_ini_pos, UAV_last_pos, UAV_ini_weights, mission_altitude):
    numberoftracks = len(tracks)-1
    UAVnumber = len(UAV_ini_pos)
    C = np.zeros((UAVnumber, numberoftracks))
    dist_mat = np.zeros((numberoftracks))
    UAV_weights = np.zeros((UAVnumber))

   # flow_matrix
    for i in range(0, numberoftracks):
        dist_mat[i] = LA.norm(tracks[i, :]-tracks[i+1, :])

    for i in range(0, UAVnumber):
        UAV_weights[i] = UAV_ini_weights[i]*sum(dist_mat)

    track_asset, track_dist_mat = bin_pack_alg2(dist_mat, UAV_weights)
    #print (dist_mat)
    #print (UAV_weights)
    #print (track_asset)

    track_firsts = np.zeros((UAVnumber))
    track_lasts = np.zeros((UAVnumber))
    individual_track_number = np.zeros((UAVnumber))

    for i in range(0, UAVnumber):
        iterator = filter(lambda x: (x >= 0), track_asset[i, :])
        filtered = list(iterator)
        individual_track_number[i] = len(filtered)  # Numero de pistas
        if individual_track_number[i] > 0:
            track_firsts[i] = min(filtered)
            track_lasts[i] = max(filtered)

    # print(track_firsts)
    # print(track_lasts)

    # ordenar matriz de track_dist, obtener los indices y asignar las alturas
    # las alturas tambien deben considerar la distancia al punto para que funcione.
    #print (track_dist_mat)
    # altitude_matrix = assign_uav_altitudes(track_dist_mat, mission_altitude)
    # print(altitude_matrix)
    C = np.zeros((UAVnumber, UAVnumber))
    #print (track_dist_mat)
    for i in range(0, UAVnumber):

        for j in range(0, UAVnumber):
            arrive_cost = abs(
                LA.norm(tracks[int(track_firsts[j]), :]-UAV_ini_pos[i, :]))  # Initial position
            return_cost = abs(
                LA.norm(tracks[int(track_lasts[j]), :]-UAV_last_pos[i, :]))  # Last position
            mission_cost = abs(track_dist_mat[j])
            # up_down_delay = abs(
            #     2*altitude_matrix[i]+2*(altitude_matrix[i]-mission_altitude))
            #print (mission_cost)
            # +up_down_delay /UAV_weights[i]
            C[i, j] = (arrive_cost+return_cost+mission_cost)

    row, asset_col = linear_sum_assignment(C)
    first_cost = C[row, asset_col]

    altitude_matrix = assign_uav_altitudes(first_cost, mission_altitude)
    cost = np.zeros_like(first_cost)
    for i in range(0, len(cost)):
        cost[i] = first_cost[i] + \
            abs(2*altitude_matrix[i]+2*(altitude_matrix[i]-mission_altitude))

    return asset_col, individual_track_number, track_firsts, track_lasts, altitude_matrix, cost


def binpat(wpt_grid, UAV_ini_pos, UAV_last_pos, uav_weight, altitude):
    return track_asset4(
        wpt_grid, UAV_ini_pos, UAV_last_pos, uav_weight, altitude)  # Track asset


# region Powell Binpat

def fitness_func(uav_weight, wpt_grid, UAV_ini_pos, UAV_last_pos, altitude):
    # Calculating the fitness value of each solution in the current population.
    assets, track_number, track_firsts, track_lasts, UAV_altitudes, final_cost = binpat(
        wpt_grid, UAV_ini_pos, UAV_last_pos, uav_weight, altitude)
    fitness = max(final_cost)
    return fitness


def powell_binpat(wpt_grid, UAV_ini_pos, UAV_last_pos, uav_weight, altitude):
    res = scipy.optimize.minimize(
        fitness_func, uav_weight, args=(wpt_grid, UAV_ini_pos, UAV_last_pos, altitude), method='Powell', tol=1e-6)

    # Track asset with best fit
    return track_asset4(wpt_grid, UAV_ini_pos, UAV_last_pos, res.x, altitude)

# endregion

# endregion

# region Generate waypoints


def generate_waypoints3(UAV, V, assets, track_number_matrix, track_first, track_last):
    UAVnumber = len(UAV)
    waypoint_number = len(V)
    waypoint_mat = np.ones((UAVnumber, waypoint_number, 2))*float("NaN")
    waypoint_mat_final = np.ones((UAVnumber, waypoint_number, 2))*float("NaN")
    # First waypoint is UAV initial position (altitude change)

    # print(V)
    #print (V_points)

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

    return waypoint_mat_final

# endregion

