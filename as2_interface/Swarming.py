import SwarmingLib as swarm
import numpy as np


area = np.array([
    [-62.09677419,  13.2034632],
    [-31.04838710, -16.01731602],
    [17.74193548, -27.92207792],
    [44.75806452,  26.73160173],
    [15.32258065,  61.9047619],
    [-36.29032258,  53.24675325],
])

UAV_initial_position = np.array([
    [-91.93548387, -38.2034632],
    [-83.06451613, -51.19047619],
    [-72.98387097, -66.34199134],
    [-60.48387097, -79.87012987],
])

UAV_last_position = np.array([
    [-102.93548387, -49.2034632],
    [-93.06451613, -62.19047619],
    [-83.98387097, -77.34199134],
    [-71.48387097, -90.87012987],
])


# Weights
vel_input = np.full(len(UAV_initial_position), 1)
vel_sum = sum(vel_input)
uav_weight = np.zeros_like(vel_input, dtype=float)
for i in range(0, len(vel_input)):
    uav_weight[i] = float(vel_input[i])/vel_sum

# Generate tracks
print("Binpat")
waypoints, wpt_grid = swarm.compute_area(
    UAV_initial_position,
    UAV_last_position,
    uav_weight,
    area,
    altitude=40,
    street_spacing=10,
    wpt_separation=20,
    path_algorithm="back_and_force",
    distribution_algorithm="binpat",
)

x = area[:, 0]
y = area[:, 1]

print("Powell binpat")
waypoints, wpt_grid = swarm.compute_area(
    UAV_initial_position,
    UAV_last_position,
    uav_weight,
    area,
    altitude=40,
    street_spacing=10,
    wpt_separation=20,
    path_algorithm="back_and_force",
    distribution_algorithm="powell_binpat",
)

x = area[:, 0]
y = area[:, 1]