"""
as2_interface.py
"""

import argparse
from as2_interface.aerostack_ui import AerostackUI

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Aerostack UI')
    parser.add_argument('--uav_list', type=str, nargs='+', default=['drone0'], help='UAV namespaces list')
    parser.add_argument('--log_level', type=int, default=1, help='log_level (int): 0 = no logs, 1 = add error logs, 2 = add warnings logs, 3 = add info logs, 4 = add debug logs')
    parser.add_argument('--sim_mode', type=bool, default=False, help='Simulation mode. For set arm and offboard')
    parser.add_argument('--use_sim_time', type=bool, default=False, help='Use simulation time')
    parser.add_argument('--use_cartesian_coordinates', type=bool, default=False, help='Use cartesian coordinates')

    args = parser.parse_args()

    aerostack_ui = AerostackUI(
        args.uav_list,
        log_level=args.log_level,
        sim_mode=args.sim_mode,
        use_sim_time=args.use_sim_time,
        use_cartesian_coordinates=args.use_cartesian_coordinates)

    aerostack_ui.shutdown()
