# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Rafael Pérez Seguí'

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
