"""
as2_interface.py
"""

import rclpy
from as2_interface.aerostack_ui import AerostackUI

if __name__ == '__main__':
    rclpy.init()

    aerostackUI = AerostackUI(
        [
            'drone0',
            'drone1'
        ],
        verbose=True,
        sim_mode=False,
        use_sim_time=False)