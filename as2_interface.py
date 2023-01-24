"""
as2_interface.py
"""

import rclpy
from as2_interface.aerostack_ui import AerostackUI

if __name__ == '__main__':
    rclpy.init()

    aerostackUI = AerostackUI(
        ['drone_sim_rafa_0'],
        verbose=True,
        sim_mode=True,
        use_sim_time=True)
