"""
as2_interface.py
"""

import rclpy
from as2_interface.aerostack_ui import AerostackUI

if __name__ == '__main__':
    rclpy.init()

    aerostackUI = AerostackUI([
        'drone0'
    ])
