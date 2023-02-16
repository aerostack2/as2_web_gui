"""
as2_interface.py
"""

from as2_interface.aerostack_ui import AerostackUI

if __name__ == '__main__':
    aerostack_ui = AerostackUI(
        [
            'drone_sim_rafa_0'
        ],
        log_level=4,
        sim_mode=True,
        use_sim_time=True,
        use_cartesian_coordinates=False)

    aerostack_ui.shutdown()
