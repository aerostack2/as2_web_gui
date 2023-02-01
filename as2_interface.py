"""
as2_interface.py
"""

from as2_interface.aerostack_ui import AerostackUI

if __name__ == '__main__':
    aerostack_ui = AerostackUI(
        [
            'drone_sim_rafa_0',
            'drone_sim_rafa_1'
        ],
        log_level=4,
        sim_mode=False,
        use_sim_time=False)

    # aerostack_ui.shutdown()
