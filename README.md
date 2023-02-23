# AerostackUI

## Requirements
Install dependencies:
```
pip install -r requirements.txt
```

## Run webpage user interface
```
python3 manage.py runserver
```

## Run server and aerostack2 interface using GPS
Open webpage:
```
http://127.0.0.1:8000/interface/map/use_cartesian=False/
```

Run Aerostack communication:
```
python3 as2_interface.py --uav_list drone0 drone1
```

If simulation mode, run:
```
python3 as2_interface.py --uav_list drone0 drone1 --simulation_mode true --use_sim_time true
```


## Run server and aerostack2 interface using cartesian coordinates
Open webpage:
```
http://127.0.0.1:8000/interface/map/use_cartesian=True/
```

Run Aerostack communication:
```
python3 as2_interface.py --uav_list drone0 drone1 --use_cartesian_coordinates true
```

If simulation mode, run:
```
python3 as2_interface.py --uav_list drone0 drone1 --simulation_mode true --use_sim_time true
```

## Config Web Interface
In `MapApp/static/Config_files` user web interface parameters can be configured.

In `Config/` the following parameters can be configured:
- `markers.json`: Icons to be draw on the map, such as UAVs, waypoints, etc.
- `mission.json`: Mission color order.
- `uav.json`: UAV color order.
- `websocket.json`: URL of the websocket connection.

In `Config_gps/` the following parameters can be configured for GPS coordinates usage, and in `Config_cartesian/` for cartesian coordinates usage:
- `global.json`: Map configuration, such as map center, zoom, map source, etc.
- `layers.json`: Map layers configuration, setting default color and add parameters to be configured such as swarm algorithm.
- `sidebars.json`: Sidebars configuration, settings default parameters values.
