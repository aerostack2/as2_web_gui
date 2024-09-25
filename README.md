# Aerostack2 Web-GUI

## Requirements
Install dependencies:
```
pip install -r requirements.txt
```

## System Architecture

Detailed architecture of the Aerostack2 Web GUI is described in [Aerostack2 Web GUI Architecture](docs/README.md)

## Run Aerostack2 Graphical User Interface server
```
python3 manage.py runserver
```

## Open Webpage and aerostack2 interface using GPS
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
python3 as2_interface.py --uav_list drone0 drone1 --sim_mode true --use_sim_time true
```


## Open Webpage and aerostack2 interface using using cartesian coordinates
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
python3 as2_interface.py --uav_list drone0 drone1 --use_cartesian_coordinates true --sim_mode true --use_sim_time true
```

## Config Aerostack2 Graphical User Interface server
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

## Using Aerostack2

Remember to have the Aerostack2 running and the mission executor for each UAV.

```
ros2 run as2_python_api mission_executor --n drone0 --add_namespace
```