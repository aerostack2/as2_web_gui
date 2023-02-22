# AerostackUI

Install dependencies:
```
pip install -r requirements.txt
```

# Run webpage user interface:
```
python3 manage.py runserver
```

# Run server and aerostack2 interface using GPS
Open webpage:
```
http://127.0.0.1:8000/interface/map/use_local_coordinates=False/
```

Run Aerostack communication:
```
python3 as2_interface.py --uav_list drone0 drone1
```


# Run server and aerostack2 interface using cartesian coordinates
Open webpage:
```
http://127.0.0.1:8000/interface/map/use_local_coordinates=True/
```

Run Aerostack communication:
```
python3 as2_interface.py --uav_list drone0 drone1 --use_cartesian_coordinates true
```


