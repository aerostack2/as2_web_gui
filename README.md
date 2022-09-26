# AerostackUI

Clone repository:
```
cd ${AEROSTACK2_PATH}/stack/user_interface/
https://github.com/aerostack2-developers/AerostackUI.git
```

Install dependencies:
```
cd ${AEROSTACK2_PATH}/stack/user_interface/AerostackUI
chmod +x install_dependencies.bash
./install_dependencies.bash
```

Run webpage user interface at http://127.0.0.1:8000/interface/map/:
```
cd ${AEROSTACK2_PATH}/stack/user_interface/AerostackUI
python manage.py runserver
```

Run Aerostack communication:
```
cd ${AEROSTACK2_PATH}/stack/user_interface/AerostackUI/ROS
python AerostackUI.py
```
