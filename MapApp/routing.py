from django.urls import include, re_path
from MapApp.consumers import CLientWebsocket

websocket_urlpatterns = [
    re_path('ws/user/', CLientWebsocket.as_asgi()),
]