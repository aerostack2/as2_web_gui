from django.urls import include, re_path
from MapApp.consumers import CLientWebsocket

websocket_urlpatterns = [
    re_path(r'^ws/play/(?P<room_code>\w+)/$', CLientWebsocket.as_asgi()),
]
