from django.urls import path

from MapApp import views

urlpatterns = [
    # path('map/', views.map, name="Aerostack"),
    path('map/use_cartesian=<str:flag>/', views.map, name="Aerostack"), # examples: http://127.0.0.1:8000/map/True, http://127.0.0.1:8000/map/False
]