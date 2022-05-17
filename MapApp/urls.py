from django.urls import path

from MapApp import views

urlpatterns = [
    path('map/', views.map, name="Aerostack"),
]