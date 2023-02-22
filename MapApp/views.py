from django.shortcuts import render, HttpResponse
from django.template.loader import get_template
from django.template.response import TemplateResponse

# Create your views here.

# MAP
# def map(request):
#     return render(request, 'map.html')
def map(request, flag="false"):
    return render(request, 'map.html', {'use_local_coordinates': flag})