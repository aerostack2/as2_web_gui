from django.shortcuts import render, HttpResponse
from django.template.loader import get_template

# Create your views here.

# MAP
def map(request):
    return render(request, 'map.html')