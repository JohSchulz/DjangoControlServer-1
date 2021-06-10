from django.urls import path
from . import views

app_name = 'pd_monitor'
urlpatterns = [
	path('', views.IndexView.as_view(), name='index'),
	path('<int:pk>/', views.DetailView.as_view(), name='detail'),
	path('jstest/', views.jstest, name='jstest'),
	path('json/', views.json, name='json')
]
