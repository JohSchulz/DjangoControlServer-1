from django.views.generic import ListView, DetailView, TemplateView, View
from django.shortcuts import render, get_object_or_404
from django.http import HttpResponse
from django.core import serializers

from .models import laser
import telnetlib, json, subprocess, platform

# Create your views here.
class LaserDetailView(DetailView):
    model = laser
    slug_url_kwarg = 'laser_name'
    slug_field = 'name'
    template_name = 'laser/laser.html'
    
    def get_context_data(self, **kwargs):
        Laser = super().get_object()
        return { 'laser' : json.loads(serializers.serialize('json', [Laser]))[0]['fields'] }

class LaserControlView(View):
	model = laser
	
	def get(self, request, *args, **kwargs):
		Laser = get_object_or_404(self.model, name=kwargs['laser_name'])
		
def laser(request, slug):
	response = {}
	r_dict = json.loads( request.body.decode())
	print(r_dict)
	command = r_dict['command']

	if command == 'PING':
		ip = r_dict['payload']
		parameter = '-n' if platform.system().lower()=='windows' else '-c'
		command = ['ping', parameter, '1', ip]
		success = subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
	
		if success == 0 : response['message'] =  'Device ready.'
		else : response['message'] = 'Failed to connect.'

	if command == 'TOGGLE':
		toggle = r_dict['payload']
		print('toggle : ', toggle)
		os.system("apps\laser\static\laser\toggle_laser_diode.bat " + toggle)
		response['message'] = "Laser Diode " + toggle + "."
        
	if command == 'TOGGLE_EDFA':
		toggle = r_dict['payload']
		print('toggle_edfa')
		# os.system("apps\laser\static\laser\toggle_edfa.bat " + toggle)
		response['message'] = "EDFAs " + toggle + "."

	if command == 'UPDATE_EDFA':
		voltage = r_dict['payload'] * 0.01
		print(voltage)
		# os.system("apps\laser\static\laser\toggle_laser_diode.bat " + voltage)
		response['message'] = "Power parameter updated."

	return HttpResponse(json.dumps(response))
	
class telnet_command:
	
	def __init__(self, server, data={}):
		host = telnetlib.Telnet( server )
		self.__host = host
		
		self.__functionable = { 'write' : self.write,
					'exit' : host.close }
		
	def write(self, text):
		self.__host.write( '{}\n'.format( text ) )
	
