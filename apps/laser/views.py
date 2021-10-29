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
    
    def get(self, request, *args, **kwargs): # Function for monitoring ?
        Laser = get_object_or_404(self.model, name=kwargs['laser_name'])
        return HttpResponse(json.dumps({ 'message' : 'LOL' }))
        
    def post(self, request, **kwargs):
        Laser = get_object_or_404(self.model, name=kwargs['laser_name'])
        r_dict = json.loads(request.body.decode())
        command = r_dict['command']; arg = r_dict['payload']
        response = { 'success' : True }
        
        if command == 'PING':
            ip = Laser.ip
            parameter = '-n' if platform.system().lower()=='windows' else '-c'
            command = ['ping', parameter, '1', ip]
            success = subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
	
            if success == 0 : response['message'] =  'Device ready.'
            else : 
                response['message'] = 'Failed to connect.'
                response['success'] = False
        
        else :

            try : session = telnet_command(Laser.ip)
            except Exception as excp :
                response['message'] = str(excp)
                response['success'] = False
        
            else :
        
                if command == 'TOGGLE':
                    print('toggle : ', arg)
                    session.toggle_LD(arg)
                    response['message'] = "Laser Diode " + arg + "."
        
                elif command == 'TOGGLE_EDFA':
                    print('toggle_edfa')
                    session.toggle_edfa(arg)
                    response['message'] = "EDFAs " + arg + "."

                elif command == 'SET_EDFA':
                    voltage = arg * 0.01
                    print(voltage)
                    response['message'] = "Power parameter updated."
            
                else : 
                    response['message'] = "Bad request."
                    response['success'] = False 

        return HttpResponse(json.dumps(response))
	
class telnet_command:
    
    def __init__(self, server, data={}):
        host = telnetlib.Telnet( server )
        self.__host = host
        
    def write(self, text):
        self.__host.write( '{}\n'.format( text ) )

    def toggle_LD(self, toggle):
       	self.write('l_tool Enable_Current_Laser_Diode ' + toggle)
    
    def toggle_edfa(self, toggle):
       	self.write('l_tool edfa_shutdown edfa1')
       	self.read_until('# ')
       	self.write('l_tool edfa_shutdown edfa0')
    
    def set_edfa(self, setpoint):
       	self.write('l_tool edfa_set_phdout edfa1 ' + setpoint)
