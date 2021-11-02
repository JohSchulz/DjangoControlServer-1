from django.views.generic import ListView, DetailView, TemplateView, View
from django.shortcuts import render, get_object_or_404
from django.http import HttpResponse
from django.core import serializers

from .models import laser
import telnetlib, json, subprocess, platform

import numpy as np
import time

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
	
            if success == 0 :
                response['laser'] = True
                session = telnetlib.Telnet(Laser.ip)        # start telnet session
                
                session.write(b'ls_tool cplot\n')           # check status of laser
                time.sleep(3)                               # wait for the output
                session.write(b'\x1b \x5d \x34')
                session.write(b'\x03')                      # quit the cplot...
                session.write(b'exit\n')
                output = session.read_all().decode('ascii')
                print(output)
                status = output.find('ON')
                if status != -1 :
                    response['message'] =  'Laser diode ON.'
                else :
                    response['message'] =  'Laser diode OFF.'
                session.close()                             # clear session to save resources
            else : 
                response['message'] = 'Failed to connect.'
                response['laser'] = False
        
        else :

            try : 
                print('session')
                session = telnetlib.Telnet(Laser.ip)
            except Exception as excp :
                response['message'] = str(excp)
                response['success'] = False
        
            else :
        
                if command == 'TOGGLE':
                    session.write(b'ls_tool Enable_Current_Laser_Diode' + bytes(arg +'\n', 'ascii'))
                    response['message'] = 'Laser Diode ' + arg + '.'
        
                elif command == 'TOGGLE_EDFA':
                    print('toggle_edfa')
                    session.toggle_edfa(arg)
                    response['message'] = 'EDFAs ' + arg + '.'

                elif command == 'SET_EDFA':
                    voltage = ( np.log( float(arg) + 0.6 ) + 0.5 ) / 0.35
                    print(voltage)
                    session.set_edfa(str(voltage))
                    response['message'] = 'Power parameter updated.'
            
                else : 
                    response['message'] = 'Bad request.'
                    response['success'] = False 
                    
                session.write(b'exit\n')
                print(session.read_all().decode('ascii'))
                session.close() # clear session to save resources
                
        return HttpResponse(json.dumps(response))
	
class telnet_command:
    def __init__(self, server, data={}):
        try : 
            host = telnetlib.Telnet(server)
            self.__host = host
        except Error as err : 
            print(err)
            return false
            
    def write(self, text):
        try : 
            self.__host.write(bytes('{}\n'.format(text), 'utf-8'))
        except OSError as err : 
            print(err)
            return false

    def toggle_LD(self, toggle):
        try :
            self.write('l_tool Enable_Current_Laser_Diode ' + toggle)
        except OSError as err : 
        	print(err)
        	return false
    
    def toggle_edfa(self, toggle):
        try :
            self.write('l_tool edfa_shutdown edfa1')
            self.read_until('# ')
            self.write('l_tool edfa_shutdown edfa0')
        except OSError as err : 
            print(err)
            return false
    
    def set_edfa(self, setpoint):
        try : 
            self.write('l_tool edfa_set_phdout edfa1 ' + setpoint)
        except OSError as err :
            print(err)
            return false