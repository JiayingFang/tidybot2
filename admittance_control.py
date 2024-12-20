import time
import socket
import numpy as np
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, Common_pb2
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport
from kortex_api.autogen.messages import ActuatorCyclic_pb2, ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2, Common_pb2, ControlConfig_pb2, Session_pb2

def is_key_pressed(key):
    events = inputs.get_key()
    for event in events:
        if event.ev_type == inputs.KEYDOWN and event.ev_device == key:
            return True
    return False

# Connection parameters
IP_ADDRESS = "192.168.1.10" 
PORT = 10000 
USERNAME = "admin" 
PASSWORD = "admin" 

# Connect to the robot
transport = TCPTransport()
router = RouterClient(transport, RouterClient.basicErrorCallback)
transport.connect(IP_ADDRESS, PORT)
session_info = Session_pb2.CreateSessionInfo()
session_info.username = 'admin'
session_info.password = 'admin'
session_info.session_inactivity_timeout = 10000   # (milliseconds)
session_info.connection_inactivity_timeout = 2000 # (milliseconds)
session = SessionManager(router)
session.CreateSession(session_info)

# Initiate base
base = BaseClient(router)

# Clear faults
if base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
    base.ClearFaults()
    while base.GetArmState().active_state != Base_pb2.ARMSTATE_SERVOING_READY:
        time.sleep(0.1)

# Initiate admittance control
admittance = Base_pb2.Admittance()
admittance.admittance_mode = Base_pb2.JOINT
base.SetAdmittance(admittance)

# Stream data to client
server_ip = "172.24.69.38"
server_port = 5555
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_ip, server_port))
server_socket.listen(1)
print("Waiting for client connection...")
client_socket, client_address = server_socket.accept()
print(f"Connected to {client_address}")
while True:
    feedback = base.GetMeasuredCartesianPose()
    data = np.array([feedback.x, feedback.y, feedback.z, feedback.theta_x, feedback.theta_y, feedback.theta_z]).copy()
    print('cartesian pose', data)
    client_socket.sendall(data)
    time.sleep(0.1)
