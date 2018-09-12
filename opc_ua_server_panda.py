#      _____         __        __                               ____                                        __  
#     / ___/ ____ _ / /____   / /_   __  __ _____ ____ _       / __ \ ___   _____ ___   ____ _ _____ _____ / /_ 
#     \__ \ / __ `// //_  /  / __ \ / / / // ___// __ `/      / /_/ // _ \ / ___// _ \ / __ `// ___// ___// __ \
#    ___/ // /_/ // /  / /_ / /_/ // /_/ // /   / /_/ /      / _, _//  __/(__  )/  __// /_/ // /   / /__ / / / /
#   /____/ \__,_//_/  /___//_.___/ \__,_//_/    \__, /      /_/ |_| \___//____/ \___/ \__,_//_/    \___//_/ /_/ 
#                                              /____/                                                           
# Salzburg Research ForschungsgesmbH
# Armin Niedermueller

# OPC UA Server on Panda
# The purpose of this OPCUA server is to provide methods to control the panda robot and read its state 

from opcua import ua, uamethod, Server
from dtz_robot_message_pb2 import RobotMessage 
import subprocess
import threading
import datetime
import socket
import time
import sys

sys.path.insert(0, "..")

robot_message = RobotMessage()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5555))


@uamethod
def move_robot(parent, movement, place):
    robot_ip = "192.168.13.1"
    print("in function")
    robot_process = subprocess.Popen(["./kick_off_event_x" , robot_ip, movement, place])
    print("receiving")
    data, addr = sock.recvfrom(1024)
    robot_message.ParseFromString(data)
    print("waiting in function for receive")
    return True

if __name__ == "__main__":

    # robot process object
    p = None

    # setup our server
    server = Server()
    url = "opc.tcp://0.0.0.0:4840/freeopcua/server"
    server.set_endpoint(url)

    # setup our own namespace
    uri = "https://github.com/iot-salzburg/dtz_panda"
    idx = server.register_namespace(uri)


    # get Objects node, this is where we should put our nodes
    objects = server.get_objects_node()

    # Add a parameter object to the address space
    object1 = objects.add_object(idx, "Object1")

    # Parameters - Addresspsace, Name, Initial Value
    server_time = object1.add_variable(idx, "ServerTime", 0)
    mover = object1.add_method(idx, "MoveRobot", move_robot, [ua.VariantType.String, ua.VariantType.String], [ua.VariantType.Boolean])
    robot_state = object1.add_variable(idx, "RobotState", "init")
    #var = object1.add_variable(idx, "Var", 0.0)

    # Set parameters writable by clients
    #server_time.set_writable()


    # Start the server
    server.start()



    print("OPC-UA - Panda - Server started at {}".format(url))

    try:
        # Assign random values to the parameters
        print("going into loop")
        
        while True:
            TIME = datetime.datetime.now()  # current time

            #with open("state.log") as f:
            #    state = f.read()
            #with open("distance.log") as f:
            #    distance = f.read()

            # set the random values inside the node
            robot_state.set_value(robot_message.state)
            print("Robot-State: [" + str(robot_message.id) + "] : " + str(robot_state.get_value()) + ". Server-Time: " + str(server_time.get_value()))
            server_time.set_value(TIME)
            #var.set_value(var2)

            # sleep 2 seconds
            time.sleep(2)
    except KeyboardInterrupt:
            print("\nCtrl-C pressed. OPCUA - Pixtend - Server stopped at {}".format(url))
    finally:
        #close connection, remove subcsriptions, etc
        if p != None:       #terminate robot process if there is one
            p.terminate()
        server.stop()
        conbelt = None
sys.exit(0)
