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


global_robot_state = None
global_robot_moving = "None"


def protocom ():

    global global_robot_state
    global global_robot_moving

    # Protobuf Init
    global_robot_message = RobotMessage()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 5555))

    while True:

        print("protobuf - waiting to receive")
        
        
        # receive protobuf message
        data, addr = sock.recvfrom(2048)
        
        print("data: " + str(data))
        
        global_robot_message.ParseFromString(data)
        


        # split the info into global vars
        global_robot_state = global_robot_message.state
        global_robot_moving = global_robot_message.moving
        print("protobuf - received: " + str(global_robot_message.state) + " and " + str(global_robot_message.moving))


@uamethod
def move_robot_ros(parent,):
    print("in move robot ros")
    panda_movement_process = subprocess.Popen(["roslaunch niks_experiments Stretching.launch", ])

    return True

@uamethod
def start_ros(parent,):
    print("in start ros")
    source_devel_process = subprocess.Popen(["source /home/panda/libfranka/ws_moveit/devel/setup.bash", ])
    franka_control_process = subprocess.Popen(["roslaunch franka_control franka_control.launch", "robot_ip:=192.168.13.1"])
    time.sleep(5)
    franka_grippper_process = subprocess.Popen(["roslaunch franka_gripper franka_gripper.launch", "robot_ip:=192.168.13.1"])
    time.sleep(5)
    panda_moveit_process = subprocess.Popen(["roslaunch panda_moveit_config panda_moveit.launch", ])
    time.sleep(5)
    panda_rviz_process = subprocess.Popen(["roslaunch panda_moveit_config moveit_rviz.launch", ])
    time.sleep(5)
    panda_move_process = subprocess.Popen(["rosrun niks_experiments Stretching", ])


    return True

@uamethod
def move_robot_libfranka(parent, movement, place):
    print("in move robot libfranka")
    robot_ip = "192.168.13.1"
    print("in method: " + robot_ip , movement + place)
    robot_process = subprocess.Popen(["./kick_off_event_x", robot_ip, movement, place])

    return True

@uamethod
def move_robot_ros(parent, movement, place):
    robot_ip = "192.168.13.1"

    with open("/home/panda/libfranka/ws_moveit/OPCExchangeData.txt", "w") as f:
        f.write(movement + "," + place)

    return True



if __name__ == "__main__":

    global global_robot_state
    global global_robot_moving
    
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
    robot_object = objects.add_object(idx, "PandaRobot")

    # Parameters - Addresspsace, Name, Initial Value
    server_time = robot_object.add_variable(idx, "ServerTime", 0)
    mover_libfranka = robot_object.add_method(idx, "MoveRobotLibfranka", move_robot_libfranka, [ua.VariantType.String, ua.VariantType.String], [ua.VariantType.Boolean])
    mover_ros = robot_object.add_method(idx, "MoveRobotRos", move_robot_ros, [ua.VariantType.String, ua.VariantType.String], [ua.VariantType.Boolean])
    starter_ros = robot_object.add_method(idx, "StartRos", start_ros, [ ], [ua.VariantType.Boolean])
    robot_state = robot_object.add_variable(idx, "RobotState", "init")
    robot_moving = robot_object.add_variable(idx, "RobotMoving", False)
    # var = robot_object.add_variable(idx, "Var", 0.0)

    # Set parameters writable by clients
    # server_time.set_writable()

    # Start the server
    server.start()

    print("OPC-UA - Panda - Server started at {}".format(url))

    # Start protobuf communication
    protocom_thread = threading.Thread(name='protobuf_com_thread', target=protocom, args=())
    protocom_thread.daemon = True
    protocom_thread.start()


    try:
        # Assign random values to the parameters
        print("going into loop")

        while True:
            TIME = datetime.datetime.now()  # current time

            # set the random values inside the node
            robot_state.set_value(global_robot_state)

        
            if global_robot_moving == "true":
                robot_moving.set_value(True)
            elif global_robot_moving == "false":
                robot_moving.set_value(False)


            print("gloval_robot_moving is: " + global_robot_moving)
            #print("Robot-State: [" + str(global_robot_message.id) + "] : " + str(robot_state.get_value()) + ". Server-Time: " + str(server_time.get_value()))
            server_time.set_value(TIME)
            # var.set_value(var2)

            # sleep 2 seconds
            time.sleep(2)

    except KeyboardInterrupt:
        print("\nCtrl-C pressed. OPCUA - Pixtend - Server stopped at {}".format(url))

    finally:
        # close connection, remove subscriptions, etc
        if p != None:  # terminate robot process if there is one
            p.terminate()
        server.stop()
        conbelt = None

sys.exit(0)

