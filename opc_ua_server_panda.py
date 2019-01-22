#!/usr/bin/env python
#      _____         __        __                               ____                                        __
#     / ___/ ____ _ / /____   / /_   __  __ _____ ____ _       / __ \ ___   _____ ___   ____ _ _____ _____ / /_
#     \__ \ / __ `// //_  /  / __ \ / / / // ___// __ `/      / /_/ // _ \ / ___// _ \ / __ `// ___// ___// __ \
#    ___/ // /_/ // /  / /_ / /_/ // /_/ // /   / /_/ /      / _, _//  __/(__  )/  __// /_/ // /   / /__ / / / /
#   /____/ \__,_//_/  /___//_.___/ \__,_//_/    \__, /      /_/ |_| \___//____/ \___/ \__,_//_/    \___//_/ /_/
#                                              /____/
# Salzburg Research ForschungsgesmbH
# Armin Niedermueller

# OPC UA Server and ROS Bridge on Panda
# The purpose of this OPCUA server is to provide methods to control the panda robot and read its state, also it is a
# bridge between ROS and OPCUA.

from opcua import ua, uamethod, Server
import rospy
from std_msgs.msg import String
import subprocess
import threading
import datetime
import logging
import socket
import time
import sys




# create logger
logger = logging.getLogger('opc_ua_server_panda')
logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s [%(filename)s:%(lineno)d] - %(levelname)s - %(message)s')

# add formatter to ch
ch.setFormatter(formatter)

fh = logging.FileHandler('/var/log/opc_ua/_server_panda.log')
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)
logger.addHandler(fh)

# 'application' code
logger.debug('debug message')
logger.info('info message')
logger.warn('warn message')
logger.error('error message')
logger.critical('critical message')

sys.path.insert(0, "..")

global_robot_state = None
global_robot_moving = "None"
global_robot_order = "XX 0"


def callback_ros_sub(data):
    global global_robot_state
    global global_robot_moving

    rospy.loginfo(rospy.get_caller_id() + "robot state received via ros topic: %s", data.data)
    
    global_robot_state = data.data

    if global_robot_state is not "Moving":
        global_robot_moving = True
    else: 
        global_robot_moving = False


@uamethod
def move_robot_libfranka(parent, movement, place):
    logger.debug("in move robot libfranka")
    robot_ip = "192.168.13.1"
    logger.debug("in method: " + robot_ip, movement + place)
    robot_process = subprocess.Popen(["./kick_off_event_x", robot_ip, movement, place])

    return True


@uamethod
def move_robot_ros(parent, movement, place):
    global global_robot_order
    global_robot_order = movement + " " + place     # SO 3

    return True


if __name__ == "__main__":

    global global_robot_state
    global global_robot_moving
    global global_robot_order

    # robot process object
    p = None

    #### ROS NODE SETUP #####
    panda_publisher = rospy.Publisher("ros_opcua_order", String, queue_size=0)   # no queue for received messages
    panda_subscriber = rospy.Subscriber("ros_opcua_response", String, callback_ros_sub)
    rospy.init_node('ros_opcua_bridge', anonymous=True)
    rate = rospy.Rate(3) # operate while loop with 3Hz

    ## OPC-UA SERVER SETUP ##
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
    mover_libfranka = robot_object.add_method(idx, "MoveRobotLibfranka", move_robot_libfranka,
                                              [ua.VariantType.String, ua.VariantType.String], [ua.VariantType.Boolean])
    mover_ros = robot_object.add_method(idx, "MoveRobotRos", move_robot_ros,
                                        [ua.VariantType.String, ua.VariantType.String], [ua.VariantType.Boolean])
    robot_state = robot_object.add_variable(idx, "RobotState", "init")
    robot_moving = robot_object.add_variable(idx, "RobotMoving", False)
    # var = robot_object.add_variable(idx, "Var", 0.0)

    # Set parameters writable by clients
    # server_time.set_writable()

    # Start the server
    server.start()

    logger.debug("OPC-UA - Panda - Server started at {}".format(url))

    try:
        # Assign random values to the parameters
        logger.debug("going into loop")

        while not rospy.is_shutdown():
            TIME = datetime.datetime.now()  # current time

            # set the random values inside the node
            logger.debug("set robot state to %s", global_robot_state)
            robot_state.set_value(global_robot_state)

            logger.debug("set robot moving value to %s", global_robot_moving)
            if global_robot_moving == "true":
                robot_moving.set_value(True)
            elif global_robot_moving == "false":
                robot_moving.set_value(False)

            if global_robot_order.split(' ')[0] != 'XX':
                
                rospy.loginfo("main: robot order changed - sending " + global_robot_order + " via ros")
                logger.debug("main: robot order changed - sending " + global_robot_order + " via ros")
                panda_publisher.publish(global_robot_order)
                global_robot_order = "XX 0" # set to XX 0 again, otherwise the robot would run again and again

            rate.sleep()


            logger.debug("gloval_robot_moving is: %s", global_robot_moving)
            # logger.debug("Robot-State: [" + str(global_robot_message.id) + "] : " + str(robot_state.get_value()) + ". Server-Time: " + str(server_time.get_value()))
            server_time.set_value(TIME)
            # var.set_value(var2)


    except KeyboardInterrupt:
        logger.debug("\nCtrl-C pressed. OPCUA - Pixtend - Server stopped at {}".format(url))
    except rospy.ROSInterruptException:
        pass
    finally:
        # close connection, remove subscriptions, etc
        if p != None:  # terminate robot process if there is one
            p.terminate()
        server.stop()
        conbelt = None

sys.exit(0)
