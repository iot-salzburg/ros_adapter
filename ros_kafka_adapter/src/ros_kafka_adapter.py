#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to listen on a specific topic."""

# Import required Python code.
import os
import sys
import json
import pytz
import time
from datetime import datetime

import rospy
# Import custom message data.
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
#from node_example.msg import NodeExampleData
#from node_example.msg import NodeExampleDataWithHeader
from rospy_message_converter import message_converter

# from confluent_kafka import Producer, KafkaError
from panta_rhei.client.digital_twin_client import DigitalTwinClient

# Panta Rhei configuration
CLIENT_NAME = os.environ.get("CLIENT_NAME", "ros-adapter")
SYSTEM_NAME = os.environ.get("SYSTEM_NAME", "at.srfg.iot.dtz")  # "at.srfg.iot.dtz"
SENSORTHINGS_HOST = os.environ.get("SENSORTHINGS_HOST", "192.168.48.71:8082")
BOOTSTRAP_SERVERS = os.environ.get("BOOTSTRAP_SERVERS", "192.168.48.71:9092,192.168.48.72:9092,192.168.48.73:9092,192.168.48.74:9092,192.168.48.75:9092")

MIN_SENDING_INTERVALL = 10  # Minimal sending intervall

class ros_status:
    def __init__(self):
        start_time = time.time()
        self.franka_state_pos_t = start_time + MIN_SENDING_INTERVALL
        self.franka_state_pos_x = float("-inf")
        self.franka_state_pos_y = float("-inf")
        self.franka_state_pos_z = float("-inf")
        self.franka_state_force_t = start_time + MIN_SENDING_INTERVALL
        self.franka_state_force = float("-inf")
        self.panda_gripper_states_t = start_time + MIN_SENDING_INTERVALL
        self.panda_gripper_states = float("-inf")


def publishExample(data):
    """deprecated: Handle and publish example data."""
    # Simply print out values in our custom message.
    rospy.loginfo(rospy.get_name() + " Type: %s, Data is %s", type(data), data)
    publish_message(data)
    rospy.loginfo(rospy.get_name() + " data: %s, type: %s", data, type(data))

def publishFrankaState(payload):
    """builds messages for the working point coordinates (O_T_EE, cols: 13,14,15)
    and the forces in z-axis there (O_F_ext_K, col 3) based on the ros topic '/franka_state_controller/franka_states'"""
    rospy.logdebug(rospy.get_name() + " Type: %s, payload is %s", type(payload), payload)

    # Send if a state change was detected, or every MIN_SENDING_INTERVALL seconds
    actual_franka_state_pos_x = payload.O_T_EE[12]
    actual_franka_state_pos_y = payload.O_T_EE[13]
    actual_franka_state_pos_z = payload.O_T_EE[14]
    if (abs(actual_franka_state_pos_x-ros_status.franka_state_pos_x) > 0.001 or abs(actual_franka_state_pos_y-ros_status.franka_state_pos_y) > 0.001 or 
        abs(actual_franka_state_pos_z-ros_status.franka_state_pos_z) > 0.001 or (time.time() >= ros_status.franka_state_pos_t)):
        # Set next timeout
        if time.time() >= ros_status.franka_state_pos_t:
            ros_status.franka_state_pos_t += MIN_SENDING_INTERVALL
        else:
            ros_status.franka_state_pos_t = time.time() + MIN_SENDING_INTERVALL

        # set the latest states
        ros_status.franka_state_pos_x = actual_franka_state_pos_x
        ros_status.franka_state_pos_y = actual_franka_state_pos_y
        ros_status.franka_state_pos_z = actual_franka_state_pos_z
        
        # try to get the timestamp from the header
        pTime = get_pTime_from_payload(payload)

        # Build and send the message for the positions
        # print("panda.franka_state.pos_x: {}, \tpanda.franka_state.pos_y: {}, \tpanda.franka_state.pos_z: {}".format(
        #    actual_franka_state_pos_x, actual_franka_state_pos_y, actual_franka_state_pos_z))
        pr_client.produce(quantity="panda.franka_state.pos_x", result=actual_franka_state_pos_x, timestamp=pTime)
        pr_client.produce(quantity="panda.franka_state.pos_y", result=actual_franka_state_pos_y, timestamp=pTime)
        pr_client.produce(quantity="panda.franka_state.pos_z", result=actual_franka_state_pos_z, timestamp=pTime)

    # Send the force if a state change was detected, or every MIN_SENDING_INTERVALL seconds
    actual_franka_state_force = payload.O_F_ext_hat_K[2]
    if abs(actual_franka_state_force-ros_status.franka_state_force) > 1 or (time.time() >= ros_status.franka_state_force_t):
        # Set next timeout
        if time.time() >= ros_status.franka_state_force_t:
            ros_status.franka_state_force_t += MIN_SENDING_INTERVALL
        else:
            ros_status.franka_state_force_t = time.time() + MIN_SENDING_INTERVALL

        # set the latest states
        ros_status.franka_state_force = actual_franka_state_force
        
        # try to get the timestamp from the header
        pTime = get_pTime_from_payload(payload)
    
        # Build and send the message for the force
        # print("panda.franka_state.force_z: {}".format(actual_franka_state_force))
        pr_client.produce(quantity="panda.franka_state.force_z", result=actual_franka_state_force, timestamp=pTime)

def publishJointState(payload):
    """builds messages for the gripper position based on the ros topic '/joint_states'"""
    rospy.logdebug(rospy.get_name() + " Type: %s, payload is %s", type(payload), payload)

    # Extract both gripper positions (index 7 and 8) from payload, sending the sum of both
    actual_panda_gripper_states = payload.position[7] + payload.position[8]

    # Send the force if a state change was detected, or every MIN_SENDING_INTERVALL seconds
    if abs(actual_panda_gripper_states-ros_status.panda_gripper_states) > 0.001 or (time.time() >= ros_status.panda_gripper_states_t):
        # Set next timeout
        if time.time() >= ros_status.panda_gripper_states_t:
            ros_status.panda_gripper_states_t += MIN_SENDING_INTERVALL
        else:
            ros_status.panda_gripper_states_t = time.time() + MIN_SENDING_INTERVALL

        # set the latest states
        ros_status.panda_gripper_states = actual_panda_gripper_states
         # try to get the timestamp from the header
        try:
            pTime = datetime.utcfromtimestamp(payload.header.stamp.secs + payload.header.stamp.nsecs * 1e-9).replace(tzinfo=pytz.UTC).isoformat()
        except Exception, e:
            rospy.logwarn("cannot get timestamp from payload %s", payload)
            rospy.logdebug(e)
            pTime = datetime.utcnow().replace(tzinfo=pytz.UTC).isoformat()
    
        # Build and send the message for the force
        print("panda.joint_states.gripper_pos: {}".format(actual_panda_gripper_states))
        pr_client.produce(quantity="panda.joint_states.gripper_pos", result=actual_panda_gripper_states, timestamp=pTime)

def get_pTime_from_payload(payload):
    try:
        pTime = datetime.utcfromtimestamp(payload.header.stamp.secs + payload.header.stamp.nsecs * 1e-9).replace(tzinfo=pytz.UTC).isoformat()
    except Exception, e:
        rospy.logwarn("cannot get timestamp from payload %s", payload)
        rospy.logdebug(e)
        pTime = datetime.utcnow().replace(tzinfo=pytz.UTC).isoformat()
    return pTime

def publish_message(message):
    # rospy.logdebug(rospy.get_name() + " publish to kafka %s", message)
    # Trigger any available delivery report callbacks from previous produce() calls
    pass  
    # rospy.loginfo(rospy.get_name() + " publishing: {}".format(message))
    # producer.poll(0)
    #producer.produce(KAFKA_TOPIC_metric, json.dumps(message).encode('utf-8'), key="Key",
    #                 callback=delivery_report)


def delivery_report(err, msg):
    """ Called once for each message produced to indicate delivery result.
        Triggered by poll() or flush(). """
    if err is not None:
        rospy.logerr('Message delivery failed: {}, {}, {}'.format(err, msg.topic(), msg.value()))
    else:
        rospy.logdebug('Message delivered to {} [{}] with content: {}'.format(msg.topic(), msg.partition(),
                                                                  msg.value()))

def adapters():
    """Configure subscriber."""
    topics = rospy.get_published_topics()
    rospy.loginfo("Topics: %s", json.dumps(topics, indent=4))
    # Create subscribers for appropriate topics, custom message and name of
    # callback function. We do not yet get all topics and subscribe to them.
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, publishFrankaState)
    rospy.Subscriber('/joint_states', JointState, publishJointState)
    #rospy.Subscriber('/panda_pc_heartbeat', NodeExampleData, publish_message)
    #rospy.Subscriber('/exampleWithHeader', NodeExampleDataWithHeader, publish_message)
    #rospy.Subscriber('/exampleWithHeader_throttle', NodeExampleDataWithHeader, publish_message)


# Main function.
if __name__ == '__main__':
    ros_status = ros_status()

    # Init the panta rhei client
    dirname = os.path.dirname(os.path.realpath(__file__))
    INSTANCES = os.path.join(dirname, "instances.json")
    MAPPINGS = os.path.join(dirname, "ds-mappings.json")

    config = {"client_name": CLIENT_NAME,
              "system": SYSTEM_NAME,
              "kafka_bootstrap_servers": BOOTSTRAP_SERVERS,
              "gost_servers": SENSORTHINGS_HOST}

    pr_client = DigitalTwinClient(**config)
    pr_client.register_new(instance_file=INSTANCES)
    # pr_client.register_existing(mappings_file=MAPPINGS)

    # Initialize the node and name it.
    rospy.init_node('ros_kafka_adapter', log_level=rospy.INFO)
    # Go to the main loop.
    adapters()
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()
