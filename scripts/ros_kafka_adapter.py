#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to listen on a specific topic."""

# Import required Python code.
import rospy
import json
import pytz
from confluent_kafka import Producer, KafkaError

# Import custom message data.
from franka_msgs.msg import FrankaState
#from node_example.msg import NodeExampleData
#from node_example.msg import NodeExampleDataWithHeader
from rospy_message_converter import message_converter
from datetime import datetime

#MAX_MSGS_PER_SEC = 10 # Publishing rate limit: Maximum number of messages per second

# Messaging System Configurations
BOOTSTRAP_SERVERS = "192.168.48.61:9092,192.168.48.62:9092,192.168.48.63:9092"
#KAFKA_GROUP_ID = "ros-adapter"
#KAFKA_TOPIC_metric = "dtz.sensorthings"
KAFKA_TOPIC_metric = "test" #"SensorData"
KAFKA_TOPIC_logging = "dtz.logging"
SENSORTHINGS_HOST = "192.168.48.60"
SENSORTHINGS_PORT = "8082"

# Messaging System
producer = Producer({'bootstrap.servers': BOOTSTRAP_SERVERS, 'api.version.request': True})

def publishExample(data):
    """deprecated: Handle and publish example data."""
    # Simply print out values in our custom message.
    rospy.loginfo(rospy.get_name() + " Type: %s, Data is %s", type(data), data)
    publish_message(data)
    rospy.loginfo(rospy.get_name() + " data: %s, type: %s", data, type(data))

def publishFrankaState(data):
    """deprecated: Handle and publish Franka State to Kafka."""
    # Simply print out values in our custom message.
    rospy.loginfo(rospy.get_name() + " Type: %s, Data is %s", type(data), data)
    publish_message(data)


def publish_message(payload):
    message = {}
    message["Datastream"] = dict({"@iot.id": 38})
    message["result"] = message_converter.convert_ros_message_to_dictionary(payload)
    message["resultTime"] = datetime.utcnow().replace(tzinfo=pytz.UTC).isoformat()

    # try to get the timestamp from the header
    try:
        pTime = datetime.utcfromtimestamp(payload.header.stamp.secs + payload.header.stamp.nsecs * 1e-9).replace(tzinfo=pytz.UTC).isoformat()
        message["phenomenonTime"] = pTime
    except Exception, e:
        rospy.logwarn("cannot get timestamp from payload %s", payload)
        rospy.logdebug(e)
#        pass
        
    
    rospy.logdebug(rospy.get_name() + " publish to kafka %s", message)
    # Trigger any available delivery report callbacks from previous produce() calls
    rospy.loginfo(rospy.get_name() + " publish...")
    producer.poll(0)
    producer.produce(KAFKA_TOPIC_metric, json.dumps(message).encode('utf-8'), key="Key",
                     callback=delivery_report)


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
    rospy.Subscriber('/franka_state_controller/franka_states_throttle', FrankaState, publish_message)
    #rospy.Subscriber('/panda_pc_heartbeat', NodeExampleData, publish_message)
    #rospy.Subscriber('/exampleWithHeader', NodeExampleDataWithHeader, publish_message)
    #rospy.Subscriber('/exampleWithHeader_throttle', NodeExampleDataWithHeader, publish_message)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ros_kafka_adapter', log_level=rospy.INFO)
    # Go to the main loop.
    adapters()
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()
