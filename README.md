# ROS-Adapter
Connecting the ROS of the Panda robot with the Panta Rhei Messaging System.
This component subscribes to ROS topics and forwards selected metrics to the [Panta Rhei](https://github.com/iot-salzburg/panta_rhei) messaging system which is based on Apache Kafka.

The metrics involve:
* Coordinates of the robot's working point
* Gripper finger width
* vertical force on the end effector

Therefore both services must be running:
* [ROS](https://github.com/iot-salzburg/ros_supporting_files)
* [panta-rhei stack](https://github.com/iot-salzburg/panta_rhei)


The ROS Adapter is based on the components:
* A ROS setup connected to the panda robot.
* Kafka Client [librdkafka](https://github.com/geeknam/docker-confluent-python) version **2.1**
* Python Kafka module [confluent-kafka-python](https://github.com/confluentinc/confluent-kafka-python) 
version **0.11.6**


## Contents

1. [Requirements](#requirements)
2. [Basic Configuration](#basic-configuration)
3. [Quickstart](#quickstart)


## Requirements

1.  Make sure the [Panta Rhei](https://github.com/iot-salzburg/panta_rhei) stack is running.
    This ROS-Adaper requires Apache **Kafka**, as well as the **SensorThings** server GOST.
2.  Make sure ROS runs and is accessable from the host machine:

    ```bash
    # cd /home/panda/franka_apps_ws/src/state_export/scripts/
    # ln -sf /home/panda/dtz_panda/ros_kafka_adapter/src/ros_kafka_adapter.py ros_kafka_adapter.py

    # cd /home/panda/franka_apps_ws
    # . devel/setup.bash
    roslaunch franka_control franka_control.launch robot_ip:=192.168.13.1
    rostopic list
    ```
    If connected with `ssh` ther will be shown an `error message including pyqt`, because there is not GUI. This message can be ignored.
    The command `rostopic list` should return multiple topics including `/franka_state_controller/franka_states` and `/joint_states`.


3.  Install the Kafka libraries and clone the Panta Rhei client into the `src`-directory:
    
    ```bash
    sudo apt-get update && sudo apt-get install make
    # Installing librdkafka which is an C client library for Kafka
    mkdir /kafka  > /dev/null 2>&1 || true
    cd /kafka
    git clone https://github.com/edenhill/librdkafka
    cd librdkafka
    git checkout v0.11.1
    ./configure && make && sudo make install && sudo ldconfig

    # Installing official python client which is based on librdkafka
    pip install confluent-kafka
    ```

## Basic Configuration
Now, the client can be imported and used in `ros_kafka_adapter.py` with:
    
    ```python
    import os, sys
    from client.digital_twin_client import DigitalTwinClient

    # Set the configs, create a new Digital Twin Instance and register file structure
    config = {"client_name": "ros-adapter",
              "system": "at.srfg.iot.dtz",
              "gost_servers": "192.168.48.71:8082",
              "kafka_bootstrap_servers": "192.168.48.71:9092,192.168.48.71:9093,192.168.48.71:9094"}
    client = DigitalTwinClient(**config)
    client.register(instance_file=INSTANCES)
    ```
    
## Quickstart

The ROS-Adapter uses SensorThings to semantically augment
the forwarded data. Data that is later on consumed by the
suggested [DB-Adapter](https://github.com/iot-salzburg/DB-Adapter/)
decodes the generic data format using the same SensorThings server.

### Starting franka control

Run in a separate console:

```bash
cd /home/panda/franka_apps_ws
roslaunch franka_control franka_control.launch robot_ip:=192.168.13.1
```
    
For information how to start the ROS demo, follow the internal link [here](https://secure.salzburgresearch.at/wiki/pages/viewpage.action?pageId=31000016).

Make sure `rostopic echo /franka_state_controller/joint_states` yields the joint states.
    
   
    
## Setup of the ROS-Kafka-Adapter

This project is based on this repository. ROS-messages can only be forwarded into the Panta Rhei project if they are received as described above.

1. Clone the repository into `/src/`

    ```git clone --recurse-submodules https://github.com/iot-salzburg/ros_adapter.git```

2. Create and activate virtualenv:
```bash
virtualenv --python=/usr/bin/python3.7 .venv
source .venv/bin/activate 
pip install -r requirements.txt
```

3. Run the script with:
```bash
/src/ros_adapter/.venv/bin/python3.7 /src/ros_adapter/src/ros_kafka_adapter.py
```

4. The auto-start script is in ` ~/.bashrc`
```
/src/ros_adapter/.venv/bin/python3.7 /src/ros_adapter/src/ros_kafka_adapter.py $> /src/ros_adapter/ros_kafka_adapter.log &
```
It should start automatically.

5. Check if messages are received:
  - In the Dashboard: http://192.168.48.71:3000/d/dtz-lab/dtz-dashboard?orgId=1&refresh=10s

  - In Kafka (check for valid iot.ids:
    ```
    /kafka/bin/kafka-console-consumer.sh --bootstrap-server 192.168.48.71:9092 --topic at.srfg.iot.dtz.int
    ```
    
    
## Deployment

4. The auto-start script is in ` ~/.bashrc`
```
/src/ros_adapter/.venv/bin/python3.7 /src/ros_adapter/src/ros_kafka_adapter.py $> /src/ros_adapter/ros_kafka_adapter.log &
```

This launches the ROS-Adapter while booting, check using `ps -ef |grep ros_kafka_adapter`
