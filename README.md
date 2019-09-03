# ROS-Adapter
Connecting the ROS of the Panda robot with the Messaging System.
This component subscribes to ROS topics and forwards selected metrics to the [Panta Rhei](https://github.com/iot-salzburg/panta_rhei) messaging system which is based on
Apache Kafka.

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
    cd /home/panda/franka_apps_ws/src/state_export/scripts/
    ln -sf /home/panda/dtz_panda/ros_kafka_adapter/src/ros_kafka_adapter.py ros_kafka_adapter.py

    cd /home/panda/franka_apps_ws
    . devel/setup.bash
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

    cd /home/panda/dtz_panda/ros_kafka_adapter
    git clone https://github.com/iot-salzburg/panta_rhei src/panta_rhei > /dev/null 2>&1 || echo "Repo already exists"
    git -C src/panta_rhei/ checkout srfg-digitaltwin
    git -C src/panta_rhei/ pull
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
              "kafka_bootstrap_servers": "192.168.48.71:9092,192.168.48.72:9092,192.168.48.73:9092,192.168.48.74:9092,192.168.48.75:9092"}
    client = DigitalTwinClient(**config)
    client.register_new(instance_file=INSTANCES)
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
    
### Starting the ROS adapter

Make sure the symlink to `/home/panda/dtz_panda/ros_kafka_adapter/src/ros_kafka_adapter.py` 
called `ros_kafka_adapter.py ` was created in `/home/panda/franka_apps_ws/src/state_export/scripts/`.

Start the adapter with:

```bash
cd /home/panda/franka_apps_ws
rosrun state_export ros_kafka_adapter.py 
```

Monitor the datastream with:
```bash
/kafka/bin/kafka-console-consumer.sh --bootstrap-server :9092 --topic test-topic.data
```

## Deployment

Copy `/systemd/ros-kafka-adapter.service` into `/src/systemd/system/` and run:

```bash
sudo systemctl daemon-reload
sudo systemctl enable ros-kafka-adapter.service
sudo service ros-kafka-adapter start
sudo service ros-kafka-adapter status
# or
sudo journalctl -u ros-kafka-adapter.service -f
```

This launches the ROS-Adapter while booting.
