# ROS-Adapter
Connecting the ROS of the Panda robot with the Messaging System.
This component subscribes to ROS topics and forwards selected metrics to the messaging system which is based on
Apache Kafka.

The metrics involve:
* Coordinates of the robot's working point
* position


Therefore both services must be running:
* [ROS](https://github.com/iot-salzburg/dtz_panda) in the same repository.
* [panta-rhei stack](https://github.com/iot-salzburg/panta_rhei)


The ROS Adapter is based on the components:
* A ROS setup, where `rosrun state_export` works.
* Kafka Client [librdkafka](https://github.com/geeknam/docker-confluent-python) version **2.1**
* Python Kafka module [confluent-kafka-python](https://github.com/confluentinc/confluent-kafka-python) 
version **0.11.6**


## Contents

1. [Requirements](#requirements)
2. [Basic Configuration](#basic-configuration)
2. [Quickstart](#quickstart)
3. [Deployment](#deployment-in-the-docker-swarm)
3. [Configuration](#configuration)
4. [Trouble-Shooting](#trouble-shooting)


## Requirements

3.  Make sure the [Panta Rhei](https://github.com/iot-salzburg/panta_rhei) stack is running.
    This ROS-Adaper requires Apache **Kafka**, as well as the **SensorThings** server GOST.
4.  Make sure ROS runs and is accessable from the host machine:

    ```bash
    cd /home/panda/franka_apps_ws/src/state_export/scripts/
    ln /home/panda/dtz_panda/ros_kafka_adapter/src/ros_kafka_adapter.py ros_kafka_adapter.py

    roslaunch franka_control franka_control.launch robot_ip:=192.168.13.1
    rostopic list
    ```

    The command `rostopic list` should return multiple topics including `/franka_state_controller/franka_states` and `/joint_states`.


5.  Clone the Panta Rhei client into the `src`-directory:
    
```bash
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
    client.register_existing(mappings_file=MAPPINGS)
    # client.register_new(instance_file=INSTANCES)  # Registering of new instances should be outsourced to the platform
    ```
    
## Quickstart

The ROS-Adapter uses SensorThings to semantically augment
the forwarded data. Data that is later on consumed by the
suggested [DB-Adapter](https://github.com/iot-salzburg/DB-Adapter/)
decodes the generic data format using the same SensorThings server.

### Starting the ROS broker

Make sure the symlink to `/home/panda/dtz_panda/ros_kafka_adapter/src/ros_kafka_adapter.py` 
called `ros_kafka_adapter.py ` was created in `/home/panda/franka_apps_ws/src/state_export/scripts/`.

```bash
cd /home/panda/franka_apps_ws
rosrun state_export ros_kafka_adapter.py 
```


