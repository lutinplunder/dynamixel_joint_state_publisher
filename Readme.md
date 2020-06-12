# Dynamixel Joint State Publisher

[![Syropod Banner](https://i.imgur.com/QyMTwG3.jpg "CSIRO Robotics")](https://research.csiro.au/robotics/)

A ROS node written in Python which subscribes to joint state from the dynamixel_motor drivers for each joint in the syropod (e.g. `/syropod/AR_coxa_joint/state`) and collates it to publish as a single topic (`/joint_states`) which is subscribed by [Syropod Highlevel Controller](https://github.com/csiro-robotics/syropod_highlevel_controller).

## Getting Started

If you haven't looked at the tutorials for using OpenSHC, see [OpenSHC Tutorials](https://github.com/csiro-robotics/shc_tutorials).

### Requirements

* Ubuntu 18.04 LTS
* ROS Melodic

### Dependencies

* [dynamixel_motor](https://github.com/csiro-robotics/dynamixel_motor)
  * `git clone https://github.com/csiro-robotics/dynamixel_motor.git`

### Installation

```bash
mkdir -p openshc_ws/src
cd openshc_ws/src
git clone https://github.com/csiro-robotics/dynamixel_joint_state_publisher.git
cd ..
catkin build
```

## Nodes

### joint_states_publisher

#### Subscribed Topics

* Joint State Data (per joint):
  * Description: Joint position for each individual joint.
  * Topic: */syropod/\*LEG_ID\*\_\*JOINT_ID\*\_joint/state*
  * Type: std_msgs::Float64

#### Published Topics

* Joint State Data (combined):
  * Description: Joint position for each individual joint combined into single message.
  * Topic: */joint\_states*
  * Type: sensor_msgs::JointState

## Authors

* Navinda Kottege
* Huub Heijnen

## Changelog

* v0.1.0
  * Initial version
  * Subscribes to dynamixel joint states and republishes as combined message.

## License

This project is licensed under the CSIRO Open Source Software Licence Agreement (variation of the BSD / MIT License) - see the [LICENSE](LICENSE) file for details.

## Issues

Please report bugs using [Issue Tracker](https://github.com/csiro-robotics/dynamixel_joint_state_publisher/issues) or contact us via email [shc-support@csiro.au](mailto:shc-support@csiro.au).
