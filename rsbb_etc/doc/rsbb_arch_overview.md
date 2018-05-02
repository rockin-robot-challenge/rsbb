RSBB Architecture Overview
=================================================

The RSBB (Referee, Scoring and Benchmarking Box) is the software that supports the execution of benchmarks in the European Robotics League - Service Robots.

The execution of a benchmark consists primarily of measuring and evaluating in many ways the behaviour and interaction of the robot with the environment and people, and secondarily in the collection of the available informations that can later be analysed and made available as datasets.
To achieve this, the RSBB communicates and interacts with the smart devices present in the testbed, the people and with the robot.

The people are the actors of the benchmarks and the referee or a referee assistant.
The communication with the people happens through the Graphical User Interface (GUI) available on the computer on which the software is run, and through an Android app running on a tablet.
The Home Automation Devices are some smart devices such as lights and window blinds, that can be controlled remotely by the robots and the people.

The communication with the robot allows to ask to execute a task or a series of goals, and to request and provide the informations useful for the context of the benchmark.
The RSBB also communicates with the Home Automation Devices present in the testbed, in order to allow the robot to use them.

Additionally the RSBB communicates with other systems that can provide informations about the robots, people or the environment.
An example is the Motion Capture system (MoCap), that can accuratley measure the position of the robots, objects and people in the testbed.

The benchmark executed by the robot can either be a simple goal, that corresponds to the execution of a manually scored task, or of an excange of goals and information between the robot and the RSBB.
In this case, the benhmark is executed by running a Python script (called Benchmark Script, or BmScript in short), that commands and requests informations to the robot, collects information about the environment and, ultimatly, evaluates the result of the benchmark with a score.

The RSBB also takes care to manage the informations collected during the benchmark, such as the score of the benchmark and the log of the informations collected during the benchmark (for example, the position of the robot or objects in the testbed throughout the execution of the benchmark, or informations about the status of the components of the software).

## Additional features

The Referee, Scoring and Benchmarking Box also supports the following features:

Clock synchronization monitoring: The RSBB is not the best solution to synchronize robot clocks but is able to warn the referee if clock skew grows above a certain threshold.
This warning is used both to alert the teams that the clock synchronization is at fault and to invalidate any received timestamps.

Benchmark starting and stopping: Benchmarks can only start if robot clock skew is below 100 milliseconds.
Stop can be issued manually by the referee, by the robot if it completed the benchmark or automatically by the RSBB if the time for the benchmark is over or if the robot does not declare that it is saving offline data.

Devices communication: the testbed can include Home Automated Devices such as lights and window blinds.
The RSBB provides an interface to control these devices, enabled only in certain benchmarks, so that the robot does not command the devices directly.
The assistant referee can control the devices from the graphical interface.

Tablet communication: a tablet device can be used to communicate with the robot.
Tablet communication passes through the RSBB and is enabled only for certain benchmarks.

Scoring: the RSBB keeps track of the scoring items that can be evaluated during the competition (not using offline data).
For Task Benchmarks (TBMs), the assistant referee marks scoring items in the interface.
For FBMs, scoring is handled by the rockin_benchmarking module.

Online data: data produced by the robot during benchmarks falls in two categories: online and offline.
Offline data is saved in a USB stick for latter analysis.
Online data is transmitted to the RSBB.
The RSBB displays and saves the data.

Logging: the RSBB saves a full log for each benchmark.

Referee interface: the RSBB includes a fully featured graphical interface to be used by the assistant referee.

Public interface: the RSBB includes an informative interface without controls to be displayed to the public.

Single client communication interface: the RSBB includes all features in a single communication interface.
This way, participating teams only have to implement one communication mechanism.

State information: the RSBB continuously displays the state of the benchmark, of the robot and of the other components of the system.

Multiple simultaneous benchmarks: in some situations, it is useful to run task and functionality benchmarks simultaneously for different teams.
The RSBB handles this transparently, supporting an arbitrary number of simultaneous benchmarks and  connected referee interfaces.
To support multiple simultaneous benchmarks, the RSBB uses the concept of zones.
A zone is a group of scheduled benchmarks, not necessarily associated with a physical area.
The RSBB handles all known zones independently, but a robot may only be participating in a benchmark in a single zone simultaneously (during competitions, at least two zones are expected to exist: Task Benchmarks and Functionality Benchmarks zones).
An arbitrary number of referee Interfaces can be used to control different zones or to provide multiple views of the same zone.



## Packages

![RSBB internal and external communication](/rsbb_etc/doc/images/RSBB_int_ext_comm.svg)

The RSBB is a collection of ROS packages that communicate with each other through ROS topics and services, and with the robots through Protobuf.
The main component is the Core.
It controls all defined zones and handles communication with the robots, devices, tablet and with the other components of the RSBB.
It functions as the brain of the system, and must always be running.
The Home Automation Devices are controlled by the roah_devices package that also connects to the core through ROS (See the package's [README](/rsbb_devices_smartif/README.md)).
The tablet runs an external Android application that connects to the core through the public channel (Sec. [Public channel](/rsbb_etc/doc/rsbb_arch_overview.md#Public-channel)).
The benchmark scripts are executed by the rsbb_bmbox package.
This package provides the interface with which the benchmark scripts can interact with the core in order to send commands and receive information from the robots and people.

The components of the system are contained in the following packages:

* rsbb_etc: This package contains the configuration, documentation, the main launchfiles and resource material

* rsbb_refbox: This package provides the rqt GUI nodes and the core node. The core node communicates with the robot and with the other nodes of the RSBB, namely with the rqt GUI, the BmBox, the roah_devices and the record server nodes.

* rsbb_bmbox: This package provides the benchmark script server that executes the benchmark scripts (BmScripts) and communicates with the refbox. The bechmark scripts also communicate with the mocap node and external nodes.

* rsbb_utils: This package provides python scripts used to acquire data that is used by benchmark scripts.

* roah_devices: This package provides the node that interact with the Home Automation Devices.

* rsbb_record_server: This package provides the node that records the rosbags during the execution of the benchmarks.

* rsbb_mocap_optitrack: This package provides the node that receives the motion capture data and published it in the ROS framework.

* rsbb_benchmarking_messages: This package provides the messages and services used by the nodes for the communication between different packages.


### Internal communication

The interface for the communication between the RSBB nodes is composed by ROS services and topics.

The services and topics used to communicate between different packages are specified in the rsbb_benchmarking_messages package.

In particular, see the [BmBox documentation](/rsbb_etc/doc/bmbox/bmbox_overview.md) for details on the implementation of the communication with the core.



## External communication

![Network Example](/rsbb_etc/doc/images/example_RSBB_network_graph.svg)

### Network

The RSBB uses the protobuf_comm library for communication.
All communication uses UDP, over two types of channels.
A single public channel uses UDP broadcast to communicate with all robots at the same time.
Multiple private channels are used to communicate with a single robot while it is running a benchmark.
Private channels use UDP unicast.
The robots and the RSBB must be on the same network, and the rsbb_broadcast_address parameter must be the broadcast address of the network.
The public channel is set up at the port specified in the rsbb_port parameter.
The private channels use rsbb_port+1, rsbb_port+2 and so on without reusing.

Note that, the RSBB and the robot can not be executed on the same computer.

### Public channel

The public channel is used to transmit information that is relevant to all robots unencrypted.
The RSBB transmits the RoahRsbbBeacon every second, containing:
* Information about which robots are active in benchmarks.

Robots listed here should set up private channels for further information.
* The full state of the home devices.
* The full state of the tablet.

Active robots should also transmit their beacon, RobotBeacon, every second.
This contains:
* Identification of the robot, to be listed as active.
* The current time at the time of transmission, to detect problems in clock synchronization.

When a robot sets up a private channel, it should stop transmitting on the public channel.
The tablet also transmits its state on the public channel, but robots should ignore it.
Robots should only trust tablet information received in the RoahRsbbBeacon.

### Private channel

When a robot is active in a benchmark, a private channel is set up for communication.
This channel is encrypted with the team password, to avoid interference from other sources.
This method of protection is only sufficient to avoid honest mistakes in a practical way, not deliberate forgery of messages.

The RSBB transmits benchmark_state containing:
* The code of the specific benchmark to execute.
* The benchmark state.
* An acknowledgment of the last message received from the robot, to avoid eternal retransmission of data.
* Generic goal. Used by the scripted benchmarks to communicate a goal to the robot.

The robot transmits the robot_state containing:
* The current time at the time of transmission, to detect problems in clock synchronization.
* Number of offline data messages saved. Can also be the size of saved data. This is considered to be a fail-safe feature to check whether the robot is recording the mandatory offline data.
* The robot state.
* General goal result. Used in scripted benchmarks to communicate the result of the executed goal.
* Notifications issued by the robot. Used in TBM “Welcoming Visitors” and “Object Manipulation” FBM.
* Activation event. Used in TBM “Welcoming Visitors”.
* Visitor identification. Used in TBM “Welcoming Visitors”.
* Final command identified by the robot. Used in TBM “Catering for Granny Annie’s Comfort”.
* Home devices control commands. Used in TBM “Catering for Granny Annie’s Comfort”.
* Tablet map display command. Used in TBM “Catering for Granny Annie’s Comfort”.

Additional informations may be added to benchmark_state and robot_state when implementing further benchmarks.

## User Interface

`TODO: insert images from new GUI`

For reference, see the [RSBBv1 documentation (deprecated)](/rsbb_etc/doc/RoAH_RSBBv1_Manual_deprecated.pdf).



