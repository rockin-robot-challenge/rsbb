RSBB: Referee, Scoring and Benchmarking Box
=================================================

This repository contains the ERL-SR (European Robotics League - Service Robots) Referee, Scoring and Benchmarking Box.
Note that the name 'rockin' refers to the previous version of the competition.

Table of Content:

* [Installation guide (this README)](/README.md#Installation)
* [Architecture overview](/rsbb_etc/doc/rsbb_arch_overview.md)
* [Benchmark script overview](/rsbb_etc/doc/bmbox/bmbox_overview.md)
* [Benchmark script tutorial](/rsbb_etc/doc/bmbox/benchmark_script_tutorial.md)
* [Configuration documentation](/rsbb_etc/doc/configuration/configuration_overview.md)
* [Utility scripts documentation](/rsbb_etc/doc/utils/utils_overview.md)
* [Smartif HAD documentation](/rsbb_devices_smartif/README.md)


# Installation

:warning: Please remember to always update right before the competitions!
```bash
git pull
git submodule update --init
```

Note that the repository *RoAH RSBB Comm* from https://github.com/rockin-robot-challenge/at_home_rsbb_comm is included as a git submodule.
This repo is shared with the [roah_rsbb_comm_ros](https://github.com/rockin-robot-challenge/at_home_rsbb_comm_ros) package.

## Dependencies

You need to have installed a C++11 compiler, CMake, Boost, Protobuf and OpenSSL.

If you are using Ubuntu, install the dependencies with:
```bash
sudo apt-get install build-essential cmake libboost-all-dev libprotoc-dev protobuf-compiler libssl-dev ros-$ROS_DISTRO-map-server
```

And install the Levenshtein module for Python:
```bash
sudo easy_install python-Levenshtein
```
<!---
still necessary?
-->

:warning: The RSBB is not compatible with Ubuntu 16.04, you need to use ROS Indigo on Ubuntu 14.04.x (or ROS Hydro on Ubuntu 12.04.x).
To install and set up ROS, follow the instructions at http://wiki.ros.org/ROS/Installation/ .

This version of the RSBB was tested with Ubuntu 14.04.5 LTS (Trusty Tahr) and ROS Indigo.

The RSBB is a collection of ROS packages, and does not depend on other packages or software except the ones listed before.

<!---
TODO add the program to play sounds in the dependencies
-->

## Compiling

After `git clone` and after every `git pull`, please do:
```bash
git submodule update --init
```

Compile as a normal ROS package in your Catkin workspace.


## Network Setup

The robots and the RSBB must be on the same network.
The following graph shows an example of a possible network setup:

![Network Example](/rsbb_etc/doc/images/example_RSBB_network_graph.svg)



## Configuration

For the installation and test of the RSBB, configuring the following parameters in rsbb_etc/config/general.yaml should be sufficient:
* `rsbb_broadcast_address` should be set to the `Bcast` of the interface you want to use, as reported by `ifconfig`.
From the example of the network reported before, the broadcast address would be `10.2.0.255`.
* `base_results_directory` and `base_record_directory` should be set to the path of the directory where the result files and the ROS bags are saved.
These two directories can be the same.
For example: `~/rsbb_output/`, in this case the directory "rsbb_output" should be created in home.

The configuration parameters are documented in [configuration overview](/rsbb_etc/doc/configuration_overview.md).


## Running

To run the full RSBB (including the devices node), execute this command in the terminal:
```bash
roslaunch rsbb_etc rsbb.launch
```


### Testing

Now all benchmarks can be executed, but since a robot must be availabe on the network, it is often useful to test the communication with the RSBB by running the so called Dummy Robot, from the [roah_rsbb_comm_ros](https://github.com/rockin-robot-challenge/at_home_rsbb_comm_ros) package.

### Executing benchmarks

`TODO: brief instructions to run benchmarks`
Running a benchmark is similar to the previous (deprecated) verison of the RSBB.
Some instructions are available in the [RSBBv1 documentation (deprecated)](/rsbb_etc/doc/RoAH_RSBBv1_Manual_deprecated.pdf).

<!--- TODO
* auxiliary nodes (MoCap, etc)
* utility nodes (link to doc/utility_scripts_overview)
* ...
--->

<!--- TODO
For a test with dummy home devices use: `:warning: not implemented yet`
```bash
roslaunch rsbb_etc rsbb_dummy_devices.launch rsbb_host:=192.168.1.255 --screen
```
--->

<!--- not necessary anymore, probably
It may be necessary to delete the rqt cache for the new components to appear:
```bash
rm ~/.config/ros.org/rqt_gui.ini
```
--->

<!--- never been used, as far as I know
## Securing the RSBB

Make sure that you run these commands in whatever computer runs the RSBB:
```bash
sudo iptables -A INPUT -i lo -p tcp -m tcp --dport 11311 -j ACCEPT
sudo iptables -A INPUT -p tcp -m tcp --dport 11311 -j DROP
```

You might add this to `/etc/rc.local`, before the `exit` command.

To be able to connect from other computers safely, you must install
the `openssh-server` package:
```bash
sudo apt-get install openssh-server
```

Make sure the `ROS_IP` variable is set correctly.

#### Connecting from remote computers

To launch RSBB clients in other computers, you must have the
`openssh-server` package installed in the server and be running the
RSBB. Then, in the remote computer do:
```bash
ssh -L 127.0.0.1:11311:10.0.0.1:11311 rockin@10.0.0.1
```

In this example, the user is named `rockin` and the server is at
`10.0.0.1`. The `127.0.0.1` at the beginning is mandatory.

Make sure the `ROS_IP` variable is set correctly.

Then, just run the client as if the ROS master were local:
```bash
roslaunch roah_rsbb roah_rsbb_client.launch
```
--->



<!---
# Running in the virtual machine

To test the RSBB without the need to install the software in this repository, a **temporary** virtual machine can be used with Virtual Box
The appliance (an archive containing the virtual machine, the virtual hard drive and the configuration) can be downloaded from this **temporary** link:
https://drive.google.com/file/d/1m_EOQ8Gdw1TNnct_1Vt4N5xH3noOxD_Z/view?usp=sharing

:warning: The current virtual machine will be substituted with a new version, so it is not to be used during competitions.

After importing the appliance into Virtual Box (File / Import Appliance), start the virtual machine named RSBB_VM.
On the first start up, Virtual Box may ask to rename the network interface, in the dialog window choose 'Change Network Settings'.
The settings for the virtual machine will open.
Virtual Box should automatically select a new name for the adapter (Network / Adapter 1 / Name).
If more adapters are available, it may be necessary to choose the adapter connected to the same network as the robots.
This decision can be changed later in the settings of the virtual machine.
Click OK and the virtual machine will start up.

The operating system installed in the virtual machine is Ubuntu 14.04.5 64-bit.
The details of the account are:
```
username: erl
password: benchmarking
```

To launch the rsbb software, open a terminal (ctrl+alt+t) and insert the following command:
```bash
roslaunch rsbb_etc rsbb.launch
```

Notice that the the virtual machine is configured to connect to the ethernet interface with a dinamyc IP (DHCP), so to test a robot against the virtual RSBB it is necessary to set the parameter `rsbb_host` in the configuration of the robots to the IP assigned to the interface of the virtual machine.
This IP can be found by opening a terminal (ctrl+alt+t) and executing the command:
```bash
hostname -I
```
The output of this command will be one IP address, or multiple IP adresses in case the virtual machine is connected to more than one network.
In this case, use the IP of the network to which the robots are connected.
--->



