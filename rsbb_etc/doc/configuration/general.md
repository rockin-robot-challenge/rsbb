RSBB general benchmarks configuration
=================================================

An example of the general benchmarks configuration:

```yaml
rsbb_broadcast_address: "10.2.0.255"
rsbb_port: 6666
generate_schedule: true
base_results_directory: "~/rsbb_output/"
base_record_directory: "~/rsbb_output/"
general_record_list:
  - "map"
  - "map_metadata"
  - "current_benchmark_result"
  - "bmbox/bmbox_state"
  - "bmbox/refbox_state"
  - "/devices/state"
  - "/devices/bell"
  - "/core/to_gui"
  - "/core/to_public"
  - "/timeout"
  - "/rosout"
  - "/tf"
```



## General parameters

The *general.yaml* configuration file contains the general configuration for all benchmarks.

All of the following parameters must be specified unless otherwise stated.
If a parameter has an invalid value or is not specified, the RSBB or the components of the RSBB using the parameter will not start and print an error message specifying the incorrect configuration.


### rsbb_broadcast_address

This parameter should be a string.

The *rsbb_broadcast_address* parameter specifies the broadcast address of the network used to communicate with the robots.

This parameter must be changed to match the network configuration.

This parameter is used by the RefBox.


### rsbb_port

This parameter should be a string.

The *rsbb_port* parameter specifies the port of the public channel used to communicate with the robots.

This parameter usually does not need to be changed.

This parameter is used by the RefBox.


### generate_schedule

This parameter should be a bool, if not specified it defaults to true.

The *generate_schedule* parameter specifies whether the list of benchmarks executed by each team should be automatically generated, making all benchmarks available to all teams (`generate_schedule: true`) or taken from the schedule configurations files (`generate_schedule: false`).

Changing this parameter is not advisable.

This parameter is used by the RefBox.


### base_results_directory

This parameter should be a string.

The *base_results_directory* parameter specifies the directory path where the results of the benchmarks are saved.
The directory should exist and be a valid path, meaning that the path should point to a directory or to a link, and not to a file.

This parameter is used by the BmBox.


### base_record_directory

This parameter should be a string.

The *base_record_directory* parameter specifies the directory path where the logs of the benchmarks are saved.
The directory should exist and be a valid path, meaning that the path should point to a directory or to a link, and not to a file.

This parameter is used by the record server.


### general_record_list

This parameter should be a sequence of strings (see examples in the beginning of this section and the following specification).

The *general_record_list* parameter specifies the topics recorded by the record server during the benchmark execution.
Note that the list of topics specified in the *benchmarks_description.yaml* configuration file as *record_topics* is also recorded for each benchmark.

This parameter is used by the RefBox and by the record server.
