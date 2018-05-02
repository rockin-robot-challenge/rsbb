RSBB benchmarks description configuration
=================================================

An example of two benchmark descriptions:

```yaml
- code: "STB"
  desc: "Test Benchmark: Simple Test Benchmark"
  scripted: true
  multiple_robots: false
  commands_devices: false
  order: 1
  default_goal_timeout: 30.000
  global_timeout: 200.0
  scoring:
    - Penalized Behaviors:
      - { type: uint, desc: "The robot bumps into the furniture or people." }
    - Disqualifying Behaviors:
      - { type: uint, desc: "The robot hits people or the furniture." }
      - { type: uint, desc: "The robot damages the test bed." }
  record_topics:
    - "/rsbb/actor_markerset/pose"
    - "/rsbb/actor_markerset/pose2d"
- code: "TBM1"
  desc: "Task Benchmark 1: Getting to know my home"
  scripted: false
  multiple_robots: false
  commands_devices: false
  order: 2
  timeout: 600
  scoring:
    - Achievements:
      - { type: bool, desc: "The robot detects the door with changed state." }
      - { type: uint, desc: "The robot detects each piece of moved furniture." }
      - { type: uint, desc: "The robot detects each changed object." }
      - { type: bool, desc: "The robot correctly executes the command given in Phase 2." }
    - Penalized Behaviors:
      - { type: uint, desc: "The robot requires multiple repetitions of human gesture/speech." }
      - { type: uint, desc: "The robot bumps into the furniture." }
      - { type: uint, desc: "The robot stops working." }
    - Disqualifying Behaviors:
      - { type: uint, desc: "The robot hits Annie or another person in the environment." }
      - { type: uint, desc: "The robot damages the test bed." }
```



## Benchmark parameters

The *benchmarks_description.yaml* configuration file contains the configuration of each benchmark.
The main yaml root object should be a sequence of benchmark yaml objects.
A benchmark object is a map in which the key is the parameter's name and the value is the parameter's value (see examples).

The following parameters can be specified for each benchmark.
Some parameters are unique, meaning that two benchmarks should not have the same value for these parameters.
Some parameters are required, others do not need to be specified.
If a parameter has an invalid value or is not specified even if required, the RSBB or the components of the RSBB using the parameter will not start and print an error message specifying the incorrect configuration (most of the times).


### code

This parameter is required, unique and should be a string.

The *code* parameter is the unique identifier of the benchmark.

This parameter is used by the RefBox, BmBox and record server nodes.


### desc

This parameter is required and should be a string.

The *desc* parameter (description) is the name and description of the benchmark and is displayed
by the RefBox to the operator as a human readable identifier of the benchmark.

This parameter is used by the RefBox.


###  scripted

This parameter is required and should be a bool.

The *scripted* parameter specifies whether the benchmark is executed by a benchmark script.
A benchmark that is not scripted (`scripted: false`) can only execute a simple goal,
whether a benchmark that is scripted (`scripted: true`) will be executed following the scripts commands.

This parameter is used by the RefBox.


### multiple_robots

This parameter is required and should be a bool.

The *multiple_robots* parameter specifies whether the benchmark should be executed with many robots.
A benchmark that runs with multiple robots can not be scripted (`scripted: false`) and can not command devices (`commands_devices: false`).

This parameter is used by the RefBox.


### commands_devices

This parameter is required and should be a bool.

The *commands_devices* parameter specifies whether the benchmark accepts devices control messages from the robots.

This parameter is used by the RefBox.


### order

This parameter is required, unique and should be a signed int.

The *order* parameter specifies the order of visualisation of the benchmarks in the RefBox.
The parameter can be negative.
The benchmarks are ordered from the one with lowest *order* to the one with the highest.

This parameter is used by the RefBox.


### default_goal_timeout

This parameter is required on some conditions and should be a float or an int greater than 0.

The parameters *default_goal_timeout* and *global_timeout* are (both) required if the *scripted* parameter is true.
If *scripted* is false, the *timeout* parameter should be specified instead.
The *default_goal_timeout* parameter specifies the default value of the timeout in seconds for a goal when not specified by the benchmark script.

This parameter is used by the RefBox.


### global_timeout

This parameter is required on some conditions and should be a float or an int greater than 0.

The parameters *global_timeout* and *default_goal_timeout* are (both) required if the *scripted* parameter is true.
If *scripted* is false, the *timeout* parameter should be specified instead.
The *global_timeout* parameter specifies the value of the timeout in seconds for the entire benchmark.
The global timeout can only be specified in this configuration and can not be set by the benchmark script contrary to the *default_goal_timeout* parameter.

This parameter is used by the RefBox.


### timeout

This parameter is required on some conditions and should be a float or an int greater than 0.

The *timeout* parameter is required if the *scripted* parameter is false.
If *scripted* is true, the *global_timeout* and *default_goal_timeout* parameters should be specified instead.
The *timeout* parameter specifies the value of the timeout in seconds for the entire benchmark.

This parameter is used by the RefBox.


### scoring

This parameter is not required and should be a map (see examples in the beginning of this section and the following specification).

The keys of the map should be the strings 'Achievements', 'Penalized Behaviors' or 'Disqualifying Behaviors'.
Each value of the map should be a sequence of objects like `{ type: uint, desc: "Description of the behavior (integer value)." }` or `{ type: bool, desc: "Description of the behavior (true or false value)." }`.

This parameter is used by the RefBox and by the BmBox.


### record_topics

This parameter is not required and should be a sequence of strings (see examples in the beginning of this section and the following specification).

The *record_topics* parameter specifies the topics recorded by the record server during the benchmark execution.
Note that the list of topics specified in the *general.yaml* configuration file as *general_record_list* is also recorded for all benchmarks.


This parameter is used by the RefBox and by the record server.
