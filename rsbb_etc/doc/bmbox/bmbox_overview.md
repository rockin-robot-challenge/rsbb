Benchmarking Box Overview
=================================================

The Benchmarking Box (BmBox) is the package that provides the benchmark scripting framework and the component of the system that runs the scripts.

![RSBB BmBox classes](/rsbb_etc/doc/images/RSBB_BmBox_classes_arch.svg)


# Script Framework

The scripts are implemented using the framework provided by the classes in the graph.
The framework is divided in two layers: the RefBox communication layer and the functionality layer.

The script uses the functions and properties provided by these two layers to acquire the informations about the benchmark and to request the execution of goals and manual operations to the core.


## RefBox Communication Layer

![RSBB Benchmark States Graph](/rsbb_etc/doc/images/RSBB_benchmark_states.svg)

The layer is implemented in the RefBoxComm class.
The RefBox communication layer provides the functions to interact with the core, and specifically the functions used to request a goal or manual operation that are available in the BenchmarkObject class.

The interaction is done through service calls.
The services are divided in three groups: services advertised by the core for RefBoxComm, services advertised by RefBoxComm for the core and services advertised by the core for the script server.

The services advertised by RefBoxComm for the core are:
* execute_manual_operation
* execute_goal
* end_benchmark

The services advertised by the core for RefBoxComm are:
* start_benchmark
* manual_operation_complete
* goal_execution_started
* goal_complete
* referee_score
* stop_benchmark

The services advertised by the core are called to request actions: start to execute a goal or a manual operation, end the benchmark.
These service calls correspond to the events that cause transitions in the state machines of the core.

The remaining transitions are triggered due to the core's internal events (e.g., global timeout) by events from the robot (e.g., the robot completing a goal), or by events from the user interface (e.g., the RefBox operator pushes the stop button).

The core maintains three states relative to the BmBox communication, the Benchmark State, the Goal Execution State and the Manual Operation State.

The Benchmark State is the main state of the benchmark, and describes the main phases of the benchmark execution.
In particular, it defines whether the benchmark is running or if the benchmark is completed (end, stop, global timeout, error).

The Goal Execution State and the Manual Operation State are used to keep track of the current goal being executed by the robot and the current manual operation being executed by the RefBox operator.




## Benchmark Functionality Layer

The layer is implemented in the BaseBenchmarkObject class.

The benchmark functionality layer provides the functions used to access to the properties of the current benchmark from the BenchmarkObject class (team name, run, score, parameters, etc), manages the initialisation of the RefBoxComm object and manages the result object.

The result object contains the informations about the benchmark and is frequently saved in the filesystem.
This object contains both the score values set through the score property, and meta informations such as the team name, the run number, the execution start and end timestamps, the reason for which the benchmark ended (stop, timeout, etc).
The result file is saved in a path composed by directories  that refer to the team name, benchmark code and run number, in order to collect the result files from all bechmark runs in a consistent way.

The benchmark functionality layer also provides the the benchmark specific configuration parameters through the params property.

The actual benchmark code is implemented inside the mandatory 'execute' function of the BenchmarkObject class, provided by the framework.

# Script Server

Scripts are loaded by the script server on start up and executed when requested by the RefBox.

The execution is requested by the core by calling the init_benchmark service.
This service provides the benchmark setup informations, such as the benchmark code, the team name, the run number, etc.

Once the execution of the benchmark terminated (for any reason), the core calls the terminate_benchmark service and the server cleans up all the objects and restarts reloading the scripts.

The script is executed by first calling the setup function of the benchmark functionality layer, that initialises the communication with the RefBox and set the benchmark informations.
The wrapped_execute function of the benchmark functionality layer is then called.
This function will wait for the start_benchmark service, signaling that the operator pressed the start button, and finally the script's execute function is called.
