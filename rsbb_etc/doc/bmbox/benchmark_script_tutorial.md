RSBB benchmark script tutorial
=================================================

This tutorial covers how to develop a simple benchmark.
The Simple Test Benchmark (STB) will be used as an example to learn how to develop a simple benchmark, that can also be used to test the components of the RSBB and the communication between the robots and the RSBB since the benchmark uses most of the RSBB's components.


## Benchmark Scripts

A benchmark is defined in three files: the benchmarks_description.yaml configuration file, the benchmark's own configuration file and the benchmark's (python) script.
The Simple Test Benchmark, used in this tutorial, is part of the default installation of the RSBB and can be found here: [simple_test_benchmark.py](/rsbb_bmbox/scripts/benchmark_scripts/simple_test_benchmark.py).
If you didn't already, install the entire RSBB by following the instructions in the [README](/README.md).


### The Code

The Simple Test Benchmark demonstrates the basic operations you can request to the RefBox: a first manual operation is executed, two goals are executed, then another manual operation is executed.
The benchmark is implemented in the execute function of BenchmarkObject, that will be executed when the RefBox runs this benchmark.

The BaseBenchmarkObject provides the functions needed to interact with the RefBox to request manual operation and goals, let's call them benchmark functions.

The BaseBenchmarkObject also provides these properties: params, referee_score and score
Python properties are objects that seem to be attributes, but are actually managed by get, set and del functions inside BaseBenchmarkObject.
The score property is used to log information on the execution and the result of the benchmark.
The referee_score property is used to read the score that the RefBox operator can change during and before the benchmark execution.
Other functions are available to obtain informations about the current run, like get_team_name and get_benchmark_run.

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml
from rsbb_bmbox.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject

class BenchmarkObject (BaseBenchmarkObject):

	benchmark_code = "STB"

	def execute(self):

		N = 2
		i = 1
		execution_time = rospy.Duration(0.0)


		print "params:\n", self.params

		print "referee_score:\n", self.referee_score

		##########################################
		#            MANUAL OPERATION            #
		##########################################

		manual_operation_first = ManualOperationObject("First Manual Operation")

		self.request_manual_operation(manual_operation_first)

		##########################################
		#     CHECK RESULT AND UPDATE SCORE      #
		##########################################

		if manual_operation_first.has_been_completed():
			print "First Manual Operation result: %s" % manual_operation_first.get_result()
			self.score["first_manual_operation"] = manual_operation_first.get_result()
			self.save_and_publish_score()
		else:
			print "First Manual Operation NOT EXECUTED"
			self.score["first_manual_operation"] = "not executed"
			self.save_and_publish_score()

		if not self.is_benchmark_running():
			if self.has_benchmark_timed_out():
				print "BENCHMARK TIMEOUT"
				return
			elif self.has_benchmark_been_stopped():
				print "BENCHMARK STOPPED"
				return
			else:
				print "BENCHMARK ABORTED"
				return


		while self.is_benchmark_running() and i <= N:

			##########################################
			#                 GOAL i                 #
			##########################################

			goal = GoalObject({"goal": "GOAL %d"%i, "details": 4}, 15.0)

			self.request_goal(goal)
			start_time = rospy.Time.now()

			print "wait_goal_result"

			self.wait_goal_result()
			end_time = rospy.Time.now()

			execution_time += end_time - start_time
			rospy.loginfo("Execution time - %f" % execution_time.to_sec())


			##########################################
			#    CHECK RESULT i AND UPDATE SCORE     #
			##########################################

			self.score["goal_%i"%i] = {}
			self.score["goal_%i"%i]["timeout"] = goal.has_timed_out()
			self.score["goal_%i"%i]["completed"] = goal.has_been_completed()

			if goal.has_timed_out():
				print "GOAL TIMEOUT"
			elif goal.has_been_completed():
				print "GOAL COMPLETED:"
				result = goal.get_result()
				print "result:\n", result
				self.score["goal_%i"%i]["result"] = result
			else:
				print "GOAL NOT COMPLETED"

			self.save_and_publish_score()

			if not self.is_benchmark_running():
				print self.get_end_description()

				if self.has_benchmark_timed_out():
					print "BENCHMARK TIMEOUT"
				elif self.has_benchmark_been_stopped():
					print "BENCHMARK STOPPED"
				else:
					print "BENCHMARK ABORTED"

			i += 1

		##########################################
		#            MANUAL OPERATION            #
		##########################################

		manual_operation_last = ManualOperationObject("Last Manual Operation")

		self.request_manual_operation(manual_operation_last)


		##########################################
		#     CHECK RESULT AND UPDATE SCORE      #
		##########################################

		if manual_operation_last.has_been_completed():
			print "Last Manual Operation result: %s" % manual_operation_last.get_result()
			self.score["last_manual_operation"] = manual_operation_last.get_result()
			self.save_and_publish_score()
		else:
			print "Last Manual Operation NOT EXECUTED"
			self.score["last_manual_operation"] = "not executed"
			self.save_and_publish_score()

		if not self.is_benchmark_running():
			if self.has_benchmark_timed_out():
				print "BENCHMARK TIMEOUT"
			elif self.has_benchmark_been_stopped():
				print "BENCHMARK STOPPED"
			else:
				print "BENCHMARK ABORTED"


		##########################################
		#            UPDATE SCORE                #
		##########################################

		self.score["execution_time"] = execution_time.to_sec()
		self.save_and_publish_score()
```


### The Configuration

Each benchmark configuration is specified in two places: the benchmark description and the benchmark configuration.
The benchmark description is used by the RefBox to know that the benchmark exists.
The benchmark configuration specifies parameters that are easily accessible in the script by using the params property.


#### The Benchmark's Description Configuration

The following is extracted from the benchmarks description configuration.


```yaml
- code: "STB"
  desc: "Test Benchmark: Simple Test Benchmark"
  scripted: true
  multiple_robots: false
  commands_devices: false
  order: 1
  default_goal_timeout: 30.0
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
```

To know more about the benchmarks description, you can read [this documentation](/rsbb_etc/doc/configuration/benchmarks_description.md).

#### The Benchmark's Configuration

The configuration for the Simple Test Benchmark is only an example, as we do not need any parameters during the execution.

The following is the content of the file rsbb_bmbox/scripts/benchmark_configs/STB.yaml
```yaml
a: AAA
b: "BBB"
```

Notice that the name of the yaml file is important: the name of the configuration file must correspond to the benchmark code.


### The Code Explained

Let's break the code down.

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml
from rsbb_bmbox.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject
```
rospy must be imported to use ros features, like publishing, subscribing and logging info, warning, error and fatal messages.
yaml is very useful to serialise and deserialise objects.

GoalObject, ManualOperationObject are used to implement the benchmark and manage manual operations and goals requests.


```python
class BenchmarkObject (BaseBenchmarkObject):

	benchmark_code = "STB"

	def execute(self):
```
The benchmark is implemented by writing the code we want to execute in the execute function, defined in the class BenchmarkObject.

The implementation of this specific object is mandatory since this is the object expected by the benchmark script server.
The benchmark script will be loaded and executed from the server when requested by the RefBox.
Actually, the server starts up, the BenchmarkObject calss is imported an instance of BenchmarkObject is created, then only when the RefBox requests the execution of the benchmark, the execute function is called.

By defining the benchmark_code attribute, we let know the benchmark script server of the benchmark code for this script, so this line of code is crucial and mandatory.
Also, two benchmark scripts should not have the same benchmark_code, for obvious reasons.


```python
		N = 2
		i = 1
		execution_time = rospy.Duration(0.0)

		print "params:\n", self.params
		print "referee_score:\n", self.referee_score
```
At the beginning of the execute function we define the variables we'll need during the execution.

Also we print the parameters and the referee's score.

The referee's score is not used, but we show how to access it from the script.
Notice that we print it at the beginning of the execution, but the RefBox operator may have already changed some values, so the content of the referee's score may not be the default even at the start of the benchmark.

In the STB we do not use any parameter, we print params only to show how to access the parameters of the benchmark.
The parameters printed are the ones specified in the benchmark's configuration, [see section](#the-benchmarks-configuration).


```python
		##########################################
		#            MANUAL OPERATION            #
		##########################################

		manual_operation_first = ManualOperationObject("First Manual Operation")

		self.request_manual_operation(manual_operation_first)
```
We execute the first manual operation by creating a ManualOperationObject.
The argument of the constructor is the text that will be shown to the operator.

To execute the manual operation we call the benchmark function request_manual_operation.
This function is blocking and will only return when the manual operation has been completed by the operator, or if something exceptional happens that prevents the operation to be completed.

```python
		##########################################
		#     CHECK RESULT AND UPDATE SCORE      #
		##########################################

		if manual_operation_first.has_been_completed():
			print "First Manual Operation result: %s" % manual_operation_first.get_result()
			self.score["first_manual_operation"] = manual_operation_first.get_result()
			self.save_and_publish_score()
		else:
			print "First Manual Operation NOT EXECUTED"
			self.score["first_manual_operation"] = "not executed"
			self.save_and_publish_score()

		if not self.is_benchmark_running():
			if self.has_benchmark_timed_out():
				print "BENCHMARK TIMEOUT"
				return
		elif self.has_benchmark_been_stopped():
				print "BENCHMARK STOPPED"
				return
			else:
				print "BENCHMARK ABORTED"
				return
```
The object manual_operation_first, that we used to request the manual operation, is also used to retrieve the result of the manual operation.

To check the result of the manual operation we use the has_been_completed function.
This function returns False if the benchmark terminated and in very exceptional cases, such as a failure in the communication between RefBox and BmBox.
If has_been_completed was True, we get the string returned by get_result, that the RefBox operator inserted in the manual operation input field, and insert it in the score to log it.
Notice that the save_and_publish_score is called whenever possible, and after updating the score property.

We then check that the benchmark is still running, by calling the benchmark function is_benchmark_running.
The function has_benchmark_timed_out returns True if the global timeout occured, and the function has_benchmark_been_stopped returns True if the RefBox operator stopped the benchmark.
If is_benchmark_running was False but also both has_benchmark_timed_out and has_benchmark_been_stopped were False, then something must have gone wrong with the communication between RefBox and BmBox, but this case can be neglected.


```python
		while self.is_benchmark_running() and i <= N:

			##########################################
			#                 GOAL i                 #
			##########################################

			goal = GoalObject({"goal": "GOAL %d"%i, "details": 4}, 15.0)

			self.request_goal(goal)
			start_time = rospy.Time.now()

			print "wait_goal_result"

			self.wait_goal_result()
			end_time = rospy.Time.now()

			execution_time += end_time - start_time
			rospy.loginfo("Execution time - %f" % execution_time.to_sec())
```
Inside the while, that is executed twice, we execute a goal by creating a GoalObject.
The arguments of the constructor are the object that will be serialised and sent to the robot and the timeout of the goal.
The timeout parameter defaults to 0, and in this case the goal timeout used by the RefBox is specified by the default_goal_timeout parameter in the benchmark description.

To start the goal execution we call the benchmark function request_goal.
This function is partially blocking, meaning that it should return quite soon since it only waits for the robot to start executing the goal.
The request_goal benchmark function also returns if something exceptional happens that prevents the goal execution to be completed, like a global or goal timeout in the preparation phase (you can read about the robot communication in the documentation of the roah_rsbb_comm_ros package [here](https://github.com/rockin-robot-challenge/at_home_rsbb_comm_ros), if the RefBox operator stops the benchmark or in case of communication failure.

Calling wait_goal_result let us wait until the goal execution terminates.
This can be because the robot completed the goal, because the global or goal timeout occured, or because of a communication failure.

The start_time and end_time variables are set to the times of start and end of the goal execution, and are used to measure the goal execution time.

```python
			##########################################
			#    CHECK RESULT i AND UPDATE SCORE     #
			##########################################

			self.score["goal_%i"%i] = {}
			self.score["goal_%i"%i]["timeout"] = goal.has_timed_out()
			self.score["goal_%i"%i]["completed"] = goal.has_been_completed()
```
The score is updated with the object relative to the first and second goal, and in this object the timeout and completed bool values are logged.


```python
			if goal.has_timed_out():
				print "GOAL TIMEOUT"
			elif goal.has_been_completed():
				print "GOAL COMPLETED:"
				result = goal.get_result()
				print "result:\n", result
				self.score["goal_%i"%i]["result"] = result
			else:
				print "GOAL NOT COMPLETED"

			self.save_and_publish_score()

			if not self.is_benchmark_running():
				print self.get_end_description()

				if self.has_benchmark_timed_out():
					print "BENCHMARK TIMEOUT"
				elif self.has_benchmark_been_stopped():
					print "BENCHMARK STOPPED"
				else:
					print "BENCHMARK ABORTED"

			i += 1
```

As in the first manual operation, we check if the goal timed out.
If not, we check that the robot completed the goal by calling has_been_completed, and in such case we get the result with the get_result function.

As always we log the result of the goal in the score object and then call save_and_publish_score.

Then, again, we check that the benchmark is still running.
The get_end_description benchmark function returns a string with the description of the reason the benchmark ended, for example, because the RefBox operator stopped the benchmark or because the benchmark ended unexpectedly.
This string is only meant to be used as a human readable description and it may change in future implementations.


```python
		##########################################
		#            MANUAL OPERATION            #
		##########################################

		manual_operation_last = ManualOperationObject("Last Manual Operation")

		self.request_manual_operation(manual_operation_last)

		##########################################
		#     CHECK RESULT AND UPDATE SCORE      #
		##########################################

		if manual_operation_last.has_been_completed():
			print "Last Manual Operation result: %s" % manual_operation_last.get_result()
			self.score["last_manual_operation"] = manual_operation_last.get_result()
			self.save_and_publish_score()
		else:
			print "Last Manual Operation NOT EXECUTED"
			self.score["last_manual_operation"] = "not executed"
			self.save_and_publish_score()

		if not self.is_benchmark_running():
			if self.has_benchmark_timed_out():
				print "BENCHMARK TIMEOUT"
			elif self.has_benchmark_been_stopped():
				print "BENCHMARK STOPPED"
			else:
				print "BENCHMARK ABORTED"
```
We execute another manual operation, in the same way we did the first time, always logging the result in the score object and then calling save_and_publish_score.

Notice that the manual operation object previously used should not be "recycled", because its internal state refers to the first manual operation.


```python
		##########################################
		#            UPDATE SCORE                #
		##########################################

		self.score["execution_time"] = execution_time.to_sec()
		self.save_and_publish_score()
```
We update the score with the execution time of the goals, and call save_and_publish_score one last time.

Notice that the score will be serialised as a yaml string before being saved and published, and inserting the value of  execution_time alone would result in an ugly string specifying the rospy Duration object and its state, so we insert execution_time.to_sec() instead, that returns the duration in seconds as a float.


### Executing a Benchmark script

```
TODO: Explanation step by step of what to do to execute the benchmark and what happens relatively to the previous explanation.
```
