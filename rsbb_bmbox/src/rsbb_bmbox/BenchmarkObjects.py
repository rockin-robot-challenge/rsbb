#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml

from std_msgs.msg import String, Float32
from rsbb_benchmarking_messages.msg import BmBoxState, RefBoxState, SystemStatus
from rsbb_benchmarking_messages.srv import *

from FSM import FSM
from StateObserver import StateObserver

from threading import Condition, Lock, Thread
from os import path, makedirs, rename
from datetime import date, datetime
from dateutil.tz import tzlocal
from exceptions import NotImplementedError
import errno

STATE_UPDATE_RATE = 10 # 10Hz

bmbox_state_to_str = {
0: "START",
1: "WAITING_CLIENT",
2: "READY",
3: "WAITING_MANUAL_OPERATION",
4: "COMPLETED_MANUAL_OPERATION",
5: "TRANSMITTING_GOAL",
6: "EXECUTING_GOAL",
7: "WAITING_RESULT",
8: "TRANSMITTING_SCORE",
9: "END"
}

refbox_state_to_str = {
0: "START",
1: "EXECUTING_BENCHMARK",
2: "END",
3: "STOP",
4: "EMERGENCY_STOP",
5: "ERROR",
6: "GLOBAL_TIMEOUT",
7: "READY",
8: "TRANSMITTING_GOAL",
9: "EXECUTING_GOAL",
10: "GOAL_TIMEOUT",
11: "EXECUTING_MANUAL_OPERATION",
12: "NONE"
}

###############################################################
#                                                             #
###############################################################

class RefBoxComm:
	
	def __init__(self):
		
		self._refbox_state_to_str = lambda state: refbox_state_to_str[state]
		self._refbox_states_to_str = lambda list_of_states: map(self._refbox_state_to_str , list_of_states)
		
		self._bmbox_state_to_str = lambda state: bmbox_state_to_str[state]
		self._bmbox_states_to_str = lambda list_of_states: map(self._bmbox_state_to_str , list_of_states)
		
		self.__current_goal = None
		self.__current_manual_operation = None
		self.__current_referee_score = None
		
		self.__fsm = FSM(
			bmbox_state_to_str,
			(
				(BmBoxState.WAITING_CLIENT, BmBoxState.END), # allowed transitions from BmBoxState.START
				(BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_CLIENT
				(BmBoxState.WAITING_MANUAL_OPERATION, BmBoxState.TRANSMITTING_GOAL, BmBoxState.TRANSMITTING_SCORE, BmBoxState.END), # allowed transitions from BmBoxState.READY
				(BmBoxState.COMPLETED_MANUAL_OPERATION, BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_MANUAL_OPERATION
				(BmBoxState.TRANSMITTING_GOAL, BmBoxState.WAITING_MANUAL_OPERATION, BmBoxState.END), # allowed transitions from BmBoxState.COMPLETED_MANUAL_OPERATION
				(BmBoxState.EXECUTING_GOAL, BmBoxState.END), # allowed transitions from BmBoxState.TRANSMITTING_GOAL
				(BmBoxState.WAITING_RESULT, BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.EXECUTING_GOAL
				(BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_RESULT
				(BmBoxState.END, ), # allowed transitions from BmBoxState.TRANSMITTING_SCORE
				(BmBoxState.END, ), # allowed transitions from BmBoxState.END
			),
			BmBoxState.START
		)
		
		self.__exception_states = [RefBoxState.STOP, RefBoxState.ERROR, RefBoxState.GLOBAL_TIMEOUT]
		
		self.__refbox_state = RefBoxState()
		self.__status = SystemStatus()
		self.__status.status = SystemStatus.NORMAL
		
		self.__state_pub = rospy.Publisher("bmbox/bmbox_state", BmBoxState, queue_size=10)
		self.__status_pub = rospy.Publisher("rsbb_system_status/bmbox", SystemStatus, queue_size=10)
		
		self.__refbox_state_observer = StateObserver(self.__exception_states, self.__refbox_state, refbox_state_to_str)

		self.__pub_thread = Thread(name="pub_thread", target=self.__pub_thread)
		self.__pub_thread.start()
		
		self._start_benchmark_server = rospy.Service("bmbox/start_benchmark", StartBenchmark, self.__start_benchmark_callback)
		self._manual_operation_complete_server = rospy.Service("bmbox/manual_operation_complete", ManualOperationComplete, self.__manual_operation_complete_callback)
		self._goal_started_server = rospy.Service("bmbox/goal_execution_started", GoalStarted, self.__goal_started_callback)
		self._goal_complete_server = rospy.Service("bmbox/goal_complete", GoalComplete, self.__goal_complete_callback)
		self._goal_complete_server = rospy.Service("bmbox/referee_score", RefereeScore, self.__referee_score_callback)
		self._stop_benchmark_server = rospy.Service("bmbox/stop_benchmark", StopBenchmark, self.__stop_benchmark_callback)
		
		
		
		
		### TEST
		self.tmp = 0
	
	
	def __pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		while not rospy.is_shutdown():
			self.__state_pub.publish(self.__fsm.state(), self.__fsm.payload())
			self.__publish_system_status("executing")
			r.sleep()
		self.__state_pub.unregister()
	
	def __publish_system_status(self, d = ""):
		self.__status.header.stamp = rospy.Time.now()
		self.__status.status_description = d
		self.__status_pub.publish(self.__status)
	
	def __update_refbox_state(self, refbox_state):
		self.__refbox_state = refbox_state
		self.__refbox_state_observer.update(refbox_state)
		
		print "\n\nbenchmark_state:        %s\ngoal_execution_state:   %s\nmanual_operation_state: %s\n" % (self._refbox_state_to_str(self.__refbox_state.benchmark_state),\
		self._refbox_state_to_str(self.__refbox_state.goal_execution_state),\
		self._refbox_state_to_str(self.__refbox_state.manual_operation_state))
		
	
	
	def __start_benchmark_callback(self, request):
		rospy.loginfo("start_benchmark_callback")
		
		if not self.is_benchmark_running():
			rospy.loginfo("start_benchmark_callback: benchmark not running")
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.WAITING_CLIENT, BmBoxState.END], log_function = rospy.logerr):
			rospy.logwarn("start_benchmark_callback: inconsistent states or benchmark not running")
		
		self.__update_refbox_state(request.refbox_state)
		
		return StartBenchmarkResponse(True)
	
	def __goal_started_callback(self, request):
		rospy.loginfo("goal_started_callback")
		
		### TEST
#		if self.tmp < 5:
#			self.tmp += 1
#			print "TEST __goal_started_callback: return GoalStartedResponse(False)"
#			return GoalStartedResponse(False)

		if not self.is_benchmark_running():
			rospy.loginfo("goal_started_callback: benchmark not running")
		
		if self.__current_goal == None:
			rospy.logerr("goal_started_callback: cannot accept goal execution started event. No goal execution is pending")
			self.__update_refbox_state(request.refbox_state)
			return GoalStartedResponse(False)
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.TRANSMITTING_GOAL, BmBoxState.END], log_function = rospy.logerr):
			rospy.logwarn("goal_started_callback: inconsistent states or benchmark not running")
		
		self.__update_refbox_state(request.refbox_state)
		
		return GoalStartedResponse(True)
	
	def __goal_complete_callback(self, request):
		rospy.loginfo("goal_complete_callback")
		
		### TEST
#		if self.tmp < 5 and request.refbox_state.benchmark_state == RefBoxState.GLOBAL_TIMEOUT:
#			self.tmp += 1
#			print "TEST __goal_complete_callback (GLOBAL_TIMEOUT): return GoalCompleteResponse(False)"
#			return GoalCompleteResponse(False)
		
		if not self.is_benchmark_running():
			rospy.loginfo("goal_complete_callback: benchmark not running")
		
		if self.__current_goal == None:
			rospy.logerr("goal_complete_callback: cannot accept goal complete event. No goal execution is pending")
			self.__update_refbox_state(request.refbox_state)
			return GoalCompleteResponse(False)
		else:
			if request.goal_timeout:
				self.__current_goal.set_has_timed_out()
			else:
				self.__current_goal.set_result_string(request.goal_result)
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.TRANSMITTING_GOAL, BmBoxState.EXECUTING_GOAL, BmBoxState.WAITING_RESULT, BmBoxState.END], log_function = rospy.logerr):
			rospy.logwarn("goal_complete_callback: inconsistent states or benchmark not running")
		
		self.__update_refbox_state(request.refbox_state)
		
		return GoalCompleteResponse(True)
	
	def __referee_score_callback(self, request):
		rospy.loginfo("referee_score_callback")
		
		try:
			self.__current_referee_score = yaml.load(request.score)
			self.save_and_publish_score()
		except yaml.YAMLError as e:
			rospy.logerr("referee_score_callback: YAMLError while parsing refere score yaml string\n%s"%e)
		
		return RefereeScoreResponse(True)
	
	def __manual_operation_complete_callback(self, request):
		rospy.loginfo("manual_operation_complete_callback")
		
		### TEST
#		if self.tmp > 5:
#			self.tmp += 1
#			print "TEST __manual_operation_complete_callback: return ManualOperationCompleteResponse(False)"
#			return ManualOperationCompleteResponse(False)
		
		if not self.is_benchmark_running():
			rospy.loginfo("manual_operation_complete_callback: benchmark not running")
		
		if self.__current_manual_operation == None:
			rospy.logerr("manual_operation_complete_callback: cannot accept manual operation result. No manual operation is pending")
			self.__update_refbox_state(request.refbox_state)
			return ManualOperationCompleteResponse(False)
		else:
			self.__current_manual_operation.set_result(request.manual_operation_result)
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.WAITING_MANUAL_OPERATION, BmBoxState.END], log_function = rospy.logerr):
			rospy.logwarn("manual_operation_complete_callback: inconsistent states or benchmark not running")
		
		self.__update_refbox_state(request.refbox_state)
		
		return ManualOperationCompleteResponse(True)
	
	def __stop_benchmark_callback(self, request):
		rospy.loginfo("stop_benchmark_callback")
		
		self.__update_refbox_state(request.refbox_state)
		
		return StopBenchmarkResponse(True)
	
	
	
	
	def _wait_refbox_connection(self):
		rospy.logdebug("RefBoxComm._wait_refbox_connection()")
		
		if not self.is_benchmark_running():
			rospy.loginfo("wait_refbox_connection: benchmark not running")
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.START, BmBoxState.END], log_function = rospy.logerr):
			rospy.logerr("_wait_refbox_connection: can not execute request")
			return
		
		self.__fsm.update(BmBoxState.WAITING_CLIENT)
		
		# woken by start_benchmark_callback
		self.__refbox_state_observer.wait_benchmark_state_transition(from_state = RefBoxState.START, to_states = [RefBoxState.EXECUTING_BENCHMARK])
		
		### normal post conditions:
		self.__fsm.update(BmBoxState.READY)
		self.__current_manual_operation = None
		return
	
	def request_manual_operation(self, manual_operation_object=None):
		"""
		Requests a manual operation to the refbox.
		A manual operation consists of a request to the refbox operator to do something.
		It is possible to prompt the refbox operator to write a response, that will be returned as a string.
		This function is blocking, meaning the function will only return after the manual operation is complete.
		:param ManualOperationObject manual_operation_object: The manual operation object, containing the request and the result of the of the manual operation. The result is set after the manual operation has been completed by the refbox operator.
		"""
		
		rospy.logdebug("RefBoxComm.request_manual_operation()")
		
		if not isinstance(manual_operation_object, ManualOperationObject):
			rospy.logerr("request_manual_operation: not isinstance(manual_operation_object, ManualOperationObject)")
			return
		
		if not self.is_benchmark_running():
			rospy.loginfo("request_manual_operation: benchmark not running")
			return
		
		if self.__current_manual_operation != None:
			rospy.logerr("request_manual_operation: another manual operation request is pending")
			return
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.READY, BmBoxState.END], manual_operation_states=[RefBoxState.READY]):
			rospy.loginfo("request_manual_operation: can not execute request")
			return
		
		
		self.__current_manual_operation = manual_operation_object
		
		self.__fsm.update(BmBoxState.WAITING_MANUAL_OPERATION, self.__current_manual_operation.get_request())
		
		try:
			
			execute_manual_operation = rospy.ServiceProxy("bmbox/execute_manual_operation", ExecuteManualOperation)
			manual_operation_payload = String(data = self.__current_manual_operation.get_request())
			response = execute_manual_operation(manual_operation_payload)
			
			if response.result.data:
				
				self.__update_refbox_state(response.refbox_state)
				
				self.__refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.EXECUTING_MANUAL_OPERATION])
				self.__refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.EXECUTING_MANUAL_OPERATION, to_states = [RefBoxState.READY])
			
			else:
				rospy.logerr("request_manual_operation: Manual operation FAILED (refbox refused to execute the manual operation)")
				
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s" % (e))
		
		### normal post conditions:
		self.__fsm.update(BmBoxState.READY)
		self.__current_manual_operation = None
		return
	
	

	def start_manual_operation(self, manual_operation_object=None):
		"""
		Requests a manual operation to the refbox.
		A manual operation consists of a request to the refbox operator to do something.
		It is possible to prompt the refbox operator to write a response, that will be returned as a string when the manual operation is completed.
		This function is non-blocking, meaning the function will return immediatly.
		:param ManualOperationObject manual_operation_object: The manual operation object, containing the request and the result of the of the manual operation. The result is set after the manual operation has been completed by the refbox operator.
		"""
		
		rospy.logdebug("RefBoxComm.start_manual_operation()")
		
		if not isinstance(manual_operation_object, ManualOperationObject):
			rospy.logerr("start_manual_operation: not isinstance(manual_operation_object, ManualOperationObject)")
			return
		
		if not self.is_benchmark_running():
			rospy.loginfo("start_manual_operation: benchmark not running")
			return
		
		if self.__current_manual_operation != None:
			rospy.logerr("start_manual_operation: another manual operation request is pending")
			return
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.READY, BmBoxState.END], manual_operation_states=[RefBoxState.READY]):
			rospy.loginfo("start_manual_operation: can not execute request")
			return
		
		
		self.__current_manual_operation = manual_operation_object
		
		self.__fsm.update(BmBoxState.WAITING_MANUAL_OPERATION, self.__current_manual_operation.get_request())
		
		try:
			
			execute_manual_operation = rospy.ServiceProxy("bmbox/execute_manual_operation", ExecuteManualOperation)
			manual_operation_payload = String(data = self.__current_manual_operation.get_request())
			response = execute_manual_operation(manual_operation_payload)
			
			if response.result.data:
				
				self.__update_refbox_state(response.refbox_state)
				
				self.__refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.EXECUTING_MANUAL_OPERATION])
#				self.__refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.EXECUTING_MANUAL_OPERATION, to_states = [RefBoxState.READY])
			
			else:
				rospy.logerr("start_manual_operation: Manual operation FAILED (refbox refused to execute the manual operation)")
				
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s" % (e))
		
		### normal post conditions:
#		self.__fsm.update(BmBoxState.READY)
#		self.__current_manual_operation = None
		return

	def wait_manual_operation(self):
		"""
		Waits until the manual operation is completed.
		This function is blocking.
		"""
		rospy.logdebug("RefBoxComm.wait_manual_operation()")
		
		if not self.is_benchmark_running():
			rospy.loginfo("wait_manual_operation: benchmark not running")
			return
		
		if self.__current_manual_operation == None: #? or self.__current_manual_operation.has_been_completed():
			rospy.logerr("wait_manual_operation: no manual operation request is pending")
			return
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.WAITING_MANUAL_OPERATION, BmBoxState.END], manual_operation_states=[RefBoxState.EXECUTING_MANUAL_OPERATION, RefBoxState.READY]):
			rospy.loginfo("wait_manual_operation: can not execute request")
			return
		
		
		self.__refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.EXECUTING_MANUAL_OPERATION, to_states = [RefBoxState.READY])
		
		
		### normal post conditions:
		self.__fsm.update(BmBoxState.READY)
		self.__current_manual_operation = None
		return
		
		
		
	
	
	
	def request_goal(self, goal_object):
		"""
		Requests the refbox to send a goal to the robot.
		A goal consists of a request to the robot to do something and must be consistent with the data types of the communication node on the robot.
		If the robot provides some result data, the result will be set in the goal object.
		This function is blocking until the robot receives the goal request.
		:param GoalObject goal_object: The goal object, containing the request, the timeout for the execution of the goal, the result provided by the robot. The result is set after the goal has been completed by the robot unless the timeout occured.
		"""
		rospy.logdebug("RefBoxComm.request_goal()")
		
		if not isinstance(goal_object, GoalObject):
			rospy.logerr("request_goal: not isinstance(goal_object, GoalObject)")
			return
		
		if not self.is_benchmark_running():
			rospy.loginfo("request_goal: benchmark not running")
			return
		
		if self.__current_goal != None:
			rospy.logerr("request_goal: another goal request is pending")
			return
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.READY, BmBoxState.END], goal_execution_states=[RefBoxState.READY]):
			rospy.loginfo("request_goal: can not execute request")
			return
		
		
		self.__current_goal = goal_object
		
		self.__fsm.update(BmBoxState.TRANSMITTING_GOAL, self.__current_goal.get_request_string())
		
		try:
			
			execute_goal = rospy.ServiceProxy("bmbox/execute_goal", ExecuteGoal)
			
			goal_payload = String(data = self.__current_goal.get_request_string())
			timeout_payload = Float32(data = self.__current_goal.get_timeout())
			
			response = execute_goal(goal_payload, timeout_payload)
			
			if response.result.data:
				
				self.__update_refbox_state(response.refbox_state)
				
				# should return immediately
				self.__refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.TRANSMITTING_GOAL])
				
				# woken by goal_started_callback RefBox state update
				self.__refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.TRANSMITTING_GOAL, to_states = [RefBoxState.EXECUTING_GOAL, RefBoxState.READY])
			
			else:
				rospy.logerr("request_goal: Goal request FAILED (refbox refused to execute the goal)")
				
		except rospy.ServiceException, e:
			rospy.logerr("request_goal: Service call failed: %s" % (e))
		
		
		### normal post conditions:
		self.__fsm.update(BmBoxState.EXECUTING_GOAL, self.__current_goal.get_request_string())
		return
	
	
	def wait_goal_result(self):
		"""
		Waits until either the robot completes the goal or the goal times out.
		This function is blocking.
		"""
		rospy.logdebug("RefBoxComm.wait_goal_result()")
		
		if not self.is_benchmark_running():
			rospy.loginfo("wait_goal_result: benchmark not running")
			return
		
		if self.__current_goal == None:
			rospy.logerr("wait_goal_result: no goal request is pending")
			return
		
		if not self.check_preconditions(bmbox_states=[BmBoxState.EXECUTING_GOAL, BmBoxState.END], goal_execution_states=[RefBoxState.EXECUTING_GOAL, RefBoxState.READY]):
			rospy.loginfo("wait_goal_result: can not execute request")
			return
		
		self.__fsm.update(BmBoxState.WAITING_RESULT)
		self.__refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.EXECUTING_GOAL, to_states = [RefBoxState.READY])
		
		### normal post conditions:
		self.__fsm.update(BmBoxState.READY)
		self.__current_goal = None
		return
	
	
	def end_benchmark(self):
		"""
		Ends the benchmark.
		This function is non-blocking.
		"""
		rospy.logdebug("RefBoxComm.end_benchmark()")
		
		self.save_and_publish_score()
		
		if not self.is_benchmark_running():
			rospy.loginfo("end_benchmark: benchmark already not running")
			return
		
		if not self.check_preconditions(\
		       bmbox_states=[BmBoxState.READY, BmBoxState.END], \
		       benchmark_states=[RefBoxState.EXECUTING_BENCHMARK], \
		       goal_execution_states=[RefBoxState.READY], \
		       manual_operation_states=[RefBoxState.READY]):
			rospy.loginfo("end_benchmark: can not execute request")
			return
		
		
		try:
			
			end_benchmark = rospy.ServiceProxy("bmbox/end_benchmark", EndBenchmark)
			end_benchmark_payload = String()
			response = end_benchmark(end_benchmark_payload)
			
			if response.result.data:
				
				self.__update_refbox_state(response.refbox_state)
				
				self.__refbox_state_observer.wait_benchmark_state_transition(from_state = RefBoxState.EXECUTING_BENCHMARK, to_states = [RefBoxState.END])
				
				self.save_and_publish_score()
				
			else:
				rospy.logerr("end_benchmark: request FAILED (refbox refused to end the benchmark)")
		
		except rospy.ServiceException, e:
			rospy.logerr("end_benchmark: service call failed: %s" % (e))
		
		
		### normal post conditions:
		self.__fsm.update(BmBoxState.END)
		return
	
	def terminate_benchmark(self):
		
		self.save_and_publish_score()
		
		if self.can_terminate_benchmark():
			
			rospy.loginfo("calling signal_shutdown(\"Benchmark terminated\")")
			rospy.signal_shutdown("Benchmark terminated")
			self.__fsm.notify_condition_variables()
			self.__refbox_state_observer.notify_condition_variables()
		
		self.__fsm.update(BmBoxState.END)
		self.__fsm.notify_condition_variables()
		self.__refbox_state_observer.notify_condition_variables()

	
	
	def __is_waiting_to_start(self):
		rospy.logdebug("RefBoxComm.__is_waiting_to_start()")
		return self.__fsm.state() in [BmBoxState.START, BmBoxState.WAITING_CLIENT]
	
	def check_preconditions(self, bmbox_states = None, benchmark_states = None, goal_execution_states = None, manual_operation_states = None, log_function = rospy.logwarn):
		
		if not (bmbox_states == None or self.__fsm.state() in bmbox_states):
			log_function("check_preconditions: inconsistent bmbox states. expected [%s], current state [%s]", self._bmbox_states_to_str(bmbox_states), self._bmbox_state_to_str(self.__fsm.state()) )
		
		if not (benchmark_states == None or self.__refbox_state.benchmark_state in benchmark_states):
			log_function("check_preconditions: inconsistent benchmark states. expected [%s], current state [%s]", self._refbox_states_to_str(benchmark_states), self._refbox_state_to_str(self.__refbox_state.benchmark_state) )
		
		if not (goal_execution_states == None or self.__refbox_state.goal_execution_state in goal_execution_states):
			log_function("check_preconditions: inconsistent goal execution states. expected [%s], current state [%s]", self._refbox_states_to_str(goal_execution_states), self._refbox_state_to_str(self.__refbox_state.goal_execution_state) )
		
		if not (manual_operation_states == None or self.__refbox_state.manual_operation_state in manual_operation_states):
			log_function("check_preconditions: inconsistent manual operation states. expected [%s], current state [%s]", self._refbox_states_to_str(manual_operation_states), self._refbox_state_to_str(self.__refbox_state.manual_operation_state) )
		
		return self.is_benchmark_running() \
		and   (bmbox_states  == None or self.__fsm.state() in bmbox_states) \
		and   (benchmark_states == None or self.__refbox_state.benchmark_state in benchmark_states) \
		and   (goal_execution_states == None or self.__refbox_state.goal_execution_state in goal_execution_states) \
		and   (manual_operation_states == None or self.__refbox_state.manual_operation_state in manual_operation_states)
	
	def is_benchmark_running(self):
		"""
		Returns true if the benchmark is running, meaning that the benchmerk has not ended. The benchmark may not be running because the refbox operator stopped the benchmark or because the benchmark's timeout (global timeout) has occured. The function also returns False if the roscore of the node executing this benchmark has received the shutdown signal.
		This function is non-blocking.
		"""
		return not ( \
		       rospy.is_shutdown() \
		or     self.__refbox_state.benchmark_state in self.__exception_states \
		or     self.__refbox_state.goal_execution_state in self.__exception_states \
		or     self.__refbox_state.manual_operation_state in self.__exception_states \
		or     self.__fsm.state() == BmBoxState.END )
	
	def is_goal_timed_out(self):
		"""
		This function is deprecated.
		"""
		rospy.logdebug("RefBoxComm.is_goal_timed_out()")
		rospy.logwarn("is_goal_timed_out: deprecated")
		return False
	
	def has_benchmark_timed_out(self):
		"""
		Returns true if the global timeout has occured. The global timeout is set in the configuration of the benchmark.
		This function is non-blocking.
		"""
		return self.__refbox_state.benchmark_state == RefBoxState.GLOBAL_TIMEOUT
	
	def has_benchmark_been_stopped(self):
		"""
		Returns true if the refbox operator stopped the benchmark.
		This function is non-blocking.
		"""
		return self.__refbox_state.benchmark_state == RefBoxState.STOP
	
	def get_end_description(self):
		"""
		Returns a string containing the description of the reason the benchmark is not running.
		This function is non-blocking.
		"""
		if   self.__refbox_state.benchmark_state == RefBoxState.END:            return 'END: benchmark ended normally'
		elif self.__refbox_state.benchmark_state == RefBoxState.STOP:           return 'STOP: benchmark stopped by referee'
		elif self.__refbox_state.benchmark_state == RefBoxState.EMERGENCY_STOP: return 'EMERGENCY_STOP: benchmark stopped due to emergency'
		elif self.__refbox_state.benchmark_state == RefBoxState.ERROR:          return 'ERROR: benchmark terminated due to RefBox error'
		elif self.__refbox_state.benchmark_state == RefBoxState.GLOBAL_TIMEOUT: return 'GLOBAL_TIMEOUT: benchmark ended due to global timeout'
		else:                                                                   return 'still running or unknown reason'
	
	def can_terminate_benchmark(self):
		return not self.is_benchmark_running() or self.__is_waiting_to_start()
	
	def get_referee_score(self):
		"""
		Returns an object with the scoring of the refbox operator.
		This function is non-blocking.
		"""
		return self.__current_referee_score




###############################################################
#                                                             #
###############################################################

class BenchmarkCodeNotImplementedError (NotImplementedError):
	def __init__(self, value):
		self.parameter = value
class ExecuteMethodNotImplementedError (NotImplementedError):
	def __init__(self, value):
		self.parameter = value

def trim(s):
	return s.strip()

def is_valid_dir_path(p):
	return path.exists(p) and path.isdir(p)

def check_and_make_dir(p):
	
	try:
		makedirs(p)
		
	except OSError as e:
	
		if e.errno != errno.EEXIST:
			raise
		else:
		
			if path.isdir(p):
				return True
			elif path.isfile(p):
				
				rospy.logwarn("Renaming file [%s] to create directory" % (p))
				rename(p, p + "_renamed_file")
				
				try:
					makedirs(p)
				except OSError as e:
					rospy.logerr("Directory [%s] could not be created. Please, manually create the directory." % (p))
					raise
				
				return True
				
			else:
				rospy.logerr("Directory [%s] could not be created. Please, manually create the directory." % (p))
				raise
	
	return True


class BaseBenchmarkObject (RefBoxComm, object):
	
	def __init__(self):
		
		self.__date_string_format = "%Y-%m-%d_%H:%M:%S_%Z(%z)"
		
		self.__result_publisher = rospy.Publisher("current_benchmark_result", String, queue_size=10, latch=True)
		
		self.__result_object = {'benchmark_info': {'team': "undefined", 'run': 0, 'benchmark_code': "undefined", 'end_description': "undefined", 'params':"undefined"}, 'score': {}}
		self.__benchmark_config_object = None
		
		self.__result_base_path = None
		self.__result_filename = None
		
		
#		try:
#			benchmark_configs_directory = rospy.get_param("~benchmark_configs_directory")
#			rospy.logdebug("benchmark_configs_directory: %s" % (benchmark_configs_directory))
#			
#			benchmark_configs_path = path.normpath(path.expanduser(benchmark_configs_directory))
#			
#			benchmark_config_path = path.join(benchmark_configs_path, "%s.yaml" % (self.get_benchmark_code()))
#			rospy.logdebug("benchmark_config_path: %s" % (benchmark_config_path))
#			
#			self.__result_object['benchmark_info']['params'] = "None"
#			
#			try:
#				with open(benchmark_config_path, 'r') as benchmark_config_file:
#					try:
#						self.__benchmark_config_object = yaml.load(benchmark_config_file)
#						self.__result_object['benchmark_info']['params'] = self.__benchmark_config_object
#					except yaml.YAMLError as e:
#						rospy.logerr("Raised YAML exception while loading benchmark script [%s]. Configuration file [%s] not valid.\n%s" % (self.get_benchmark_code(), benchmark_config_path, e))
#				
#			except IOError:
#				rospy.loginfo("No configuration file [%s] found for benchmark script [%s]" % (benchmark_config_path, self.get_benchmark_code()))
#			
#		except KeyError:
#			rospy.logerr("parameter benchmark_configs_directory not set in the configuration")
#			raise
	
		
		try:
			self.__result_base_path = rospy.get_param("~base_results_directory")
		except KeyError:
			rospy.logerr("parameter base_results_directory not set in the configuration")
			raise
	
	
	@property
	def score(self):
		"""score property."""
		return self.__get_current_score()
	
	@score.setter
	def score(self, value):
		self.__set_current_score(value)
	
	@score.deleter
	def score(self):
		rospy.logerr("it is not allowed to delete the score object")
	
	
	@property
	def referee_score(self):
		"""referee score property."""
		return self.get_referee_score()
	
	@referee_score.setter
	def referee_score(self, value):
		rospy.logerr("it is not allowed to modify the referee score object")
	
	@referee_score.deleter
	def referee_score(self):
		rospy.logerr("it is not allowed to delete the referee score object")
	
	
	@property
	def params(self):
		"""params property."""
		return self.__benchmark_config_object
	
	@params.setter
	def params(self, value):
		rospy.logwarn("params object should not be modified")
		self.__benchmark_config_object = value
	
	@params.deleter
	def params(self):
		rospy.logerr("it is not allowed to delete the params object")
	
	
	def __write_result_file(self):
				
		### Create directories and initialise the score file with the benchmark info
		
		# base path
		base_path = path.normpath(path.expanduser(self.__result_base_path))
		
		# check base path
		if not is_valid_dir_path(base_path):
			rospy.logerr("Base directory [%s] does not exist. Please create the directory or update the base_directory parameter with the directory where the score should be saved" % (base_path))
			raise Exception
		
		# result directory and path
		#   note that this directory is necessary to ensure that the logs and the result files are saved in different directories,
		#   avoiding the race condition on the check and creation of directories
		results_base_dir = "results"
		results_base_path = path.join(base_path, results_base_dir)
		
		# benchmark directory and path
		bm_dir = trim(self.get_benchmark_code())
		bm_path = path.join(results_base_path, bm_dir)
	
		# check benchmark path
		if not check_and_make_dir(bm_path):
			raise Exception
		
		# team directory and path
		team_dir = trim(self.get_benchmark_team())
		team_path = path.join(bm_path, team_dir)
	
		# check team path
		if not check_and_make_dir(team_path):
			raise Exception
		
		# result filename and path of the result file
		result_filename_path = path.join(team_path, self.__result_filename)
		
		with open(result_filename_path, 'w') as outfile:
			yaml.dump(self.__result_object, outfile, default_flow_style=False)
	
	def save_and_publish_score(self):
		"""
		Writes the result (containing the score) to a file and publishes it
		"""
		self.__result_object['benchmark_info']['end_description'] = self.get_end_description()
		self.__result_object['benchmark_info']['end_time'] = datetime.now(tzlocal()).strftime(self.__date_string_format)
		self.__result_object['referee_score'] = self.get_referee_score()
		self.__write_result_file()
		if not rospy.is_shutdown():
			try:
				self.__result_publisher.publish(String(data = yaml.safe_dump(self.__result_object, default_flow_style=False)))
			except rospy.ROSException:
				print "Couldn't publish result"
	
	
	
	
	def __set_current_score(self, current_score):
		self.__result_object['score'] = current_score

	def __get_current_score(self):
		return self.__result_object['score']
	
	
	def get_benchmark_run(self):
		"""
		Returns the run for the current benchmark, obtained from the RefBox
		"""
		return self.__result_object['benchmark_info']['run']
	
	def get_benchmark_team(self):
		"""
		Returns the team name for the current benchmark, obtained from the RefBox
		"""
		return self.__result_object['benchmark_info']['team']

	
	def setup(self, team, run):
		
		# load the cofiguration parameters of the benchmark script
		self.__result_object['benchmark_info']['params'] = "None"
		
		try:
			benchmark_configs_directory = rospy.get_param("~benchmark_configs_directory")
			rospy.logdebug("benchmark_configs_directory: %s" % (benchmark_configs_directory))
			
			benchmark_configs_path = path.normpath(path.expanduser(benchmark_configs_directory))
			
			benchmark_config_path = path.join(benchmark_configs_path, "%s.yaml" % (self.get_benchmark_code()))
			rospy.logdebug("benchmark_config_path: %s" % (benchmark_config_path))
			
			try:
				with open(benchmark_config_path, 'r') as benchmark_config_file:
					try:
						self.__benchmark_config_object = yaml.load(benchmark_config_file)
						self.__result_object['benchmark_info']['params'] = self.__benchmark_config_object
					except yaml.YAMLError as e:
						rospy.logerr("Raised YAML exception while loading benchmark script configuration. Configuration file [%s] not valid.\n%s" % (self.get_benchmark_code(), benchmark_config_path, e))
		
			except IOError:
				rospy.loginfo("No configuration file [%s] found for benchmark script [%s]" % (benchmark_config_path, self.get_benchmark_code()))
			
		except KeyError:
			rospy.logerr("parameter benchmark_configs_directory not set in the configuration")
			raise
		
		# initialise the RefBox communication object
		RefBoxComm.__init__(self)
		
		# insert the benchmark informations in the yaml result object
		datetime_string = datetime.now(tzlocal()).strftime(self.__date_string_format)
		self.__result_object['benchmark_info']['benchmark_code'] = self.get_benchmark_code()
		self.__result_object['benchmark_info']['team'] = team
		self.__result_object['benchmark_info']['run'] = run
		self.__result_object['benchmark_info']['end_description'] = "initialised"
		self.__result_object['benchmark_info']['start_time'] = datetime_string
		
		self.__result_filename = "result_run_%i_%s.yaml" % (self.get_benchmark_run(), datetime_string)
		
		self.__write_result_file()
	
	
	def wrapped_execute(self):
		
		# wait for refbox
		self._wait_refbox_connection()
		
		if self.is_benchmark_running():
			self.execute()
		
		self.end_benchmark()
		
	
	def execute(self):
		raise ExecuteMethodNotImplementedError("")
		
	def get_benchmark_code(self):
		
		return self.benchmark_code


class GoalObject:
	def __init__(self, request = None, timeout = 0):
		self._request = request
		self._timeout = timeout
		self._executed = False
		self._result  = None
		self._has_timed_out = None
	
	def get_request_string(self):
		return yaml.dump(self._request)
	
	def get_timeout(self):
		return self._timeout
	
	def has_been_completed(self):
		return self._executed and not self._has_timed_out
	
	def has_timed_out(self):
		return self._has_timed_out
	
	def set_has_timed_out(self):
		self._has_timed_out = True
		self._executed = True
	
	def get_result(self):
		if self._executed:
			return self._result
		else:
			return None
	
	def set_result_string(self, result_yaml_string):
		self._result = yaml.load(result_yaml_string)
		self._has_timed_out = False
		self._executed = True


class ManualOperationObject:
	def __init__(self, request):
		self._request = request
		self._executed = False
		self._result  = None
	
	def get_request(self):
		return self._request
	
	def has_been_completed(self):
		return self._executed
	
	def get_result(self):
		if self._executed:
			return self._result
		else:
			return None
	
	def set_result(self, result):
		self._result = result
		self._executed = True


