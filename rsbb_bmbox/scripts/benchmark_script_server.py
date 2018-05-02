#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from exceptions import Exception
import rospy

from benchmark_scripts import *
import benchmark_scripts
from rsbb_benchmarking_messages.srv import *
from rsbb_benchmarking_messages.msg import SystemStatus
from rsbb_bmbox.BenchmarkObjects import *


class BenchmarkServer(object):
	
	def __init__(self):
		
		self.scripts = {}
		self.execute_request = None
		
		self._init_benchmark_request_service_server = rospy.Service("bmbox/init_benchmark", InitBenchmark, self.execute_benchmark_callback)
		self._terminate_benchmark_request_service_server = rospy.Service("bmbox/terminate_benchmark", TerminateBenchmark, self.terminate_benchmark_callback)
		self._terminate_benchmark_request = False
		
		self._status = SystemStatus()
		self._status.status = SystemStatus.NORMAL
		self._status_pub = rospy.Publisher("rsbb_system_status/bmbox", SystemStatus, queue_size=10)
		
		rospy.init_node('rsbb_benchmark_server')
		
		self.import_benchmark_scripts()
	
	
	def execute_benchmark_callback(self, request):
		rospy.loginfo('execute_benchmark_callback: benchmark_code %s \t team: %s \t run: %i' % (request.benchmark_code, request.team, request.run))
		
		if request.benchmark_code not in self.scripts.keys():
			rospy.logerr("requested benchmark script not available")
			return InitBenchmarkResponse(False)
		
		self.execute_request = request
		
		return InitBenchmarkResponse(True)
	
	
	def terminate_benchmark_callback(self, request):
		rospy.loginfo('terminate_benchmark_callback')
		
		
		if self.execute_request == None:
			self.publish_system_status("restarting")
			self._terminate_benchmark_request = True
			return TerminateBenchmarkResponse(True)
		
		if self.execute_request == None or self._terminate_benchmark_request == True or not self.scripts[self.execute_request.benchmark_code].can_terminate_benchmark():
			return TerminateBenchmarkResponse(False)
		
		self._terminate_benchmark_request = True
		
		self.scripts[self.execute_request.benchmark_code].terminate_benchmark()
		
		return TerminateBenchmarkResponse(True)
	
	
	def publish_system_status(self, d = ""):
		self._status.header.stamp = rospy.Time.now()
		self._status.status_description = d
		self._status_pub.publish(self._status)


	def import_benchmark_scripts(self):
		
		rospy.loginfo("imported scripts:")
	
		for module_name in benchmark_scripts.__all__:
			module = globals()[module_name]
		
			try:
			
				benchmark_object = module.BenchmarkObject()
				benchmark_code = ""
			
				try:
				
					benchmark_code = benchmark_object.get_benchmark_code()
				
				except AttributeError:
					raise BenchmarkCodeNotImplementedError("benchmark_code not specified in BenchmarkObject")
				
				except TypeError:
					raise BenchmarkCodeNotImplementedError("Wrong number of attributes in get_benchmark_code method")
				
			
				if type(benchmark_code) != type(""):
					raise BenchmarkCodeNotImplementedError("benchmark_code in BenchmarkObject should be a string")
						
				self.scripts[benchmark_code] = benchmark_object
			
				rospy.loginfo("\t%s", benchmark_code)
			
			
			except BenchmarkCodeNotImplementedError as e:
				self._status.status = SystemStatus.ERROR
				self.publish_system_status("error")
				rospy.logerr("Error from script [%s]: %s", module_name, e.parameter)
	

	def run(self):
		
		rate = rospy.Rate(10) # 10Hz
		
		rospy.loginfo("\n\n\n\n\nready to execute imported benchmark scripts")
		
		while not rospy.is_shutdown():
			
			if self.execute_request:
			
				try:
					self.scripts[self.execute_request.benchmark_code].setup(self.execute_request.team, self.execute_request.run)
					self.scripts[self.execute_request.benchmark_code].wrapped_execute()
				
				except ExecuteMethodNotImplementedError as e:
					rospy.logerr("Error for execute request [%s]: method execute(self) not implemented in BenchmarkObject", self.execute_request.benchmark_code)
					rospy.logerr(e)
					self._status.status = SystemStatus.ERROR
					self.publish_system_status("error")
				
				except rospy.ROSInterruptException as e:
					rospy.loginfo("Shutdown signal received")
					rospy.loginfo(e)
					self.publish_system_status("restarting")
				
				except Exception as e:
					rospy.logerr("Exception raised in benchmark script [%s]. Benchmark script terminated", self.execute_request.benchmark_code)
					self._status.status = SystemStatus.ERROR
					self.publish_system_status("error")
					rospy.signal_shutdown("Exception raised in benchmark script")
					raise
				
				self.execute_request = None
				
				
				print "\n\n FINISHED EXECUTING SCRIPT \n\n"
			
			else:
				if self._terminate_benchmark_request:
					self.publish_system_status("restarting")
					rospy.signal_shutdown("Restarting node")
				else:
					self.publish_system_status("waiting")
			
			rate.sleep()


if __name__ == '__main__':
	try:
		server = BenchmarkServer()
		server.run()
	except rospy.ROSInterruptException:
		pass
		









