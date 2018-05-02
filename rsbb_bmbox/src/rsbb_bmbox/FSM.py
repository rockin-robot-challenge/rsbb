from threading import Condition, Lock

import rospy

class FSM:
	def _check_transition(self, new_state):
		if new_state not in self._transitions[self._state]:
			rospy.logerr("State transition not allowed: %s -> %s", self._states[self._state], self._states[new_state]);
			return False
		
		return True
	
	
	def acquire(self):
		self._lock.acquire()


	def release(self):
		self._lock.release()


	def state(self):
		return self._state


	def payload(self):
		return self._payload
	
	
	def payload(self):
		return self._payload
	
	
	def check_state(self, state):
		if self._state != state:
			rospy.logerr("Wrong state: expected %s, observed %s", self._states[state], self._states[self._state]);
			return False

		return True
		

	def wait_transition(self, from_state, to_state):
		self._condvar.acquire()
		while (from_state != None) and (self._state == from_state):
			self._condvar.wait()
		while (to_state != None) and self._state != to_state:
			self._condvar.wait()
		self._condvar.release()

	def notify_condition_variables(self):
		self._condvar.acquire()
		self._condvar.notifyAll()
		self._condvar.release()

	def update(self, new_state, payload = None):
		if not self._check_transition(new_state):
			return

		rospy.logdebug("State transition: %s -> %s", self._state, new_state);
		rospy.loginfo(">>> %s", self._states[new_state]);
		self._condvar.acquire()
		self._state = new_state
		self._payload = payload
		self._condvar.notify()
		self._condvar.release()
	
	
	def __init__(self, states, transitions, init_state):
		self._states = states
		self._payload = ""
		self._transitions = transitions
		self._state = init_state
		
		self._condvar = Condition()
