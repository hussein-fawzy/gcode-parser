import time

class ExecutionControl(object):

	_DEBUG = False
	_DEBUG_LEVEL = 1 # a debug message will be printed if its level is smaller than or equal to _DEBUG_LEVEL

	def __init__(self):
		self.timers = {}		#units are seconds
		self._timers_start = {}	#time when when the timer is started to measure the execution using start_timer and end_timer
		self._timers_count = {}	#track how many times a timer was used

	def add_timer(self, timer_name):
		if timer_name in self.timers:
			return

		self.timers[timer_name] = 0.0
		self._timers_start[timer_name] = 0.0
		self._timers_count[timer_name] = 0

	def remove_timer(self, timer_name):
		if timer_name not in self.timers:
			return

		self.timers.pop(timer_name)
		self._timers_start.pop(timer_name)
		self._timers_count.pop(timer_name)

	def time_execution(self, timer_name, function, *function_args):
		if timer_name not in self.timers:
			self.add_timer(timer_name)

		start = time.time()
		result = function(*function_args)
		end = time.time()

		self.timers[timer_name] += round(end - start, 2)
		self._timers_count[timer_name] += 1

		return result

	def start_timer(self, timer_name):
		if timer_name not in self.timers:
			return

		self._timers_start[timer_name] = time.time()

	def end_timer(self, timer_name):
		if timer_name not in self.timers:
			return

		#timer was not started
		if self._timers_start[timer_name] == 0.0:
			return

		end = time.time()
		execution_time = end - self._timers_start[timer_name]

		self.timers[timer_name] += execution_time
		self._timers_count[timer_name] += 1


	def get_time(self, timer_name):
		if timer_name not in self.timers:
			return

		return round(self.timers[timer_name], 2)

	def print_times(self):
		if len(self.timers) == 0:
			return

		print("")
		print("Timers")

		for t in self.timers:
			if self._timers_count[t] == 0:
				print(str(t) + ": 0s")

				continue

			total_time = round(self.timers[t], 2)
			average_time = round(self.timers[t] * 1000000 / self._timers_count[t], 2)

			print(str(t) + ":",  str(total_time) + "s", "- Count:", self._timers_count[t], "- Avg:", str(average_time) + "us")

		print("")

	######################### Static Methods ##################################

	def Debug(*messages, level = 1, force_debug = False):
		""" print a message if debugging is enabled """

		if (ExecutionControl._DEBUG or force_debug) and level <= ExecutionControl._DEBUG_LEVEL:
			print(*messages)
			