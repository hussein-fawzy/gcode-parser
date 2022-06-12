import geometry
import math

from collections import deque
from executioncontrol import ExecutionControl


class GCodeParser(object):

	_COMMENT_CHAR = ";"
	_IDENTIFIER_NOT_FOUND = "IDENTIFIER_NOT_FOUND"
	_SEGMENT_BREAK_IDENTIFIERS = ["M140", "M109"] #identifiers that, if encountered, set the next_segment as no-motion-segment (stops end-effector motion)

	def __init__(self, file, machine):
		self.file = file
		self.machine = machine

		#execution control
		self.ec = ExecutionControl()

		self.ec.add_timer("FileRead")
		self.ec.add_timer("InitSegment")
		self.ec.add_timer("CopySegment")
		self.ec.add_timer("SetNextSegment")
		self.ec.add_timer("FeedrateChangeJerk")
		self.ec.add_timer("CalculateSegmentMotionTime")

		#file parsing
		self._lines_buffer_length = 20	#line buffer
		self._lines_buffer = deque()	#list containing buffered lines

		#init params and control variables
		self._feed = 0.0		#current feed in mm/second
		self._abs_coord = True	#use absolute coordinates

		self._continuous_segment = Segment1D()
		self._continuous_motion_ended = True

		self.ec.start_timer("InitSegment")
		self._last_segment = Segment3D(self.machine, Segment3D.AXES_ZERO, 0)
		self.ec.end_timer("InitSegment")

		self.ec.start_timer("InitSegment")
		self._curr_segment = Segment3D(self.machine, Segment3D.AXES_ZERO, 0)
		self.ec.end_timer("InitSegment")

		self.ec.start_timer("InitSegment")
		self._next_segment = Segment3D(self.machine, Segment3D.AXES_ZERO, 0)
		self.ec.end_timer("InitSegment")

		#monitored parameters
		self.time = 0.0					#execution time in seconds
		self.time_a = 0.0				#total acceleration time in seconds
		self.time_d = 0.0				#total deceleration time in seconds


	def parse(self):
		""" 
		parse GCode file line-by-line
		"""

		self.ec.start_timer("FileRead")
		fp = open(self.file)
		self.ec.end_timer("FileRead")
		
		#fill buffer
		for i in range(self._lines_buffer_length):
			self.ec.start_timer("FileRead")
			line = fp.readline()
			self.ec.end_timer("FileRead")

			if line:
				self._lines_buffer.append(line)
			else: #file has ended
				break

		#parse
		while(len(self._lines_buffer) > 0):
			#pop from queue
			curr_line = self._lines_buffer.popleft()

			#refill queue
			self.ec.start_timer("FileRead")
			line = fp.readline()
			self.ec.end_timer("FileRead")

			if line:
				self._lines_buffer.append(line)

			#parse popped line
			self.parse_line(curr_line)

		fp.close()

	def parse_line(self, line):
		"""
		parse GCode line
		"""

		line = GCodeParser.clean_line(line)

		#G28: home all axes
		if line.startswith("G28"):
			self.home_axes()

		#G92: set axes value
		elif line.startswith("G92"):
			self.set_axis(line)

		#M6: change tool
		elif line.startswith("M6"):
			self.change_tool(line)

		#G0: move axes with max feed, G1: move axes with a certain feed
		elif line.startswith("G1") or line.startswith("G0"):
			self.move_axis(line)

		#parse codes that may co-exist on the same line
		else:
			for curr_code in line.split(" "):
				#G90: use absolute coordinates
				if curr_code == "G90":
					self._abs_coord = True

				#G91: use relative coordinates
				elif curr_code == "G91":
					self._abs_coord = False

				#G20: set units to inches
				elif curr_code == "G20":
					pass #currently not supported

				#G21: set units to mm
				elif curr_code == "G21":
					pass #assumed to always be the case

				#F: set feedrate
				elif curr_code.startswith("F"):
					self.set_feedrate(line)


	def set_next_segment(self):
		"""
		read the buffer to find the next motion segment. should be called after setting curr_segment
		"""

		for line in self._lines_buffer:
			line = GCodeParser.clean_line(line)

			if line.startswith("G1"):
				if GCodeParser.line_has_endeffector_motion(line):
					#get feedrate
					f = GCodeParser.identifier_value(line, "F")

					if f != GCodeParser._IDENTIFIER_NOT_FOUND:
						feed = f / 60.0 #convert mm/min to mm/sec
					else:
						feed = self._feed

					#set next segment
					self.ec.start_timer("InitSegment")
					self._next_segment = Segment3D(self.machine, self._curr_segment.pos_end, feed, line, self._abs_coord)
					self.ec.end_timer("InitSegment")

					self._next_segment.adjust_to_max_feedrate()

					return
				else:
					#next_segment feed should be read from the current line.
					#however, it is set as self.feed since in this case, there is no motion and the feed value is useless (efficiency)
					self.ec.start_timer("InitSegment")
					self._next_segment = Segment3D(self.machine, self._curr_segment.pos_end, self._feed, "", self._abs_coord)
					self.ec.end_timer("InitSegment")

					return
			else:
				blocks = line.split(" ")

				if blocks[0] in GCodeParser._SEGMENT_BREAK_IDENTIFIERS:
					#next_segment feed should be read from the current line.
					#however, it is set as self.feed since in this case, there is no motion and the feed value is useless (efficiency)
					self.ec.start_timer("InitSegment")
					self._next_segment = Segment3D(self.machine, self._curr_segment.pos_end, self._feed, "", self._abs_coord)
					self.ec.end_timer("InitSegment")

					return

		#no end-effector motion segments were found in the buffer

		#next_segment feed should be read from the current line.
		#however, it is set as self.feed since in this case, there is no motion and the feed value is useless (efficiency)
		self.ec.start_timer("InitSegment")
		self._next_segment = Segment3D(self.machine, self._curr_segment.pos_end, self._feed, "", self._abs_coord)
		self.ec.end_timer("InitSegment")


	def home_axes(self):
		"""
		home all axes. homing time is not counted
		"""

		for axis in Segment3D.AXES_ZERO:
			self._curr_segment.set_axis_value(axis, 0.0)

		self._curr_segment.recalculate_distances_and_feedrates()

	def set_axis(self, line):
		"""
		parse line to set axes value
		"""

		endeffector_position_changes = False	#flag to determine if any of the end-effector axes value is changed
		blocks = line.split(" ")[1:]			#remove the first block which is the GCode command

		for block in blocks:
			axis = block[0]
			pos = block[1:]

			self._curr_segment.set_axis_value(axis, float(pos))
			endeffector_position_changes = True

		if endeffector_position_changes:
			self._curr_segment.recalculate_distances_and_feedrates()

	def set_feedrate(self, line):
		"""
		parse line to set feedrate
		"""

		f = GCodeParser.identifier_value(line, "F")
		self._feed = f / 60.0 #convert mm/min to mm/sec

	def move_axis(self, line):
		"""
		parse line to move axes with a certain feed and add the motion time to the total time
		"""

		#set feedrate
		f = GCodeParser.identifier_value(line, "F")

		if f != GCodeParser._IDENTIFIER_NOT_FOUND:
			self._feed = f / 60.0 #convert mm/min to mm/sec

		#move last, curr and next segments
		if GCodeParser.line_has_endeffector_motion(line):
			#set G0 feedrate if G0 is used
			if line.startswith("G0"):
				curr_feedrate = self._feed
				self._feed = 1000 #TODO: set to a number higher than G0 feedrate of the machine. will be automatically adjusted to machine.max_feedrate

			if self._next_segment.line == line:
				self.ec.start_timer("CopySegment")
				self._last_segment.copy_from_segment(self._curr_segment)
				self.ec.end_timer("CopySegment")

				self._curr_segment = self._next_segment #safe as set_next_segment always creates a new segment for next_segment
			else:
				self._last_segment = self._curr_segment #safe as a new instance for curr_segment will be created

				self.ec.start_timer("InitSegment")
				self._curr_segment = Segment3D(self.machine, self._last_segment.pos_end, self._feed, line, self._abs_coord)
				self.ec.end_timer("InitSegment")

				self._curr_segment.adjust_to_max_feedrate()

			#restore original feedrate if G0 is used
			if line.startswith("G0"):
				self._feed = curr_feedrate

			self.ec.start_timer("SetNextSegment")
			self.set_next_segment()
			self.ec.end_timer("SetNextSegment")
		else:
			self._last_segment = self._curr_segment #safe as a new instance for curr_segment will be created

			#if the end-effector will not move, curr_segment starts and ends at the same position
			self.ec.start_timer("InitSegment")
			self._curr_segment = Segment3D(self.machine, self._last_segment.pos_end, self._feed, "", self._abs_coord)
			self.ec.end_timer("InitSegment")

			#next segment is not going to be used in this case. no need to set it

		#calculate times
		t, t_a, t_d = self.calculate_last_move_time()

		self.time += t
		self.time_a += t_a
		self.time_d += t_d

	def change_tool(self, line):
		"""
		perform a tool change
		"""

		self.time += 15 #TODO: set estimate tool change time


	def calculate_last_move_time_no_acc(self):
		"""
		calcualte the time requried for the last move with instantaneous feedrate changes
		"""

		#do not calculate if the feed is zero to avoid infinite time
		if self._feed == 0:
			return [0, 0, 0]
		
		if self._curr_segment.has_motion():
			time = self._curr_segment.calculate_motion_time_no_acc()
		else:
			time = 0
		
		return [time, 0, 0] #acceleration and deceleration times are zero

	def calculate_last_move_time(self):
		"""
		calcualte the time requried for the last move taking into account accelerations and jerks
		"""

		#assumes that a segment always starts with feed = 0 and ends with feed = 0

		#do not calculate if the feed is zero to avoid infinite time
		if self._feed == 0:
			return [0, 0, 0]
		
		if self._curr_segment.has_motion():
			self.ec.start_timer("FeedrateChangeJerk")

			#prev segment transition
			if Segment.feedrate_change_within_jerk(self.machine, self._last_segment, self._curr_segment):
				#add to continuous segment
				self._continuous_segment.add_dist(self._curr_segment.calculate_distance())
			else:
				#init continuous segment
				self._curr_segment.calculate_resultant_a_and_j()
				self._continuous_segment = Segment1D(self._curr_segment.calculate_distance(), self._curr_segment.feedrate, self._curr_segment.a, self._curr_segment.j)

				self._continuous_motion_ended = False

			#next segment transition
			if Segment.feedrate_change_within_jerk(self.machine, self._curr_segment, self._next_segment) == False:
				#end continuous segment
				self._continuous_motion_ended = True

			self.ec.end_timer("FeedrateChangeJerk")

			#calculate end-effector motion time
			time = [0, 0, 0]

			if self._continuous_motion_ended:
				self.ec.start_timer("CalculateSegmentMotionTime")
				time = self._continuous_segment.calculate_motion_time(0, 0)
				self.ec.end_timer("CalculateSegmentMotionTime")
		else:
			time = [0, 0, 0]

		return time

	
	def get_times(self):
		"""
		return the printing time without changing the units
		"""

		return self.time, self.time_a, self.time_d


	@staticmethod
	def clean_line(line):
		"""
		remove comments and strip white spaces from a line
		"""

		#comment line
		if line.startswith(GCodeParser._COMMENT_CHAR):
			return ""

		#remove line comment
		comment_char = line.find(GCodeParser._COMMENT_CHAR)
		if comment_char > -1:
			line = line[:comment_char]

		

		#remove section names (between brackets)
		open_br = line.find("(")
		if open_br > -1:
			close_br = line.find(")")

			if open_br == 0:
				prefix = ""
			else:
				prefix = line[:open_br]
			
			if close_br + 1 == len(line):
				suffix = ""
			else:
				suffix = line[close_br + 1:]

			line = prefix + suffix
		
		#removing a section name from in-between gcodes results in having 2 spaces. remove consecutive spaces and strip
		line = line.replace("  ", " ")
		line = line.strip()

		#remove line number
		if line.startswith("N"):
			line = " ".join(line.split(" ")[1:])

		return line

	@staticmethod
	def identifier_value(line, identifier):
		"""
		get the value of an identifier
		"""

		identifier_index = line.find(identifier)

		if identifier_index == -1:
			return GCodeParser._IDENTIFIER_NOT_FOUND
		
		line = line[identifier_index:] #shorten the line for efficiency and to eliminate the need to loop over the line blocks
		blocks = line.split(" ")

		return float(blocks[0][1:])

	@staticmethod
	def line_has_endeffector_motion(line):
		"""
		return whether the given line has an end-effector axis motion
		"""

		for axis in Segment3D.AXES_ZERO:
			if line.find(axis) > -1:
				return True

		return False


class Segment(object):

	@staticmethod
	def calculate_axis_motion_time(d, f_start, f, f_end, a, j):
		""" 
		calcualte the time taken to complete a distance (d) with a starting feed (f_start), motion feed (f), ending feed (f_end), acceleration (a) and jerk (j)
		"""

		#if the axis did not move, do not calculate
		if d == 0:
			return [0, 0, 0]

		#lambda to get the sign of a number
		sign = lambda x: (1, -1)[x < 0]
		
		ExecutionControl.Debug("", level = 5)
		ExecutionControl.Debug("Motion Time", level = 5)
		ExecutionControl.Debug("d:", d, level = 5)
		ExecutionControl.Debug("f:", f_start, f, f_end, level = 5)
		ExecutionControl.Debug("a, j:", a, j, level = 5)

		#calculate acceleration time
		t_a = Segment.calculate_axis_acceleration_time(f_start, f, a, j)

		#calculate deceleration time
		t_d = Segment.calculate_axis_acceleration_time_at_end(f, f_end, a, j)

		#get acceleration and deceleration rate signs (because machine.a[any_axis] is a positive number)
		s_a = sign(f - f_start)
		s_d = sign(f_end - f)

		#if there is acceleration, update f_start because acceleration starts from f_start + j
		if abs(f_start) != abs(f):
			f_start += s_a * j

			#check that adding the jerk does not make f_start exceed f
			#use the sign function to check because it would also account for the case where abs(f_start) > abs(f)
			#abs(f_start) > abs(f) means that acceleration period is actually deceleration (should not be faced according to current understanding of machine operation)
			if sign(f - f_start) != s_a:
				f_start = f

		#no need to update f_end as deceleration starts from f (jerk at end) and t_d accounts for the jerk (at the end of the deceleration motion)

		#calcualte distances
		d_a = Segment.calculate_axis_acceleration_distance(f_start, s_a * a, t_a)	#acceleration distance
		d_d = Segment.calculate_axis_acceleration_distance(f, s_d * a, t_d)			#deceleration distance

		ExecutionControl.Debug("t_a, t_d:", t_a, t_d, level = 5)
		ExecutionControl.Debug("d_a, d_d:", d_a, d_d, level = 5)

		#if there is a special case (d_a + d_d > d), calculate t_ov
		if d_a + d_d > d:
			t_ov = Segment.calculate_axis_overlap_time(sign(f) * d, f_start, s_a * a, t_a, t_d) #d is a displacement value so the sign is included

			time = t_a + t_d - t_ov

			t_a_tmp = t_a - t_ov * t_a / (t_a + t_d)
			t_d_tmp = t_d - t_ov * t_d / (t_a + t_d)
			t_a = t_a_tmp
			t_d = t_d_tmp

			# f_updated = f_start + s_a * a * t_a 											#feed reached in the new t_a
			# d_a = Segment.calculate_axis_acceleration_distance(f_start, s_a * a, t_a) 	#acceleration distance
			# d_d = Segment.calculate_axis_acceleration_distance(f_updated, s_d * a, t_d)	#deceleration distance

			# ExecutionControl.Debug("t_a, t_d:", t_a, t_d, level = 5)
			# ExecutionControl.Debug("d_a, d_d:", d_a, d_d, level = 5)
		else:
			d_m = d - d_a - d_d	#const. feedrate motion distance
			t_m = d_m / abs(f) 	#const. feedrate motion time

			time = t_a + t_m + t_d

		ExecutionControl.Debug("time:", time, level = 5)
		ExecutionControl.Debug("", level = 5)

		return [time, t_a, t_d]

	@staticmethod
	def calculate_axis_overlap_time(d, f_start, a, t_a, t_d):
		""" 
		calculate time required to be overlapped so that d_ov satisfies the condition d = d_a + d_d - d_ov
		"""

		t_a_a = t_a * t_a
		t_d_d = t_d * t_d
		t_a_d = t_a * t_d
		f_t_a = f_start + a * t_a

		A = (0.5 * a * t_a_a + a * t_a_d - 0.5 * a * t_d_d) / math.pow(t_a + t_d, 2)
		B = (-f_start * t_a - a * t_a_a - f_t_a * t_d - a * t_a_d + a * t_d_d) / (t_a + t_d)
		C = f_start * t_a + 0.5 * a * t_a_a + f_t_a * t_d - 0.5 * a * t_d_d - d

		root_squared = B * B - 4 * A * C

		if root_squared < 0:
			ExecutionControl.Debug("", force_debug = True)
			ExecutionControl.Debug("========================================", force_debug = True)
			ExecutionControl.Debug("Overlap Time", force_debug = True)
			ExecutionControl.Debug("d:", d, force_debug = True)
			ExecutionControl.Debug("f_start:", f_start, force_debug = True)
			ExecutionControl.Debug("a:", a, force_debug = True)
			ExecutionControl.Debug("t:", t_a, t_d, force_debug = True)
			ExecutionControl.Debug("", force_debug = True)

			return 0

		root = math.sqrt(root_squared)

		A_2 = 2 * A
		s_1 = (-B + root) / A_2
		s_2 = (-B - root) / A_2

		if abs(s_1) < abs(s_2):
			return s_1
		else:
			return s_2

	@staticmethod
	def calculate_axis_acceleration_time(f_start, f, a, j):
		"""
		calculate the time required to accelerate from (f_start) to (f) with acceleration (a) and jerk (j) at the start
		"""

		f_difference = f - f_start
		if abs(f_difference) > j:
			time = (abs(f_difference) - j) / a
		else:
			time = 0

		return time

	@staticmethod
	def calculate_axis_acceleration_time_at_end(f, f_end, a, j):
		""" 
		calculate the time required to decelerate from (f) to (f_end) with acceleration (a) and a maximum jerk (j) at the end 
		"""

		f_difference = f_end - f

		if j > 0:
			j_multiples = int(abs(f_end - f) / j)

			if (f_difference % j) > 0:
				time = j_multiples * (j / a)
			elif j_multiples >= 1:
				time = (j_multiples - 1) * (j  / a)
			else:
				return 0
		else:
			time = abs(f_difference) / a

		return time

	@staticmethod
	def calculate_axis_acceleration_distance(f_start, a, t_a):
		"""
		calculate cumulative distance moved while accelerating from (f_start) with acceleration (a) for a period (t_a)
		"""

		#if accelerating from, for example, -5 to 5 then the displacement would be zero. this function solves this issue

		f_end = f_start + a * t_a

		#apply a threshold to f_end to avoid approximation errors
		if abs(f_end) < 0.000000001:
			f_end = 0

		if (f_start < 0 and f_end > 0) or (f_start > 0 and f_end < 0):
			t_0 = abs(-f_start / a) # time taken to reach from f_start to zero

			return Segment.calculate_axis_acceleration_distance(f_start, a, t_0) + Segment.calculate_axis_acceleration_distance(0, a, t_a - t_0)
		else:
			return abs(f_start * t_a + 0.5 * a * t_a * t_a)

	@staticmethod
	def feedrate_change_within_jerk(machine, curr_segment, next_segment):
		""" 
		return whether the change in feed between (curr_segment) and (next_segment) would not exceed the jerk values of (machine) 
		"""
		
		for axis in curr_segment.axis_feedrate:
			if abs(next_segment.axis_feedrate[axis] - curr_segment.axis_feedrate[axis]) > machine.j[axis]:
				return False

		return True


class Segment1D(object):

	def __init__(self, dist = 0, feedrate = 0, a = 0, j = 0):
		""" 
		initialize a 1d motion segment from a GCode line given the starting position  
		"""

		#class variables
		self.feedrate = feedrate
		self.a = a
		self.j = j

		self.dist = dist

	def copy_from_segment(self, segment):
		""" 
		copy the data of another segment to this segment 
		"""

		self.dist = segment.dist
		self.feedrate = segment.feedrate
		self.a = segment.a
		self.j = segment.j


	def has_motion(self):
		""" 
		return whether there is any motion in this segment or not 
		"""

		return (self.dist > 0)

	def add_dist(self, extra_dist):
		""" 
		increase the distance of this segment 
		"""

		self.dist += extra_dist


	def calculate_motion_time_no_acc(self):
		""" 
		calculate time taken to complete the motion of this segment 
		"""

		if self.feedrate == 0:
			return [0, 0, 0]

		time = self.dist / self.feedrate

		return [time, 0, 0]

	def calculate_motion_time(self, f_start, f_end):
		""" 
		calcualte the time requried for this segment taking into account the accelerations and jerks 
		"""
		
		if self.feedrate == 0:
			return [0, 0, 0]

		return Segment.calculate_axis_motion_time(self.dist, f_start, self.feedrate, f_end, self.a, self.j)


class Segment3D(object):

	AXES_ZERO = {"X": 0.0, "Y": 0.0, "Z": 0.0}

	def __init__(self, machine, pos_start, feedrate, line = "", abs_coord = True):
		""" 
		initialize a 3d motion segment from a GCode line given the starting position 
		"""

		#class variables
		self.machine = machine
		self.line = line

		self.abs_coord = abs_coord
		self.feedrate = feedrate

		self.a = 0				#resultant acceleration
		self.j = 0				#resultant jerk

		self.pos_start = {}		#absolute coordinates (dictionary)
		self.pos_end = {}		#absolute coordinates (dictionary)

		self.axis_dist = {} 	#disatnce moved for each axis (dictionary)
		self.axis_cosines = {}	#cosine the angles between motion axis and the x,y,z axes unit vectors (dictionary)
		self.axis_feedrate = {}	#feedrate for each axis (dictionary)

		#set segment start and end point
		self.pos_start = pos_start.copy()
		self.pos_end = pos_start.copy()

		if line != "":
			for axis in pos_start:
				axis_value = GCodeParser.identifier_value(line, axis)

				if axis_value != GCodeParser._IDENTIFIER_NOT_FOUND:
					if abs_coord:
						self.pos_end[axis] = axis_value
					else:
						self.pos_end[axis] = pos_start[axis] + axis_value

		#calculate distance and feedrate of each axis
		self.recalculate_distances_and_feedrates()

	def copy_from_segment(self, segment):
		""" 
		copy the data of another segment to this segment 
		"""

		self.machine = segment.machine
		self.line = segment.line

		self.abs_coord = segment.abs_coord
		self.feedrate = segment.feedrate

		self.a = segment.a
		self.j = segment.j

		self.pos_start = segment.pos_start.copy()
		self.pos_end = segment.pos_end.copy()
		self.axis_dist = segment.axis_dist.copy()
		self.axis_cosines = segment.axis_cosines.copy()
		self.axis_feedrate = segment.axis_feedrate.copy()

	def set_axis_value(self, axis, value, recalculate = False):
		""" 
		update the value of an axis and recalculate distances and feedrates if required 
		"""

		self.pos_end[axis] = value

		if recalculate:
			self.recalculate_distances_and_feedrates()


	def recalculate_distances_and_feedrates(self):
		""" 
		recalculate distances and feedrates 
		"""

		#calculate distance of each axis
		self._calculate_axes_distances()

		#calculate the feedrate of each axis
		self._calculate_axes_feedrates()

	def _calculate_axes_distances(self):
		""" 
		calculate the distance moved for each axis 
		"""

		if self.line == "":
			self.axis_dist = Segment3D.AXES_ZERO.copy()

		for axis in self.pos_start:
			self.axis_dist[axis] = abs(self.pos_end[axis] - self.pos_start[axis])
	
	def _calculate_axes_feedrates(self):
		""" 
		calculate the feedrate for each axis 
		"""

		if self.line == "" or self.feedrate == 0:
			self.axis_cosines = Segment3D.AXES_ZERO.copy()
			self.axis_feedrate = Segment3D.AXES_ZERO.copy()

		#direction vector starting at the origin
		u = [self.pos_end["X"] - self.pos_start["X"], self.pos_end["Y"] - self.pos_start["Y"], self.pos_end["Z"] - self.pos_start["Z"]]

		#cosines between the direction vector and each of the axes unit vectors
		#refereunce: https://www.intmath.com/vectors/5-dot-product-vectors-2-dimensions.php#anglebetweenvectors
		mag_u = geometry.vector_mag(u)

		if mag_u == 0:
			self.axis_cosines = Segment3D.AXES_ZERO.copy()
			self.axis_feedrate = Segment3D.AXES_ZERO.copy()

			return
		else:
			# consine formula (axes unit vectors should be defined (Segment3D._U_X = [1, 0, 0], etc.))
			# cos_x = geometry.dot_product(u, Segment3D._U_X) / ( mag_u * geometry.vector_mag(v_x) )
			# cos_y = geometry.dot_product(u, Segment3D._U_Y) / ( mag_u * geometry.vector_mag(v_y) )
			# cos_z = geometry.dot_product(u, Segment3D._U_Z) / ( mag_u * geometry.vector_mag(v_z) )

			#simplification of the above commented lines
			cos_x = u[0] / mag_u
			cos_y = u[1] / mag_u
			cos_z = u[2] / mag_u

		self.axis_cosines = {"X": cos_x, "Y": cos_y, "Z": cos_z}
		self.axis_feedrate = {"X": self.feedrate * cos_x, "Y": self.feedrate * cos_y, "Z": self.feedrate * cos_z}
	

	def has_motion(self):
		""" 
		return whether there is any motion in this segment or not 
		"""

		for axis in self.axis_dist:
			if self.axis_dist[axis] > 0:
				return True

		return False

	def adjust_to_max_feedrate(self):
		""" 
		limit the feedrate on an axis if it exceeds the maximum feedrate allowable and adjust the feedrate of the other axes accordingly 
		"""

		#find the lowest required feedrate to satisfy the machine's limits
		feedrate = self.feedrate

		for axis in self.axis_feedrate:
			if abs(self.axis_feedrate[axis]) > self.machine.max_feedrate[axis]:
				tmp_feedrate = abs(self.machine.max_feedrate[axis] / self.axis_cosines[axis])

				if tmp_feedrate < feedrate:
					feedrate = tmp_feedrate

		#update feedrates
		self.feedrate = feedrate

		for axis in self.axis_feedrate:
			self.axis_feedrate[axis] = feedrate * self.axis_cosines[axis]

	def calculate_resultant_a_and_j(self):
		""" 
		calculate a and j for the straight line motion 
		"""

		#find the lowest required a and j to satisfy the machine's limits
		first_axis = True #flag to determine if the current for loop is the first loop

		a = 0
		j = 0

		for axis in self.pos_start:
			if self.axis_cosines[axis] == 0:
				continue

			if first_axis:
				a = abs(self.machine.a[axis] / self.axis_cosines[axis])
				j = abs(self.machine.j[axis] / self.axis_cosines[axis])

				first_axis = False
			else:
				axis_a = abs(a * self.axis_cosines[axis])

				if axis_a > self.machine.a[axis]:
					tmp_a = abs(self.machine.a[axis] / self.axis_cosines[axis])

					if tmp_a < a:
						a = tmp_a

				axis_j = (j * self.axis_cosines[axis])

				if axis_j > self.machine.j[axis]:
					tmp_j = abs(self.machine.j[axis] / self.axis_cosines[axis])

					if tmp_j < j:
						j = tmp_j

		self.a = a
		self.j = j

	def calculate_distance(self):
		""" 
		calculate distance between start and end points 
		"""

		dist_square = 0

		for axis in self.axis_dist:
			dist_square += math.pow(self.axis_dist[axis], 2)

		dist = math.sqrt(dist_square)

		return dist

	def calculate_motion_time_no_acc(self):
		""" 
		calculate time taken to complete the motion of this segment 
		"""

		#do not calculate if the feed is zero to avoid infinite time
		if self.feedrate == 0:
			return [0, 0, 0]

		time = self.calculate_distance() / self.feedrate

		return [time, 0, 0]

	def calculate_motion_time(self, f_start, f_end):
		""" 
		calcualte the time requried for this segment taking into account the accelerations and jerks of the machine 
		"""

		#do not calculate if the feed is zero to avoid infinite time
		if self.feedrate == 0:
			return [0, 0, 0]

		#find the axis with the highest acceleration time to take as a reference
		#because it will force the other axes to increase their acceleration time so that they have the same acceleration time as the reference axis
		t_a = {} #acceleration time for each axis
			
		first_axis = True	#flag to differentiate between the first item in self.pos and the rest to help find the highest acceleration time
		max_time_axis = ""	#the axis with the highest acceleration time

		for axis in self.pos_end:
			t_a[axis] = self.calculate_axis_acceleration_time(axis, f_start[axis])

			if first_axis:
				max_time_axis = axis

				first_axis = False
			else:
				if t_a[axis] > t_a[max_time_axis]:
					max_time_axis = axis

		#calculate end-effector motion time
		time = self.calculate_axis_motion_time(max_time_axis, f_start[max_time_axis], f_end[max_time_axis])

		return time


	def calculate_axis_motion_time(self,  axis, f_start, f_end):
		""" 
		calcualte the time taken to complete the distnace to be moved by (axis) with a starting feed (f_start) and ending feed (f_end) while accounting for acceleration and jerk 
		"""

		d = self.axis_dist[axis]
		f = self.axis_feedrate[axis]
		a = self.machine.a[axis]
		j = self.machine.j[axis]

		return Segment.calculate_axis_motion_time(d, f_start, f, f_end, a, j)

	def calculate_axis_acceleration_time(self, axis, f_start):
		""" 
		calculate the time required to accelerate from (f_start) to (axis) feedrate while accounting for acceleration and jerk 
		"""

		f = self.axis_feedrate[axis]
		a = self.machine.a[axis]
		j = self.machine.j[axis]

		return Segment.calculate_axis_acceleration_time(f_start, f, a, j)


class Machine3D(object):

	def __init__(self, max_feedrate, a, j):
		self.max_feedrate = max_feedrate

		self.a = a
		self.j = j
