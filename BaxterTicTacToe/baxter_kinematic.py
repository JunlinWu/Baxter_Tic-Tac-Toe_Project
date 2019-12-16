#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import math
import random
import struct
import rospy

from gazebo_msgs.srv import (
	SpawnModel,
	DeleteModel,
)
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_msgs.msg import (
	UInt16,
	Header,
	Empty,
)
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)
from sensor_msgs.msg import JointState

import baxter_interface
from baxter_interface import CHECK_VERSION

# Code borrowed from :
#     RethinkRobotics' joint_velocity_wobbler.py -> Referred to here on as Wobbler
#     github.com/yashmanian/Baxter-Tic-Tac-Toe/blob/master/src/IK.py -> Referred to here on as Yashmanian
#     https://github.com/RethinkRobotics/baxter_examples/blob/master/scripts/ik_service_client.py -> Referred to here on as ik_service_client

class TicTacToeBaxter(object):

	# function __init__(self) taken from Wobbler
	def __init__(self):
		self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
										 UInt16, queue_size=10)
		self._left_arm = baxter_interface.limb.Limb("left")
		self._right_arm = baxter_interface.limb.Limb("right")
		# self._right_joint_names = self._right_arm.joint_names()

		# control parameters
		self._rate = 500.0  # Hz

		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()

		# set joint state publishing to 500Hz
		self._pub_rate.publish(self._rate)
		self.table_height = -0.20514202804666667 #avg of z's?


	# function ik_request retrieved from Yashmanian
	# function modified for our purposes
	def ik_request(self, limb, pose):
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		try:
			resp = self._iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return False
		# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		limb_joints = {}
		if (resp_seeds[0] != resp.RESULT_INVALID):
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		else:
			rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
			return False
		return limb_joints

	def move_to_position(self, limb, pos_data):
		joint_angles = self.ik_request(limb, pos_data)
		self._right_arm.move_to_joint_positions(joint_angles)

	def move_and_draw_X(self, limb, pos_data, right_w1_diff):
		#rostopic echo /robot/limb/right/endpoint_state 
		joint_angles = self.ik_request(limb, pos_data)
		self._right_arm.move_to_joint_positions(joint_angles)
		joint_angles['right_w1'] = joint_angles['right_w1'] + 0.5
		self._right_arm.move_to_joint_positions(joint_angles)
		joint_angles['right_w1'] = joint_angles['right_w1'] + right_w1_diff
		self._right_arm.move_to_joint_positions(joint_angles)
		joint_angles['right_e0'] = joint_angles['right_e0'] + 0.1
		self._right_arm.move_to_joint_positions(joint_angles)
		joint_angles['right_e0'] = joint_angles['right_e0'] - 0.5
		self._right_arm.move_to_joint_positions(joint_angles)
		joint_angles['right_s1'] = joint_angles['right_s1'] - 0.5
		self._right_arm.move_to_joint_positions(joint_angles)


	def _reset_control_modes(self):
		rate = rospy.Rate(self._rate)
		for _ in xrange(100):
			if rospy.is_shutdown():
				return False
			self._left_arm.exit_control_mode()
			self._right_arm.exit_control_mode()
			self._pub_rate.publish(100)  # 100Hz default joint state rate
			rate.sleep()
		return True

	def set_neutral(self):
		"""
		Sets both arms back into a neutral pose.
		"""
		print("Moving to neutral pose...")
		self._left_arm.move_to_neutral()
		self._right_arm.move_to_neutral()

	def clean_shutdown(self):
		print("\nExiting example...")
		#return to normal
		self._reset_control_modes()
		self.set_neutral()
		if not self._init_state:
			print("Disabling robot...")
			self._rs.disable()
		return True
		
	# From Wobbler and ik_service_client
	def get_position_coords(self, row, col):
		if (row == -2):
			return Pose(
				position=Point(
					x=0.656982770038,
					y=-0.852598021641,
					z=0.0388609422173,
					# z=self.table_height,
				),
				orientation=Quaternion(
					x=0.367048116303,
					y=0.885911751787,
					z=-0.108908281936,
					w=0.261868353356,
				),
			)      
		elif (row == -1):
			#x from baxter to table
			#y from my right to left
			# z up and down?
			return Pose(
				position=Point(
					x=0.787249483958,
					y=-0.0339184119656,
					z=0.339076014588,
				),
				orientation=Quaternion(
					x=0.0217844362745,
					y=0.996048246533,
					z=0.0193781859039,
					w=0.0838916850819,
				),
			)
		elif (row == 0):
			if (col == 0):
				return Pose(
					position=Point(
						x=0.91974542468,
						y=0.24101757845,
						z=-0.176811631634
					),
					orientation=Quaternion(
						x=-0.0393450150427,
						y=0.948237432094,
						z=0.114172139126,
						w=0.293704723854,
					),
				)    
			elif (col == 1):
				return Pose(
					position=Point(
						x=1.03977182876,
   						y=-0.0243442138083,
   						z=-0.133746290317,
					),
					orientation=Quaternion(
						x=-0.33338005997,
    					y= 0.850923326427,
   						z= 0.0124581838141,
   						w= 0.405748717573,
					),
				)
			else: # col == 2
				return Pose(
					position=Point(
						x=1.00255600035,
						y=-0.357155468799,
						z=-0.169818892857,
					),
					orientation=Quaternion(
						x=0.305470706701,
						y=0.924107000024,
						z=0.104713905473,
						w=0.204325470399,
					),
				)
		elif (row == 1):
			if (col == 0):
				return Pose(
					position=Point(
						x=0.820372263688,
						y=0.316064895536,
						z=-0.13938963785,
					),
					orientation=Quaternion(
						x=-0.188787821516,
						y=0.958636516802,
						z=0.0622648894339,
						w=0.203711243295,
					),
				)
			elif (col == 1):
				return Pose(
					# position=Point(
					# 	x=0.72754973777,
					# 	y=-0.0456673463623,
					# 	z=-0.113081248743,
					# ),
					# orientation=Quaternion(
					# 	x=-0.183856081621,
					# 	y=0.931711170228,
					# 	z=0.20248487417,
					# 	w=0.238979313449,
					# ),

					position=Point(
						x=0.826547852931,
						y=0.21683370928,
						z=-0.157385573811,
					),
					orientation=Quaternion(
						x=-0.0234675298383,
						y=0.98027580015,
						z=0.132305628084,
						w=0.144927055657,
					),
				)
			else: # col == 2
				# return Pose(
				# 	# position=Point(
				# 	# 	x=0.719890960027,
				# 	# 	y=-0.343499133907,
				# 	# 	z=-0.23907157418,
				# 	# ),
				# 	# orientation=Quaternion(
				# 	# 	x=-0.183425451674,
				# 	# 	y=0.932098389595,
				# 	# 	z=0.201755887983,
				# 	# 	w=0.238416143449,
				# 	# ),
				# 	position=Point(
				# 		x=0.780358962262,
				# 		y=-0.16810174759,
				# 		z=-0.175714652577,
				# 	),
				# 	orientation=Quaternion(
				# 		x=-0.676208860777,
				# 		y=0.724317581917,
				# 		z=-0.00576185901652,
				# 		w=0.134433694113,
				# 	),
				# )
				return Pose(
					position=Point(
						x=0.726730386256,
						y=-0.337138061507,
						z=-0.180180184185,
					),
					orientation=Quaternion(
						x=0.0860937854482,
						y=0.98526135043,
						z=0.102043235465,
						w=0.106935071664,
					),
				)
		else: #row == 2
			if (col == 0):
				return Pose(
					position=Point(
						x=0.547734913249,
						y=0.251832753007,
						z=-0.215441510007,
					),
					orientation=Quaternion(
						x=-0.184449999868,
						y=0.931761103617,
						z=0.202058383842,
						w=0.238687772737,
					),
				)
			elif (col == 1):
				return Pose(
					position=Point(
						x=0.534874043181,
						y=-0.0757209255018,
						z=-0.181414000715,
					),
					orientation=Quaternion(
						x=0.183635876367,
						y=0.978400025963,
						z=0.0949192332852,
						w=-0.00126224387046,
					),
				)
			else: #col == 2
				return Pose(
					position=Point(
						x=0.547145675892,
						y=-0.318224548226,
						z=-0.198527322337,
					),
					orientation=Quaternion(
						x=0.199231251424,
						y=0.972984752085,
						z=0.101071125597,
						w=0.0582426668072,
					),
				)
		return pose


	def make_move(self, row, col):
		limb = "right"
		if (row == 0):
			if (col == 2):
				self.move_and_draw_X(limb, self.get_position_coords(row,col), -0.2)
			else:
				# col == 0
				self.move_to_position(limb, self.get_position_coords(row,col))
		elif (row == 1):
			if (col == 2):
				# right_w1: -0.35 for (1, 2)
				self.move_to_position(limb, self.get_position_coords(row, col))
			else: 
				# col == 0 or 1
				# right_w1: -0.2 for (1, 1), (1, 0)
				self.move_and_draw_X(limb, self.get_position_coords(row, col), -0.2)
		else: #row == 2
			self.move_to_position(limb, self.get_position_coords(row,col))


	def play(self):
		self.set_neutral()
		rate = rospy.Rate(self._rate)
		start = rospy.Time.now()
		limb = "right"
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(0,0)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(0,1)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(0,2)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(1,0)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(1,1)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(1,2)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(2,0)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(2,1)
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.set_neutral()
		self.move_to_position(limb, self.get_position_coords(-1,0))
		self.make_move(2,2)
		self.move_to_position(limb, self.get_position_coords(-1,0))

		self.set_neutral()
		# self.move_to_position(limb, self.get_position_coords(-1,0))
		# # self.move_and_draw_X(limb, self.get_position_coords(2,1), 1)
		
		# # self.set_neutral()
		# self.move_to_position(limb, self.get_position_coords(-1,0))
		# self.move_and_draw_X(limb, self.get_position_coords(2,2),1)





			# self.move_to_position(limb, Pose(
			# 	position=Point(
			# 		x=0.719890960027,
			# 		y=-0.343499133907,
			# 		z=0.173497157418,
			# 	),
			# 	orientation=Quaternion(
			# 		x=-0.183425451674,
			# 		y=0.932098389595,
			# 		z=0.201755887983,
			# 		w=0.238416143449,
			# 	),
			# ))
			# self.move_to_position(limb, self.get_position_coords(1,2))
			# # Lifting arm up so it doesn't bash into the board
			# self.move_to_position(limb, Pose(
			# 	position=Point(
			# 		x=0.719890960027,
			# 		y=-0.643499133907,
			# 		z=-0.11497157418,
			# 	),
			# 	orientation=Quaternion(
			# 		x=-0.183425451674,
			# 		y=0.932098389595,
			# 		z=0.201755887983,
			# 		w=0.238416143449,
			# 	),
			# ))

			#======================================

			#move to 1,1
			#print("moved to center pos")
			#self.move_to_position(limb, self.get_position_coords(1,1))
			
			#pause for 10 seconds
			# rate.sleep()
			
	#	#command to move right arm to position to read board state
	#        print("moved to watching pos")
	#        self.move_to_position(limb, self.get_position_coords(-1,0))

			#pause for 10 seconds
			#rate.sleep()
		
			# self.drawX()


def main():
	# Play TicTacToe
	rospy.init_node("rsdk_ik_service_client")
	baxter = TicTacToeBaxter()
	# rospy.on_shutdown(baxter.clean_shutdown)
	baxter.play()

	print("Done.")

if __name__ == '__main__':
	main()

