#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import os
import signal
import subprocess

start_save_cmd_code = 1
stop_save_cmd_code = 0



class sav_vid:
	def __init__(self):
		self.sub = rospy.Subscriber('/saveVideoCmd', Int32, self.callback)
		self.video_cnt = 0
		self.p = 0
		self.cmd = "/home/firefly/gst_bash/save_video.sh"
		self.save_cmd_flag = False
		self.start_save_cmd_code = 1
		self.stop_save_cmd_code = 0
		self.rate = rospy.Rate(100)

	def callback(self,data):
		rospy.loginfo("Получено число: %d", data.data)
		if data.data == start_save_cmd_code:
			self.save_cmd_flag = True
		if data.data == 0:
			self.save_cmd_flag = False

	def listener(self):
		print("SAVE VIDEO NODE IS RUNNING\n")
		
		while not rospy.is_shutdown():
			if self.save_cmd_flag == True:	
				self.start_save_video()
			self.rate.sleep()

	def start_save_video(self):
		print("\n\nSTART_SAVE_VIDEO\n\n")
		os.environ["VIK_VIDEO_FILENAME"] = "output_" + str(self.video_cnt) + ".flv"
		
		self.p = subprocess.Popen("exec " + self.cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
		
		while not rospy.is_shutdown():
			if self.save_cmd_flag == False:
				self.stop_save()
				return
			self.rate.sleep()

		# print(subprocess.run(["/home/firefly/gst_bash/save_video.sh"], shell=True))   
		print("\n\nRETURN FROM START_SAVE_VIDEO\n\n")
	
	def stop_save(self):
		#self.p.kill()
		os.killpg(os.getpgid(self.p.pid), signal.SIGTERM)
		print("\nSTOP_SAVE_VIDEO\n\n")
		self.video_cnt+=1



if __name__ == '__main__':
	try:
		rospy.init_node('tof_cam_control_listener', anonymous=True)
		node = sav_vid()
		node.listener()
	except rospy.ROSInterruptException:
		pass
