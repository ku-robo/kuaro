#!/usr/bin/env python
#coding:utf-8
import rospy
import Tkinter
import re
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Int16MultiArray

from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from pylab import figure, show
import math

import warnings

def fxn():
    warnings.warn("deprecated", DeprecationWarning)

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    fxn()


end_flag = False

status_text = "---------"

#ros
rospy.init_node('uway_pointmaker_py')
pub = rospy.Publisher('/click_point', Int16MultiArray, queue_size=10)
r = rospy.Rate(10)
draw_element = []

# TK set
window = Tkinter.Tk()
label1 = Tkinter.Label(window, text = "-----u UWAY PointsMaker u-----")
label1.pack()
label2 = Tkinter.Label(window, text = "Now Status  :  %s" %(status_text))
label2.pack()


#func
def publish_point(_x,_y):
	cp = Int16MultiArray()
	if status_text == "MAKE":
		num = 1
	elif status_text == "DELETE":
		num = 2
	elif status_text == "MOVE":
		num = 3
	elif status_text == "NEUTRAL":
		return
	elif status_text == "END":
		num = -1
	else :
		num = 0
		for var in range(0, 10):
			cp.data = [num,int(round(_x,0)),int(round(_y,0))]
			pub.publish(cp)
			r.sleep()
			#return
	cp.data = [num,int(round(_x,0)),int(round(_y,0))]
	print 'send_data : ',cp.data
	pub.publish(cp)
	r.sleep()

def make_point():
	global status_text
	status_text = "MAKE"
	label2.configure(text =  "Now Status  :  %s" %(status_text))

def delete_point():
	global status_text
	status_text = "DELETE"
	label2.configure(text =  "Now Status  :  %s" %(status_text))

def move_point():
	global status_text
	status_text = "MOVE"
	label2.configure(text =  "Now Status  :  %s" %(status_text))

def neutral_point():
	global status_text
	status_text = "NEUTRAL"
	label2.configure(text =  "Now Status  :  %s" %(status_text))

def end_point():
	global end_flag
	end_flag = True
	status_text = "END"
	label2.configure(text =  "Now Status  :  %s" %(status_text))
	window.destroy()

def draw_callback(all_data):
	data_num = len(all_data.data)
	dc_count = 0
	x0 = -100
	y0 = -100
	x1 = 0
	y1 = 0
	# point and line remove
	if len(draw_element) != 0 :
		for de in draw_element :
			de.remove()
		draw_element[:] = []

	while dc_count != data_num:
		if all_data.data[dc_count + 0] == 100:
			x1 = all_data.data[dc_count + 1]
			y1 = all_data.data[dc_count + 2]
			cb_point, = plt.plot([x1],[y1],color="r", marker="o",markersize=7)
			draw_element.append(cb_point)
		else:
			x1 = all_data.data[dc_count + 1]
			y1 = all_data.data[dc_count + 2]
			cb_point, = plt.plot([x1],[y1],color="b", marker="o",markersize=7)
			draw_element.append(cb_point)
		if x0 != -100 and y0 != -100 :
			cb_line, = plt.plot([x0,x1],[y0,y1],'g',lw=1)
			draw_element.append(cb_line)

		x0 = x1
		y0 = y1
		dc_count = dc_count + 3



button1 = Tkinter.Button(window, text = " MAKE  ", bg = "blue", fg = "white",command = make_point )
button1.pack()
button2 = Tkinter.Button(window, text = "DELETE " , command = delete_point )
button2.pack()
button3 = Tkinter.Button(window, text = " MOVE  "   , command = move_point )
button3.pack()
button4 = Tkinter.Button(window, text = "NEUTRAL", command = neutral_point )
button4.pack()
button5 = Tkinter.Button(window, text = "  END  ", bg = "yellow", fg = "red" ,command = end_point )
button5.pack()

file_name = rospy.get_param('~image', 'home/test.pgm')
rospy.Subscriber("/draw_points", Int16MultiArray, draw_callback)

im = Image.open(file_name)

im_list = np.asarray(im)

plt.imshow(im_list)

publish_point(im.size[0],im.size[1])
make_point()

while not rospy.is_shutdown():
	c_point = plt.ginput(1,0.1)
	if end_flag:
		break
	if c_point == []:
		continue
	publish_point(c_point[0][0],c_point[0][1])

def end_print():
	print 'good bye uway'

plt.close('all')
if not end_flag:
	window.destroy()
