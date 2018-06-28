#!/usr/bin/env python
import rospy
import sys
import time
import math
import os
import numpy as np
import cv2
#from typing import NamedTuple

from gpd.msg import GraspConfigList
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import String
from sensor_msgs.msg import Image
from detecting_hand.srv import GetObjLoc
from detecting_hand.srv import Conv2DTo3D
from detecting_hand.msg import bounding_box_calculated_center
from detecting_hand.msg import msg2dto3d
from detecting_hand.msg import msg2dto3d_object
from cv_bridge import CvBridge, CvBridgeError
from robotics_task_tree_msgs.msg import ObjStatus
theta1_cup = 0
theta2_cup = 0
theta1_scissor = 0
theta2_scissor = 0
theta1_clock = 0
theta2_clock = 0
theta1_bear = 0
theta2_bear = 0
theta1_ball = 0
theta2_ball = 0
count = 0
val_1 = 0
val_4 = 0
val_3 = 0
val_2 = 0
val_5 = 0
flag = 1
hand_x_sum = 0
hand_x_avg = 0
hand_y_avg =0
hand_y_sum = 0
hand_z_sum = 0
hand_z_avg = 0
hand_count = 0
change = 2
change2 = 2
distance_cup_hand1 = 0
distance_cup_hand2 = 0
distance_scissor_hand1 = 0
distance_scissor_hand2 = 0
prev_hand_x_sum=0
prev_hand_y_sum=0
cup_loc = []
hand_loc = []
v1 = []
v2 =[]
v3 = []
v4 = []
count_inside = 0;
count_cup=0;
count_scissor=0;
cup_loc_save = []
scissor_loc_save = []
clock_loc_save = []
bear_loc_save = []
ball_loc_save = []

draw_box = 0
cup_chance = 0
scissor_chance = 0
clock_chance = 0
bear_chance = 0
ball_chance = 0

v_cup=0
v_scissor=0
v_clock=0
v_bear = 0
v_ball = 0

distance = [0,0,0,0,0]

class location:
    center_x_1 = 0
    center_y_1 = 0
    center_x_2 = 0
    center_y_2 = 0

class location_center_3d:
    center_x = 0
    center_y = 0


class location_center:
    center_x = 0
    center_y = 0
   

hand = location_center() 
cup1 = location_center() #first point of the cup
cup2 = location_center() #second point of the cup
scissor1 = location_center() #first point of the scissor
scissor2 = location_center() #second point of the scissor
clock1 = location_center() # first point of the clock
clock2 = location_center() #second point of the clock
bear1 = location_center() #first point of the bear
bear2=location_center() #second point of the bear
ball1 = location_center() #first point of the ball
ball2=location_center() #second point of the ball

cup1_loc = location()
cup1_loc_center = location_center() 
obj_cup1=[]
obj_cup2=[]
cup_fix_loc = location_center()


fix_cup_flag = 0
fix_scissor_flag = 0
fix_clock_flag = 0
fix_bear_flag = 0
fix_ball_flag = 0

hand_fix_flag = 0
hand_change = 0
cup_check = 1
scissor_check = 1
clock_check = 1
bear_check = 1
ball_check = 1

object_started_flag = 0
started = [0,0,0,0,0] #initially all the started flag for the objects are zero 
done = [0,0,0,0,0] #initially all the done flag for the objects are zero 

hand_i = 0
hand_i_x = 0
hand_i_y = 0

cup_status_pub = rospy.Publisher('/cup_status', ObjStatus, queue_size=10)
scissors_status_pub = rospy.Publisher('/scissors_status', ObjStatus, queue_size=10)
clock_status_pub = rospy.Publisher('/clock_status', ObjStatus, queue_size=10)
bear_status_pub = rospy.Publisher('/bird_status', ObjStatus, queue_size=10)
ball_status_pub = rospy.Publisher('/sports_ball_status', ObjStatus, queue_size=10)

def distance_object(x2,y2,x1,y1):
	return math.sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1))


def center_callback(hand_center): #storing the hand location 
	
     global hand,flag,hand_i,hand_i_x,hand_i_y
     hand.center_x=hand_center.x
     hand.center_y=hand_center.y
     if hand_i == 0:		#storing the first location after running the program
	hand_i_x = hand_center.x
	hand_i_y = hand_center.y
	hand_i = 1
     get_obj_loc_client()
     callback()




#
 

	
	  

 
     
    
def get_obj_loc_client(): #service call to find the object location from YOLO

    rospy.wait_for_service('get_object_loc')
   
  
    try:
        get_obj_loc = rospy.ServiceProxy('get_object_loc', GetObjLoc)
	global cup_loc_save,scissor_loc_save,clock_loc_save,bear_loc_save,ball_loc_save
	global fix_cup_flag,fix_scissor_flag,fix_clock_flag,fix_bear_flag,fix_ball_flag,cup_fix_loc,cup_check,scissor_check,clock_check,bear_check,ball_check,hand_loc,hand_count,hand_fix_flag,val_1,val_2,val_3,val_4,val_5
	with open("example4.txt", "a") as f:
		f.write("*%d %d %d %dcheckkkkk\n"%(cup_check,scissor_check,clock_check,bear_check))
	#if 105 < hand.center_x <110 and 237 < hand.center_y < 69 == 242:
	if hand_i_x-20<hand.center_x<hand_i_x+20 and  hand_i_y-20 <=hand.center_y<=hand_i_y+20:
		print" "
	else:		
		if cup_check ==1:
			msg=get_obj_loc("cup")	
			if msg.Class == "cup": #storing the location points for the cup
			
				cup1.center_x = msg.xmin;
				cup1.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				cup2.center_x = msg.xmax;
				cup2.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
			
				if fix_cup_flag == 0:
					cup_fix_loc = cup1
					#cup_fix_loc_2 = cup2
					cup_loc_save = msg
					fix_cup_flag = 1

				if msg.xmin <cup_loc_save.xmin - 15 or msg.xmin > cup_loc_save.xmin +15 or msg.ymin<cup_loc_save.ymin - 15 or msg.ymin>cup_loc_save.ymin+15:
				#if (cup_fix_loc_1.center_x-10 <=cup1.center_x<=cup_fix_loc_1.center_x + 10 and cup_fix_loc_1.center_y-10 <=cup1.center_y<=cup_fix_loc_1.center_y + 10)==0:
					
					fix_cup_flag = 2
					cup_check= 0
					hand_fix_flag = 2


				
					
	
					 
				'''if fix_cup_flag == 2:
					if cup_fix_loc_1.center_x-10 <=cup1.center_x<=cup_fix_loc_1.center_x + 10 or cup_fix_loc_1.center_y-10 <=cup1.center_y<=cup_fix_loc_1.center_y + 10: 
					#if cup_fix_loc_1 == cup1 and cup_fix_loc_2 == cup2:
						print "cup location didn't change**********"
					else:
						print "cup location change"
						with open("example3.txt", "a") as f:
							f.write("I didn't find cup location. my bad")		
						#cup_fix_loc_1 = cup1
						#cup_fix_loc_2 = cup2
						fix_cup_flag = 1
				
				
						hand_loc = [];
						hand_count = 0;
						cup_chance =0
						scissor_chance = 0
						clock_chance = 0'''

				
			elif msg.Class == ' ':
				val_1 =  val_1 + 1
				if val_1 > 5:
					
					fix_cup_flag = 2
					cup_check= 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
		
				
		if scissor_check ==1:
			msg=get_obj_loc("scissors")
			if msg.Class =="scissors":  ##storing the location points for the scissor

				scissor1.center_x = msg.xmin;
				scissor1.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				scissor2.center_x = msg.xmax;
				scissor2.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
			
				if fix_scissor_flag == 0:
					scissor_loc_save = msg
					scissor_fix_loc_1 = scissor1
					scissor_fix_loc_2 = scissor2
					fix_scissor_flag = 1
				if msg.xmin <scissor_loc_save.xmin - 10 or msg.xmin > scissor_loc_save.xmin +10 or msg.ymin<scissor_loc_save.ymin - 10 or msg.ymin>scissor_loc_save.ymin+10:
				
				
				
					fix_scissor_flag = 2
					scissor_check = 0
					hand_fix_flag = 2
					hand_loc = []; 
					hand_count = 0;
				
				'''if fix_scissor_flag == 2:
					if scissor_fix_loc_1.center_x-10 <=scissor1.center_x<=scissor_fix_loc_1.center_x + 10 or scissor_fix_loc_1.center_y-10 <=scissor1.center_y<=scissor_fix_loc_1.center_y + 10:
					#if scissor_fix_loc_1 == cup1 and scissor_fix_loc_2 ==  cup2:
						print "bowl location didn't change**********"
					else:
						print "bowl location change"
						with open("example3.txt", "a") as f:
							f.write("I didn't find cup location. my bad")
						#scissor_fix_loc_1 = scissor1
						#scissor_fix_loc_2 = scissor2
						fix_scissor_flag = 1
						hand_loc = [];
						hand_count = 0;
			
			
				
						cup_chance =0
						scissor_chance = 0
						clock_chance = 0'''
			elif msg.Class == ' ':
		
				
				val_2 = val_2 + 1
				if val_2 > 5:
					
					fix_scissor_flag = 2
					scissor_check = 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
			
	
		if clock_check ==1:
			msg=get_obj_loc("clock")
			if msg.Class =="clock":  #storing the location points for the clock

				clock1.center_x = msg.xmin;
				clock1.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				clock2.center_x = msg.xmax;
				clock2.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				
				if fix_clock_flag == 0:
					clock_loc_save = msg
					clock_fix_loc_1 = clock1
					clock_fix_loc_2 = clock2
					fix_clock_flag = 1
				
				if msg.xmin <clock_loc_save.xmin - 10 or msg.xmin > clock_loc_save.xmin +10 or msg.ymin<clock_loc_save.ymin - 10 or msg.ymin>clock_loc_save.ymin+10:
				#if (clock_fix_loc_1.center_x-10 <=clock1.center_x<=clock_fix_loc_1.center_x + 10 and clock_fix_loc_1.center_y-10 <=clock1.center_y<=clock_fix_loc_1.center_y + 10)==0:
					
					
					fix_clock_flag = 2
					clock_check = 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
				
				'''if fix_clock_flag == 2:
					if clock_fix_loc_1.center_x-10 <=clock1.center_x<=clock_fix_loc_1.center_x + 10 or clock_fix_loc_1.center_y-10 <=clock1.center_y<=clock_fix_loc_1.center_y + 10:
					#if clock_fix_loc_1 == clock1 and clock_fix_loc_2 ==  clock2:
						print "clock location didn't change**********"
					else:
						print "clock location change"
						with open("example3.txt", "a") as f:
							f.write("I didn't find cup location. my bad")
						#clock_fix_loc_1 = clock1
						#clock_fix_loc_2 = clock2
						fix_clock_flag = 1
						hand_loc = [];
						hand_count = 0;
				
				
				
						cup_chance =0
						scissor_chance = 0
						clock_chance = 0'''
				
			elif msg.Class == ' ':
		
				
				val_3 = val_3 + 1
				
				if val_3 > 10:
					
					fix_clock_flag = 2
					clock_check = 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
		if bear_check == 1:
			msg=get_obj_loc("bird")
			if msg.Class =="bird":  ##storing the location points for the bear
			
				bear1.center_x = msg.xmin;
				bear1.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				bear2.center_x = msg.xmax;
				bear2.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				
				if fix_bear_flag == 0:
					bear_loc_save = msg
					
					fix_bear_flag = 1
				
				if msg.xmin <bear_loc_save.xmin - 5 or msg.xmin > bear_loc_save.xmin +5 or msg.ymin<bear_loc_save.ymin - 5 or msg.ymin>bear_loc_save.ymin+5:
				#if (clock_fix_loc_1.center_x-10 <=clock1.center_x<=clock_fix_loc_1.center_x + 10 and clock_fix_loc_1.center_y-10 <=clock1.center_y<=clock_fix_loc_1.center_y + 10)==0:
					
				
					fix_bear_flag = 2
					bear_check = 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
				
				'''if fix_clock_flag == 2:
					if clock_fix_loc_1.center_x-10 <=clock1.center_x<=clock_fix_loc_1.center_x + 10 or clock_fix_loc_1.center_y-10 <=clock1.center_y<=clock_fix_loc_1.center_y + 10:
					#if clock_fix_loc_1 == clock1 and clock_fix_loc_2 ==  clock2:
						print "clock location didn't change**********"
					else:
						print "clock location change"
						with open("example3.txt", "a") as f:
							f.write("I didn't find cup location. my bad")
						#clock_fix_loc_1 = clock1
						#clock_fix_loc_2 = clock2
						fix_clock_flag = 1
						hand_loc = [];
						hand_count = 0;
				
				
				
						cup_chance =0
						scissor_chance = 0
						clock_chance = 0'''
				
			elif msg.Class == ' ':
		
				
				val_4 = val_4 + 1
				if val_4 > 5:
					
					bear_check = 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
			

		if ball_check == 1:
			msg=get_obj_loc("sports ball")
			if msg.Class =="sports ball":   #storing the location points for the ball
			
				ball1.center_x = msg.xmin;
				ball1.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				ball2.center_x = msg.xmax;
				ball2.center_y = msg.ymin + (msg.ymaxx - msg.ymin)/2;
				
				if fix_ball_flag == 0:
					ball_loc_save = msg
					
					fix_ball_flag = 1
				
				if msg.xmin <ball_loc_save.xmin - 5 or msg.xmin > ball_loc_save.xmin +5 or msg.ymin<ball_loc_save.ymin - 5 or msg.ymin>ball_loc_save.ymin+5:
				#if (clock_fix_loc_1.center_x-10 <=clock1.center_x<=clock_fix_loc_1.center_x + 10 and clock_fix_loc_1.center_y-10 <=clock1.center_y<=clock_fix_loc_1.center_y + 10)==0:
					
				
					fix_ball_flag = 2
					ball_check = 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
				
				'''if fix_clock_flag == 2:
					if clock_fix_loc_1.center_x-10 <=clock1.center_x<=clock_fix_loc_1.center_x + 10 or clock_fix_loc_1.center_y-10 <=clock1.center_y<=clock_fix_loc_1.center_y + 10:
					#if clock_fix_loc_1 == clock1 and clock_fix_loc_2 ==  clock2:
						print "clock location didn't change**********"
					else:
						print "clock location change"
						with open("example3.txt", "a") as f:
							f.write("I didn't find cup location. my bad")
						#clock_fix_loc_1 = clock1
						#clock_fix_loc_2 = clock2
						fix_clock_flag = 1
						hand_loc = [];
						hand_count = 0;
				
				
				
						cup_chance =0
						scissor_chance = 0
						clock_chance = 0'''
				
			elif msg.Class == ' ':
		
				
				val_5 = val_5 + 1
				if val_5 > 5:
					
					ball_check = 0
					hand_fix_flag = 2
					hand_loc = [];
					hand_count = 0;
			

		
	 
 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    

def unit_vector(vector): #calculating angle between two vectors 
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    with open("example2.txt", "a") as f:
	f.write("%s %s\n"%(v1,v2))
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return round(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)),2)

def calculating_vectors(objx2,objy2,objx1,objy1,handx,handy):   #calculating vectors from given points 
	global v1,v2,v3,v4
	v1 = []
	v2 =[]
	v3 = []
	v4 = []	
	v1.append(objx2-objx1) #vector between obj1 -> obj2
	v1.append(objy2-objy1)
	

	v2.append(handx -objx1)  #vector between obj1 -> hand
	v2.append(handy-objy1)
	

	v3.append(objx1-objx2) #vector between obj2 -> obj1
	v3.append(objy1-objy2)
	

	v4.append(handx-objx2) #vector between obj2 -> hand
	v4.append(handy-objy2)
def intention_object(theta1,prev_theta1,theta2,prev_theta2):  #comparing the angles which are created with the two points of the object and the hand
 	if theta1==prev_theta1 and theta2==prev_theta2:       #if both the angle decrease from the previous angles then the hand is moving towards the object. 

		val = 1
	elif theta1<prev_theta1 and theta2<prev_theta2:
		val = 2
	elif theta1<prev_theta1 and theta2==prev_theta2:
		val = 2
	elif theta1<=prev_theta1 and theta2<prev_theta2:
		val = 2
	elif theta1==prev_theta1 and theta2<prev_theta2:
		val = 2
	else:
		val = 0
	
	return val	
def callback():
	global count,near_cup,theta1_cup,theta2_cup,theta1_scissor,theta2_scissor,theta1_clock,theta2_clock,theta1_bear,theta2_bear,theta1_ball,theta2_ball,hand_count,hand_x_sum,hand_y_sum,hand_z_sum,hand_loc,prev_hand_x_avg,prev_hand_y_avg,hand_x_avg,hand_y_avg,cup1_loc,cup1_loc_center,hand,obj1,obj2,v1,v2,v3,v4,flag,obj_fix1,obj_fix2
	#time.sleep(0.01)
	global distance_cup_hand1,distance_cup_hand2,distance_scissor_hand1,distance_scissor_hand2,change,change2,count_inside,count_cup,count_scissor,draw_box,cup_chance,scissor_chance,clock_chance,bear_chance,ball_chance,v_cup,v_scissor,v_clock,v_bear,v_ball	
	global hand_loc_save,hand_fix_flag,hand_save_x,hand_save_y,distance,hand_i_x,hand_i_y
	
	if hand_i_x-20<hand.center_x<hand_i_x+20 and  hand_i_y-20 <=hand.center_y<=hand_i_y+20: #it compares the hand position with the initial position that is stored in the center_callback function.
					                #After each turn, in this part the chance value and started value are assigned initial value and the hand goes for another object.                   
		print "loc no change~~~~~~~~~~~~"
		print "%d %d\n"%(hand_i_x,hand_i_y)
		'''if hand_fix_flag == 2:
			if cup_check == 0:
				cup_chance = 0
			if ban_check ==0:
				cup_check = 0
			if clock_check ==0:
				clock_chance =0
			if bear_check ==0:
				bear_chance =0'''
		cup_chance = 0
		scissor_chance = 0
		clock_chance = 0
		bear_chance = 0
		ball_chance = 0
		object_started_flag = 0
		for x in range (0,5):    #it will check if any started flag is one. if it is then it will make the done flag value to one. 
			if started[x] == 1:
				done[x]=1
				started[x]=0
		
		
	elif hand_fix_flag == 2:
		
		
		if hand.center_y>30:
			hand_fix_flag = 0
			cup_chance = 0
			scissor_chance = 0
			clock_chance = 0
			bear_chance = 0
			ball_chance = 0
			#count = 0
			val_1 = 0
			val_2 =0
			val_3 = 0
			val_4 = 0
			val_5 = 0
			'''object_started_flag = 0
			for x in range (0,5):
				if started[x] == 1:
					done[x]=1
					started[x]=0'''
	elif hand_fix_flag == 0:
		
			
		hand_fix_flag = 1
			
	elif hand_fix_flag ==1:
		
		
		

		hand_loc.append(hand); #storing the hand location in a list for average 
	

		hand_x_sum=hand_x_sum + hand_loc[hand_count].center_x;
		hand_y_sum=hand_y_sum + hand_loc[hand_count].center_y;
		
	
		
		hand_count = hand_count + 1;

			
		if count % 2 == 0:   #average the hand values after 2 rounds
			count_inside=count_inside+1
			prev_hand_x_avg = hand_x_avg
			prev_hand_y_avg = hand_y_avg
			hand_x_avg = hand_x_sum/2
			hand_y_avg = hand_y_sum/2
		
					
			
			if done[0] == 1 and v_cup!=-1:
				
				cup_chance =0
				scissor_chance = 0
				clock_chance = 0
				bear_chance = 0
				ball_chance = 0 
				v_cup = -1
				hand_fix_flag = 2
				

			if done[0] == 1:
				val_cup = -1
				
			else:
				calculating_vectors(cup2.center_x,cup2.center_y,cup1.center_x,cup1.center_y,hand_x_avg,hand_y_avg)
			

	
				prev_theta1_cup = theta1_cup
				prev_theta2_cup = theta2_cup
				prev_distance_cup_hand1 = distance_cup_hand1
		
					
				temp_val_x = ( cup2.center_x - cup1.center_x )/ 2
				temp_val_y = cup2.center_y 
				distance[0] = distance_object(temp_val_x,temp_val_y,hand.center_x,hand.center_y)
				theta1_cup = angle_between(v1,v2) #calculating the angle between first two vectors in degree
				theta1_cup = (int)(math.degrees(theta1_cup))
				theta2_cup = angle_between(v3,v4) #calculating the angle between last two vectors in degree	
				theta2_cup = (int)(math.degrees(theta2_cup))
				val_cup = intention_object(theta1_cup,prev_theta1_cup,theta2_cup,prev_theta2_cup)
			
			if done[1] ==1 and v_scissor!=-1:
				
				cup_chance =0
				scissor_chance = 0
				clock_chance = 0
				bear_chance = 0
				ball_chance = 0
				v_scissor = -1
				hand_fix_flag = 2
				
			
			if done[1] ==1:
				val_scissor = -1
			else:
				calculating_vectors(scissor2.center_x,scissor2.center_y,scissor1.center_x,scissor1.center_y,hand_x_avg,hand_y_avg)
				prev_theta1_scissor = theta1_scissor
				prev_theta2_scissor = theta2_scissor
				prev_distance_scissor_hand1 = distance_scissor_hand1

			
					
				temp_val_x = ( scissor2.center_x - scissor1.center_x )/ 2
				temp_val_y = scissor2.center_y 
				distance[1] = distance_object(temp_val_x,temp_val_y,hand.center_x,hand.center_y)
				theta1_scissor = angle_between(v1,v2) #calculating the angle between first two vectors in degree
				theta1_scissor = (int)(math.degrees(theta1_scissor))
				theta2_scissor = angle_between(v3,v4) #calculating the angle between last two vectors in degree	
				theta2_scissor = (int)(math.degrees(theta2_scissor))
				val_scissor = intention_object(theta1_scissor,prev_theta1_scissor,theta2_scissor,prev_theta2_scissor)
			
			if done[2] == 1 and v_clock!=-1:
				
				cup_chance =0
				scissor_chance = 0
				clock_chance = 0
				bear_chance = 0
				ball_chance = 0
				v_clock = -1
				hand_fix_flag = 2
				
			if done[2] ==1:
				val_clock = -1
				
			else:

				calculating_vectors(clock2.center_x,clock2.center_y,clock1.center_x,clock1.center_y,hand_x_avg,hand_y_avg)
				prev_theta1_clock = theta1_clock
				prev_theta2_clock = theta2_clock
				
				temp_val_x = ( clock2.center_x - clock1.center_x )/ 2
				temp_val_y = clock2.center_y 
				
				

				distance[2] = distance_object(temp_val_x,temp_val_y,hand_x_avg,hand_y_avg)
				
				theta1_clock = angle_between(v1,v2) #calculating the angle between first two vectors in degree
				theta1_clock = (int)(math.degrees(theta1_clock))
				theta2_clock = angle_between(v3,v4) #calculating the angle between last two vectors in degree	
				theta2_clock = (int)(math.degrees(theta2_clock))
				val_clock = intention_object(theta1_clock,prev_theta1_clock,theta2_clock,prev_theta2_clock)
			if done[3] == 1 and v_bear!=-1:
				
				cup_chance =0
				scissor_chance = 0
				clock_chance = 0
				bear_chance = 0
				ball_chance = 0
				v_bear = -1
				hand_fix_flag = 2
				
			if done[3] ==1:
				val_bear = -1
				
			else:

				calculating_vectors(bear2.center_x,bear2.center_y,bear1.center_x,bear1.center_y,hand_x_avg,hand_y_avg)
				prev_theta1_bear = theta1_bear
				prev_theta2_bear = theta2_bear
				
				temp_val_x = ( bear2.center_x - bear1.center_x )/ 2
				temp_val_y =  bear2.center_y 
				distance[3] = distance_object(temp_val_x,temp_val_y,hand.center_x,hand.center_y)
				theta1_bear = angle_between(v1,v2) #calculating the angle between first two vectors in degree
				theta1_bear = (int)(math.degrees(theta1_bear))
				theta2_bear = angle_between(v3,v4) #calculating the angle between last two vectors in degree	
				theta2_bear = (int)(math.degrees(theta2_bear))
				val_bear = intention_object(theta1_bear,prev_theta1_bear,theta2_bear,prev_theta2_bear)
			if done[4] == 1 and v_ball!=-1:
				
				cup_chance =0
				scissor_chance = 0
				clock_chance = 0
				ball_chance = 0
				bear_chance = 0
				v_ball = -1
				hand_fix_flag = 2
				

			if done[4] ==1:
				val_ball = -1
				
			else:

				calculating_vectors(ball2.center_x,ball2.center_y,ball1.center_x,ball1.center_y,hand_x_avg,hand_y_avg)
				prev_theta1_ball = theta1_ball
				prev_theta2_ball = theta2_ball
				
				temp_val_x = ( ball2.center_x - ball1.center_x )/ 2
				temp_val_y =  ball2.center_y 
				distance[4] = distance_object(temp_val_x,temp_val_y,hand.center_x,hand.center_y)
				theta1_ball = angle_between(v1,v2) #calculating the angle between first two vectors in degree
				theta2_ball = angle_between(v3,v4) #calculating the angle between last two vectors in degree	
				theta2_ball = (int)(math.degrees(theta2_ball))
				val_ball = intention_object(theta1_ball,prev_theta1_ball,theta2_ball,prev_theta2_ball)
			
			
			val_list = [val_cup,val_scissor,val_clock,val_bear,val_ball]
			m= [ i for i,v in enumerate(val_list) if v==max(val_list) ]
			if len(m) == 1: 
			
				if val_cup!= -1 and val_cup>val_scissor and val_cup>val_clock and val_cup>val_bear and val_cup>val_ball:
					
					draw_box = 1
					
					cup_chance = cup_chance+1
					
						
				elif val_scissor!= -1 and val_scissor>val_cup and val_scissor>val_clock and val_scissor>val_bear and val_scissor>val_ball:
					
					draw_box = 2
					scissor_chance = scissor_chance + 1
					
					
				elif val_clock!= -1 and val_clock>val_cup and val_clock>val_scissor and val_clock>val_bear and val_clock>val_ball:
					
					draw_box = 2
					clock_chance = clock_chance + 1
					
				elif val_bear!= -1 and val_bear>val_cup and val_bear>val_scissor and val_bear>val_clock and val_bear>val_ball:
					
					draw_box = 2
					bear_chance = bear_chance + 1
					
				elif val_ball!= -1 and val_ball>val_cup and val_ball>val_scissor and val_ball>val_clock and val_ball>val_bear:
					
					draw_box = 2
					ball_chance = ball_chance + 1
					
				
			elif len(m)>1:
				if max(val_list)!=0 and max(val_list)!=-1:
					
					draw_box = 0
					if val_cup==max(val_list) and cup_chance>scissor_chance and cup_chance>clock_chance and cup_chance>bear_chance and cup_chance>ball_chance:
						cup_chance=cup_chance+1
						
							
					if val_scissor == max(val_list) and scissor_chance>cup_chance and scissor_chance>clock_chance and scissor_chance>bear_chance and scissor_chance>ball_chance:
						scissor_chance=scissor_chance+1
						
					
					if val_clock == max(val_list) and clock_chance>cup_chance and clock_chance>scissor_chance and clock_chance>bear_chance and clock_chance>ball_chance:
						clock_chance=clock_chance+1
						
						
					if val_bear == max(val_list) and bear_chance>cup_chance and bear_chance>scissor_chance and bear_chance>clock_chance and bear_chance>ball_chance:
						bear_chance=bear_chance+1
						
					if val_ball == max(val_list) and ball_chance>cup_chance and ball_chance>scissor_chance and ball_chance>clock_chance and ball_chance>bear_chance:
						ball_chance=ball_chance+1
						
						
				elif max(val_list)==0:
					
					draw_box = 0
					
						
				elif max(val_list)==-1:
					print "DONE"				
		
		
			hand_x_sum = 0;
			hand_y_sum = 0;
		
		count = count + 1
	notify_callback()
		

def notify_callback():
	global started,object_started_flag
	chance_list = [cup_chance,scissor_chance,clock_chance,bear_chance,ball_chance]
	max_first = max(chance_list)
	max_first_index = chance_list.index(max(chance_list))
	chance_list.pop(max_first_index)
	chance_list.insert(max_first_index,0)
	max_second = max(chance_list)
	max_second_index = chance_list.index(max_second)
	if object_started_flag ==0 and max_first - max_second >5:
		object_started_flag ==1
		started[max_first_index]=1
	with open("example5.txt", "a") as f:
		f.write("\nnotify_callback chance_value %d %d %d %d %d %d"%(count,cup_chance,scissor_chance,clock_chance,bear_chance,ball_chance))
	with open("example5.txt", "a") as f:
		f.write("\nnotify_callback started_value %d %d %d %d %d %d"%(count,started[0],started[1],started[2],started[3],started[4]))
	with open("example5.txt", "a") as f:
		f.write("\nnotify_callback done_value %d %d %d %d %d %d"%(count,done[0],done[1],done[2],done[3],done[4]))

	cup_msg = ObjStatus()
	cup_msg.chance = cup_chance
	cup_msg.started = started[0]
	cup_msg.done = done[0]
	scissors_msg = ObjStatus()
	scissors_msg.chance = scissor_chance
	scissors_msg.started = started[1]
	scissors_msg.done = done[1]
	clock_msg = ObjStatus()
	clock_msg.chance = clock_chance
	clock_msg.started = started[2]
	clock_msg.done = done[2]
	bear_msg = ObjStatus()
	bear_msg.chance = bear_chance
	bear_msg.started = started[3]
	bear_msg.done = done[3]
	ball_msg = ObjStatus()
	ball_msg.chance = ball_chance
	ball_msg.started = started[4]
	ball_msg.done = done[4]
	
	cup_status_pub.publish(cup_msg)
	scissors_status_pub.publish(scissors_msg)
	clock_status_pub.publish(clock_msg)
	bear_status_pub.publish(bear_msg)
	ball_status_pub.publish(ball_msg)


def image_callback(msg_img):
	global draw_box,cup_chance,scissor_chance,clock_chance,bear_chance,hand_fix_flag,distance,ball_chance
	
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(msg_img, 'bgr8')
	with open("example4.txt", "a") as f:
		f.write("\nimage_callback %d %d %d %d %d flag=%d"%(count,cup_chance,scissor_chance,clock_chance,bear_chance,hand_fix_flag))
	

	
	if done[0] ==0 and started[0]==1:
			cv2.rectangle(cv_image, (cup_loc_save.xmin,cup_loc_save.ymin), (cup_loc_save.xmax,cup_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	
	elif done[1] ==0 and started[1]==1:
			
			cv2.rectangle(cv_image, (scissor_loc_save.xmin,scissor_loc_save.ymin), (scissor_loc_save.xmax,scissor_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	
	elif done[2] ==0 and started[2]==1:
			
	
			cv2.rectangle(cv_image, (clock_loc_save.xmin,clock_loc_save.ymin), (clock_loc_save.xmax,clock_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	elif done[3] ==0 and started[3]==1:
			
			cv2.rectangle(cv_image, (bear_loc_save.xmin,bear_loc_save.ymin), (bear_loc_save.xmax,bear_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	elif done[4] ==0 and started[4]==1:
			
			cv2.rectangle(cv_image, (ball_loc_save.xmin,ball_loc_save.ymin), (ball_loc_save.xmax,ball_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	
	elif draw_box == 0:
		cv2.rectangle(cv_image, (100,100), (200,200), (255,255,255), 1, 8, 0)
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(1)
def get_intention():
	

    	
	sub_center = rospy.Subscriber('hand_bounding_box_center', bounding_box_calculated_center, center_callback) #subscribing the topic which publishing hand center in 3d   
    	sub_img = rospy.Subscriber('/kinect2/qhd/image_color', Image, image_callback)

	
	
	
	




   	 


def usage():
    return "%s [obj_name]"%sys.argv[0]

if __name__ == "__main__":
    '''if len(sys.argv) == 1:
       # obj_name = sys.argv[1]
	obj_name = "person"
    else:
        print usage()
        sys.exit(1)'''
  
    rospy.init_node('get_obj_loc_client')
    get_intention()

    	 
	
 
    rospy.spin()	
    	
	
	
