#!/usr/bin/env python
import rospy
import sys
import time
import math
import os
import numpy as np
import cv2
# from typing import NamedTuple

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

theta1 = [0, 0, 0]
theta2 = [0, 0, 0]
prev_theta1 = [0, 0, 0]
prev_theta2 = [0, 0, 0]

count = 0
flag = 1
hand_x_sum = 0
hand_x_avg = 0
hand_y_avg = 0
hand_y_sum = 0

hand_count = 0

prev_hand_x_sum = 0
prev_hand_y_sum = 0

hand_loc = []
object_loc = []
v1 = []
v2 = []
v3 = []
v4 = []
obj_name = []
obj_loc_save = []

draw_box = 0
obj_chance = [0, 0, 0]
v_obj = [0, 0, 0]
val_object = [0,0,0]


class location:
    center_x_1 = 0
    center_y_1 = 0
    center_x_2 = 0
    center_y_2 = 0


class location_center:
    center_x = 0
    center_y = 0


hand = location_center()
object_temp = location()

fix_obj_flag = [0, 0, 0]

hand_change = 0
obj_check = [1, 1, 1]


def center_callback(hand_center):  # storing the hand location in 3d

    global hand, flag
    hand.center_x = hand_center.x
    hand.center_y = hand_center.y
    r_val = get_obj_loc_client()
    callback(r_val)


#

def calculating_midpoint(msg):
    obj = location()
    obj.center_x = msg.xmin;
    obj.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
    obj.center_x = msg.xmax;
    obj.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
    return obj


def get_obj_loc_client():  # service call to find the cup location from YOLO

    rospy.wait_for_service('get_object_loc')

    try:
        get_obj_loc = rospy.ServiceProxy('get_object_loc', GetObjLoc)
        global cup_loc_save, bowl_loc_save, clock_loc_save, hand_loc, hand_count, object_loc, obj_name,object_temp
        global fix_obj_flag, cup_fix_loc, ban_fix_loc_1, ban_fix_loc_2, clock_fix_loc_1, clock_fix_loc_2, bear_fix_loc_1, bear_fix_loc_2, obj_check
        with open("example3.txt", "a") as f:
            f.write("*%d %d %dcheckkkkk\n" % (obj_check[0], obj_check[1], obj_check[2]))
        object_loc = []

        if (30<=hand.center_x<=40  and 50<=hand.center_y<= 60)==0:
            for x in range(0, 3):
                print "%d"%(obj_check[x])
                if obj_check[x] == 1:
                    msg = get_obj_loc(obj_name[x])
                    print"%s" % (obj_name[x])
                    if msg.Class == obj_name[x]:
                        object_temp = location()
                        object_temp.center_x_1 = msg.xmin;
                        object_temp.center_y_1 = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
                        object_temp.center_x_2 = msg.xmax;
                        object_temp.center_y_2 = msg.ymin + (msg.ymaxx - msg.ymin) / 2;

                        object_loc.append(object_temp)

                        if fix_obj_flag[x] == 0:
                            # cup_fix_loc_2 = cup2
                            obj_loc_save.append(msg)
                            fix_obj_flag[x] = 1

                        if msg.xmin < obj_loc_save[x].xmin - 10 or msg.xmin > obj_loc_save[x].xmin + 10 or msg.ymin <obj_loc_save[x].ymin - 10 or msg.ymin > obj_loc_save[x].ymin + 10:
                            # if (cup_fix_loc_1.center_x-10 <=cup1.center_x<=cup_fix_loc_1.center_x + 10 and cup_fix_loc_1.center_y-10 <=cup1.center_y<=cup_fix_loc_1.center_y + 10)==0:

                            fix_obj_flag[x] = 2
                            obj_check[x] = 0

                            hand_loc = [];
                            hand_count = 0;
                            hand_count = 0;

                            print
                            "cup loc change"





                    elif msg.Class == ' ':

                        fix_obj_flag[x] = 2
                        cup_check = 0
                        hand_loc = [];
                        hand_count = 0;

                        print
                        "cup loc change"
                        with open("example2.txt", "a") as f:
                            f.write("I didn't find cup location. my bad")
            for x in range(0,3):
                    print "cup center!*#*:%d %.2f %.2f" % (x, object_loc[x].center_x_1, object_loc[x].center_y_1)





    except rospy.ServiceException, e:
        print        "Service call failed: %s" % e
    return 1


def unit_vector(vector):  # calculating angle between two vectors
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    with open("example2.txt", "a") as f:
        f.write("%s %s\n" % (v1, v2))

    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return round(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)), 2)


def calculating_vectors(objx2, objy2, objx1, objy1, handx, handy):
    global v1, v2, v3, v4
    v1 = []
    v2 = []
    v3 = []
    v4 = []
    v1.append(objx2 - objx1)  # vector between obj1 -> obj2
    v1.append(objy2 - objy1)

    v2.append(handx - objx1)  # vector between obj1 -> hand
    v2.append(handy - objy1)

    v3.append(objx1 - objx2)  # vector between obj2 -> obj1
    v3.append(objy1 - objy2)

    v4.append(handx - objx2)  # vector between obj2 -> hand
    v4.append(handy - objy2)


def intention_object(theta1, prev_theta1, theta2, prev_theta2):
    if theta1 == prev_theta1 and theta2 == prev_theta2:

        val = 1
    elif theta1 < prev_theta1 and theta2 < prev_theta2:
        val = 2
    elif theta1 < prev_theta1 and theta2 == prev_theta2:
        val = 2
    elif theta1 <= prev_theta1 and theta2 < prev_theta2:
        val = 2
    elif theta1 == prev_theta1 and theta2 < prev_theta2:
        val = 2
    else:
        val = 0
    print
    "helloooooo"
    return val


def callback(r_val):
    global count, theta1, theta2, hand_count, hand_x_sum, hand_y_sum, hand_loc, object_loc, prev_hand_x_avg, prev_hand_y_avg, hand_x_avg, hand_y_avg, hand, v1, v2, v3, v4
    # time.sleep(0.01)
    global draw_box, obj_chance, v_obj, val_object

    # print "hand center: %.2f %.2f %.2f"%(hand.center_x,hand.center_y,hand.center_z)

    if (30 <= hand.center_x <= 40 and 50 <= hand.center_y <= 60) == 0:

        # if hand_loc_save.center_x -10 <=hand.center_x<=hand_loc_save.center_x + 10 and hand_loc_save.center_y -10 <=hand.center_y<=hand_loc_save.center_y + 10:
        for x in range(0, 3):
            print "cup center***:%.2f %.2f" % (object_loc[x].center_x_1, object_loc[x].center_y_1)

        #print        "hand center: %.2f %.2f" % (hand.center_x, hand.center_y)
        print        "%d" % (count)
        #print        "cup center:%.2f %.2f" % (object_loc[0].center_x_1, object_loc[0].center_y_1)
        #print        "cup center: %.2f %.2f " % (object_loc[0].center_x_2, object_loc[0].center_y_2)
        #print        "bowl center:%.2f %.2f " % (object_loc[1].center_x_1, object_loc[1].center_y_1)
        #print        "bowl center: %.2f %.2f " % (object_loc[1].center_x_2, object_loc[1].center_y_2)

        hand_loc.append(hand);  # storing the hand location in a list for average

        hand_x_sum = hand_x_sum + hand_loc[hand_count].center_x;
        hand_y_sum = hand_y_sum + hand_loc[hand_count].center_y;

        with open("example2.txt", "a") as f:
            f.write('hand center(not avg):%.2f %.2f %d\n' % (
            hand_loc[hand_count].center_x, hand_loc[hand_count].center_y, hand_count))
        hand_count = hand_count + 1;

        if count % 2 == 0:  # average the hand values after 10 rounds

            prev_hand_x_avg = hand_x_avg
            prev_hand_y_avg = hand_y_avg
            hand_x_avg = hand_x_sum / 2
            hand_y_avg = hand_y_sum / 2

            with open("example2.txt", "a") as f:
                f.write('count %d\ncup center: %.2f %.2f \ncup center: %.2f %.2f \nhand center:%.2f %.2f \n' % (
                count, object_loc[0].center_x_1, object_loc[0].center_y_1, object_loc[0].center_x_2,object_loc[0].center_y_2, hand_x_avg, hand_y_avg))
            with open("example2.txt", "a") as f:
                f.write('count %d\nbanana center: %.2f %.2f \nbanana center: %.2f %.2f \n' % (
                count, object_loc[1].center_x_1, object_loc[1].center_y_1, object_loc[1].center_x_2,object_loc[1].center_y_2))
            print "hand avg center: %.2f %.2f " % (hand_x_avg, hand_y_avg)

            for x in range(0, 3):
                if obj_check[x] == 0 and v_obj[x] != -1:
                    obj_chance = [0, 0, 0]
                    v_obj[x] = -1

                if obj_check[x] == 0:
                    val_object[x] = -1
                else:
                    calculating_vectors(object_loc[x].center_x_2, object_loc[x].center_y_2, object_loc[x].center_x_1,object_loc[x].center_y_1, hand_x_avg, hand_y_avg)

                    prev_theta1[x] = theta1[x]
                    prev_theta2[x] = theta2[x]

                    # if not (hand_x_avg>=prev_hand_x_avg-1 and hand_x_avg<=prev_hand_x_avg+1 and hand_y_avg>=prev_hand_y_avg-1 and hand_y_avg<=prev_hand_y_avg+1):

                    # distance_cup_hand2 = round(math.sqrt((cup2.center_y-hand_y_avg)*(cup2.center_y-hand_y_avg) + (cup2.center_x-hand_x_avg)*(cup2.center_x-hand_x_avg)),2)
                    theta1[x] = angle_between(v1, v2)  # calculating the angle between first two vectors in degree
                    theta1[x] = (int)(math.degrees(theta1[x]))
                    theta2[x] = angle_between(v3, v4)  # calculating the angle between last two vectors in degree
                    theta2[x] = (int)(math.degrees(theta2[x]))
                    val_object[x] = intention_object(theta1[x], prev_theta1[x], theta2[x], prev_theta2[x])

            '''if bear_check == 0:
				val_bear = 0
			else:
				val_bear = intention_object(theta1_bear,prev_theta1_bear,theta2_bear,prev_theta2_bear)'''
            with open("example3.txt", "a") as f:
                f.write("\nvalue list: %d %d %d\n" % (val_object[0], val_object[1], val_object[2]))
            # val_list = [val_cup,val_ban,val_clock,val_bear]
            # val_list = [val_cup,val_ban,val_clock]
            m = [i for i, v in enumerate(val_object) if v == max(val_object)]
            if len(m) == 1:

                if val_object[0] != -1 and val_object[0] > val_object[1] and val_object[0] > val_object[2]:
                    print"going to the cup%d " % (count)
                    draw_box = 1

                    obj_chance[0] = obj_chance[0] + 1
                    with open("example3.txt", "a") as f:
                        f.write("\ngoing to the bottle %d" % (count))

                elif val_object[1] != -1 and val_object[1] > val_object[0] and val_object[1] > val_object[2]:
                    print
                    "going to the cbowl%d " % (count)
                    draw_box = 2
                    obj_chance[1] = obj_chance[1] + 1
                    with open("example3.txt", "a") as f:
                        f.write("\ngoing to the clock %d" % (count))

                elif val_object[2] != -1 and val_object[2] > val_object[0] and val_object[2] > val_object[1]:
                    print
                    "going to the cbowl%d " % (count)
                    draw_box = 2
                    obj_chance[2] = obj_chance[2] + 1
                    with open("example3.txt", "a") as f:
                        f.write("\ngoing to the clock %d" % (count))


            elif len(m) > 1:
                if max(val_object) != 0 and max(val_object) != -1:
                    print
                    "confused%d " % (count)
                    draw_box = 0
                    if val_object[0] == max(val_object) and obj_chance[0] > obj_chance[1] and obj_chance[0] >obj_chance[2]:
                        obj_chance[0] = obj_chance[0] + 1
                        with open("example3.txt", "a") as f:
                            f.write("\ngoing to the bottle %d" % (count))

                    elif val_object[1] == max(val_object) and obj_chance[1] > obj_chance[0] and obj_chance[1] >obj_chance[0]:
                        obj_chance[1] = obj_chance[1] + 1
                        with open("example3.txt", "a") as f:
                            f.write("\ngoing to the clock %d" % (count))

                    elif val_object[2] == max(val_object) and obj_chance[2] > obj_chance[0] and obj_chance[2] > obj_chance[1]:
                        obj_chance[2] = obj_chance[2] + 1
                        with open("example3.txt", "a") as f:
                            f.write("\ngoing to the clock %d" % (count))


                    else:

                    	with open("example3.txt", "a") as f:
                        	f.write("\nboth same %d" % (count))

                elif max(val_object[0]) == 0:
                    print
                    "I guess it's going away%d " % (count)
                    draw_box = 0
                    with open("example3.txt", "a") as f:
                        f.write("\nGOING AWAY %d" % (count))

                elif max(val_object) == -1:
                    draw_box = 0
                    print
                    "DONE"

            # hand_loc = [];
            # hand_count = 0;
            hand_x_sum = 0;
            hand_y_sum = 0;

        #	else:
        #	print"one class is blank"
        count = count + 1

    else:
        print
        "location change"


def image_callback(msg_img):
    global draw_box, obj_chance
    print
    "hei hei hei"
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg_img, 'bgr8')
    with open("example3.txt", "a") as f:
        f.write("\nimage_callback %d %d %d %d" % (count, obj_chance[0], obj_chance[1], obj_chance[2]))
    if obj_chance[0] == 0 and obj_chance[1] == 0 and obj_chance[2] == 0:
        cv2.rectangle(cv_image, (100, 100), (200, 200), (255, 255, 255), 1, 8, 0)
    elif obj_check[0] == 1 and obj_chance[0] > obj_chance[1] and obj_chance[0] > obj_chance[2]:
        cv2.rectangle(cv_image, (obj_loc_save[0].xmin, obj_loc_save[0].ymin),
                      (obj_loc_save[0].xmax, obj_loc_save[0].ymaxx), (255, 255, 255), 4, 8, 0)
    elif obj_check[1] == 1 and obj_chance[1] > obj_chance[0] and obj_chance[1] > obj_chance[2]:
        cv2.rectangle(cv_image, (obj_loc_save[1].xmin, obj_loc_save[1].ymin),
                      (obj_loc_save[1].xmax, obj_loc_save[1].ymaxx), (255, 255, 255), 4, 8, 0)
    elif obj_check[2] == 1 and obj_chance[2] > obj_chance[0] and obj_chance[2] > obj_chance[1]:
        cv2.rectangle(cv_image, (obj_loc_save[2].xmin, obj_loc_save[2].ymin),
                      (obj_loc_save[2].xmax, obj_loc_save[2].ymaxx), (255, 255, 255), 4, 8, 0)

    elif draw_box == 0:
        cv2.rectangle(cv_image, (100, 100), (200, 200), (255, 255, 255), 1, 8, 0)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)


def get_intention():
    # sub_center_object = rospy.Subscriber('chatter', msg2dto3d_object, center_callback_object) #subscribing the topic which publishing 3d cup points
    # sub_center = rospy.Subscriber('hand_bounding_box_center', bounding_box_calculated_center, center_callback) #subscribing center of a hand box from skinDetection
    sub_center = rospy.Subscriber('hand_bounding_box_center', bounding_box_calculated_center,
                                  center_callback)  # subscribing the topic which publishing hand center in 3d
    sub_img = rospy.Subscriber('/kinect2/qhd/image_color', Image, image_callback)


# if change !=0 and change2!=0:
# cup_fix1=cup1
# cup_fix2=cup2
# ban_fix1=ban1
# ban_fix2=ban2


def usage():
    return "%s [obj_name]" % sys.argv[0]


if __name__ == "__main__":
	rospy.init_node('get_obj_loc_client')

	obj_name = ["scissors", "bowl", "clock"]
	get_intention()

# print "Requesting: %s"%(obj_name)


	rospy.spin()

# time.sleep(0.5)
